/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "mav_exercise.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot_static.h"
#include "firmwares/rotorcraft/autopilot_guided.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/guidance.h"
#include <stdio.h>
#include <time.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)


# ifndef KPI
# define KPI 0.000001
# endif

# ifndef KDI
# define KDI 0.000001
# endif

# ifndef YI
# define YI 2.f
# endif

# ifndef VI
# define VI 2.f
# endif
# ifndef DIVI
# define DIVI 0.0001f
# endif

# ifndef OFDI
# define OFDI 5000
# endif
//uint8_t increase_nav_heading(float incrementDegrees);
//uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);

//uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define and initialise global variables
float oa_color_count_frac = 0.18f;
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
//float moveDistance = 2;                 // waypoint displacement [m]
float oob_haeding_increment = 5.f;      // heading angle increment if out of bounds [deg]
float avoidance_heading_direction = 1.f;  // heading change direction for avoidance [rad/s]

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free
float heading_increment = 20.f;          // heading angle increment [deg]
float div_1 = 0.f;
float div_thresh = 100 * 5.f;
double Kp = KPI;
double Kd = KDI;
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
bool rotated = 0;
int count = 0;
float yaw_rate = 0;
float of_diff_thresh = OFDI;
float of_diff ; // difference in optical flow between right and left side
float of_diff_prev = 0;
float yaw_thresh = YI;
float dr_vel = YI;
int count_backwards=0;
// needed to receive output from a separate module running on a parallel process

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

#ifndef FLOW_OPTICFLOW_ID
# define FLOW_OPTICFLOW_ID ABI_BROADCAST
# endif

#ifndef FLOOR_VISUAL_DETECTION_ID
# define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
# endif

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width,
                               int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra) {
  color_count = quality;
}


static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

static abi_event optical_flow_ev;
static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id,
                               uint32_t __attribute__((unused)) now_ts, int16_t __attribute__((unused)) flow_x,int16_t __attribute__((unused)) flow_y,
                               int16_t __attribute__((unused)) flow_der_x,
                               int16_t __attribute__((unused)) flow_der_y,
                               float __attribute__((unused)) noise_measurement, float div_size, float value) {
  div_1 = div_size;
  of_diff = value;
}


void mav_exercise_init(void) {
  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &optical_flow_ev, optical_flow_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

void mav_exercise_periodic(void) {
  // only evaluate our state machine if we are flying

  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }
  /*
  if (!autopilot_in_flight()) {
    return;
  }
  */

  // compute current color thresholds
  // front_camera defined in airframe xml, with the video_capture module
  int32_t color_count_threshold = 10000 * oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  PRINT("Color_count: %d  threshold: %d state: %d \n", floor_count, color_count_threshold, navigation_state);
  PRINT("OF difference: %f \n", of_diff);
  PRINT("OF difference prev: %f \n", of_diff_prev);
  PRINT("Yaw rate: %f \n", stateGetBodyRates_f()->r);
  PRINT("Divergence value is: %f \n",div_1);
  // update our safe confidence using color threshold
  if (color_count < color_count_threshold) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  //Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
  //if navigation_state = OUT_OF_BOUNDS;

  switch (navigation_state) {
    case SAFE:
      //guidance_h_set_guided_body_vel(0, 0);
      //moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
  if(floor_count<0){
	if(count_backwards<=2)
{
       guidance_h_set_guided_body_vel(-2, 0);
       yaw_rate =0;
    count_backwards++;
PRINT("NO GREEN");
	break;
}
else{
      navigation_state = OUT_OF_BOUNDS;
      PRINT("NO GREEN");
count_backwards=0;
}
   }
    else  if(fabs(of_diff)>of_diff_thresh) 
    {
        yaw_rate =  -Kp * of_diff + Kd * (of_diff - of_diff_prev);
        if(yaw_rate > yaw_thresh)
          {yaw_rate = yaw_thresh;}
        else if(yaw_rate < -yaw_thresh)
          {yaw_rate = -yaw_thresh;}
        
    }

   
   else {
        yaw_rate =0;
PRINT("NO ROTATION");
         //guidance_h_set_guided_heading_rate(oag_heading_rate);
        }
   guidance_h_set_guided_heading_rate(yaw_rate);
     if (!InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y )) {
        navigation_state = OUT_OF_BOUNDS;
      }
      else if (obstacle_free_confidence == 0){
        count = 0;
        navigation_state = OBSTACLE_FOUND;
      }
       else {
        count = 0;
        guidance_h_set_guided_body_vel(dr_vel, 0);
      }
     of_diff_prev = of_diff;
      /*

      //moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

     if (!InsideObstacleZone(GetPosX(), GetPosY() )) {
	//setGuided();
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0) {
        navigation_state = OBSTACLE_FOUND;
      } else {
        //moveWaypointForward(WP_GOAL, moveDistance);

        guidance_h_set_guided_body_vel(1, 1);
      }
      of_diff_prev = of_diff;
      break;
    case OBSTACLE_FOUND:
      // TODO Change behavior


      guidance_h_set_guided_body_vel(0, 0);
      // stop as soon as obstacle is found
      /*
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);
      */
      //navigation_state = SEARCH_FOR_SAFE_HEADING;
      break;
    case SEARCH_FOR_SAFE_HEADING:
      //increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe

      //guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi + RadOfDeg(30.f));

      guidance_h_set_guided_heading_rate(oag_heading_rate);


      // make sure we have a couple of good readings before declaring the way safe

      if (obstacle_free_confidence >= 2){
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
       }

      /*
      if (obstacle_free_confidence >= 2){
        navigation_state = SAFE;
      }
      */
      break;
    case OUT_OF_BOUNDS:
      // stop

     guidance_h_set_guided_body_vel(0, 0);

      // start turn back into arena
      //guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(15));
     //guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(180));
     /*
     if (InsideObstacleZone(stateGetPositionEnu_f()->x,stateGetPositionEnu_f()->y)){
        // add offset to head back into arena

        guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(15));
        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = REENTER_ARENA;
       }
     */
      rotated = 0;
      if(count<10)
      {
      while(!rotated){
        rotated = autopilot_guided_goto_ned_relative (0,0,0,RadOfDeg(180));
      }
      }
       navigation_state = REENTER_ARENA;
      break;
    case REENTER_ARENA:

     //rotated = 0;

        //guidance_h_set_guided_heading_rate(oag_heading_rate);

    ++count;
     if(count>=10)
    {

     guidance_h_set_guided_body_vel(dr_vel, 0);
     navigation_state = SAFE;
    }

        // add offset to head back into arena

        //guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(15));

        // reset safe counter
       // obstacle_free_confidence = 0;

        // ensure direction is safe before continuing





      //guidance_h_set_guided_body_vel(0, 0);
	    // force floor center to opposite side of turn to head back into arena


	break;

    default:
      break;

  }
 return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */

/*
uint8_t increase_nav_heading(float incrementDegrees) {
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  return false;
}
*/

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */

/*
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters) {
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}
*/

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
/*
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}
*/
/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
/*
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}
*/
