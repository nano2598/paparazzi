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
# define KPI 0.000018
# endif

# ifndef KDI
# define KDI 0.000001
# endif

# ifndef KYD
# define KYD 0.0
# endif

# ifndef YI
# define YI 1.f
# endif

# ifndef VI
# define VI 1.f
# endif

# ifndef DIVI
# define DIVI 0.0001f
# endif

# ifndef OFDI
# define OFDI 8000
# endif

# ifndef GRDI
# define GRDI 1500
# endif

# ifndef DVDI
# define DVDI 900
# endif

# ifndef RBST
# define RBST 4
# endif

enum navigation_state_t {
  SAFE,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  GOBACK,
  TURN
};

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;                //Need to remove this before hand-in
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)

float div_1 = 0.f;
float div_thresh = 0.f;
float divergence_thresh = DVDI;
double Kp = KPI;
double Kd = KDI;
double Kyd = KYD;
bool rotated = 0;
int count = 0;
float yaw_rate = 0;
float green_thresh = GRDI;
float of_diff_thresh = OFDI;
double of_diff;                          // difference in optical flow between right and left side
double of_diff_prev = 0;
float yaw_thresh = YI;
float dr_vel = VI;
int count_backwards=0;
int count_robust=0;
int count_oob=0;
int robust = RBST;
float heading_increment = 0.f;
float oa_color_count_frac = 0.18f;
float oag_floor_count_frac = 0.05f;
int16_t obstacle_free_confidence = 0;
float oob_haeding_increment = 5.f;
float avoidance_heading_direction = 1.f;
const int16_t max_trajectory_confidence = 5;
// needed to receive output from a separate module running on a parallel process

#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif //Need to remove this before hand-in

#ifndef FLOW_OPTICFLOW_ID
#define FLOW_OPTICFLOW_ID ABI_BROADCAST
#endif

#ifndef FLOOR_VISUAL_DETECTION_ID
#define FLOOR_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width,
                               int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra) {
  color_count = quality;
} //Need to remove this before hand-in


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
                               float __attribute__((unused)) noise_measurement, float div_size, double value) {
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
    navigation_state = SAFE;
    return;
  }

  PRINT("OF difference: %f \n", of_diff);
  PRINT("OF difference prev: %f \n", of_diff_prev);
  PRINT("Yaw rate: %f \n", stateGetBodyRates_f()->r);
  PRINT("Divergence value is: %f \n",div_1);
  PRINT("Green count value is: %f \n",floor_count);
  PRINT("Navigation state is: %d \n",navigation_state);


  switch (navigation_state) {
    case SAFE:
      // If not inside, state to OOB
      if (!InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y ))
      {
        navigation_state = OUT_OF_BOUNDS;
      }
      // If divergence is above threshold several times, go back
      else if(div_1 > divergence_thresh)
      {
        count_robust++;
        if(count_robust >= robust)
        {
          navigation_state = GOBACK;
        }
      }
      // Otherwise if optical flow difference is above threshold, switch to turning state
      else if(fabs(of_diff)>of_diff_thresh)
      {
          count_robust = 0;
          navigation_state = TURN;
      }
      // Otherwise just go forward
      else
      {
          count_robust = 0;
          yaw_rate =0;
          PRINT("NO ROTATION \n");
          guidance_h_set_guided_body_vel(dr_vel, 0);
      }

      break;

    case TURN:
      if (!InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y ))
      {
        navigation_state = OUT_OF_BOUNDS;
      }
      // If optical flow difference is below threshold, go back to safe state
      if(fabs(of_diff)<of_diff_thresh)
      {
        navigation_state = SAFE;
      }
      // Setting yaw rate to gain * optical flow difference, clipping at maximum and minimum yaw values
      yaw_rate =  -Kp * of_diff + Kd * (of_diff - of_diff_prev);
      if(yaw_rate > yaw_thresh){yaw_rate = yaw_thresh;}
      else if(yaw_rate < -yaw_thresh){yaw_rate = -yaw_thresh;}
      guidance_h_set_guided_heading_rate(yaw_rate);
      guidance_h_set_guided_body_vel(dr_vel, 0);
      break;

    case GOBACK:
      if (!InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y )) {
          navigation_state = OUT_OF_BOUNDS;
        }
      // Go backwards for two counts
      else if(count_backwards<=2)
      {
          guidance_h_set_guided_body_vel(-2, 0);
          yaw_rate =0;
          count_backwards++;
          PRINT("GO BACK \n");
      }
      // Then switch to OOB state, reset counter
      else
      {
        navigation_state = OUT_OF_BOUNDS;
        count_backwards=0;
      }
      break;

    case OUT_OF_BOUNDS:
      // Always stopping
      guidance_h_set_guided_body_vel(0, 0);
      // On 'first' loop, reverse a bit and set target heading
      if(count_oob == 0)
      {

          guidance_h_set_guided_body_vel(-1, 0);
      	  guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi + RadOfDeg(160));
      }
      // Essentially a counter to make it wait so that it doesn't immediately go to another state
      if(count_oob > 10)
      {
    	  navigation_state = REENTER_ARENA;
    	  count_oob=0;
      }
      else
      {
    	  count_oob++;
      }
      break;

    case REENTER_ARENA:
      // Keep going until back inside, then switch back to safe state
      if(!InsideObstacleZone(stateGetPositionEnu_f()->x , stateGetPositionEnu_f()->y ))
      {
    	  guidance_h_set_guided_body_vel(dr_vel, 0);
      }
      else
      {
    	  navigation_state = SAFE;
      }
      break;



    default:
      break;



  }
  of_diff_prev = of_diff;
  return;
}
