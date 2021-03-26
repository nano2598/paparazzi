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

#ifndef PAPARAZZI_MAV_EXERCISE_H
#define PAPARAZZI_MAV_EXERCISE_H

extern void mav_exercise_init(void);
extern void mav_exercise_periodic(void);
extern float div_thresh;
extern float heading_increment;
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]
extern double Kp;
extern double Kd;
extern double Kyd;
extern float yaw_thresh;
extern float dr_vel;
extern float of_diff_thresh;
extern float green_thresh;
extern float divergence_thresh;
extern int robust;
#endif //PAPARAZZI_MAV_EXERCISE_H
