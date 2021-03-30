/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2016 Kimberly McGuire <k.n.mcguire@tudelft.nl
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

/**
 * @file modules/computer_vision/opticflow/opticflow_calculator.h
 * @brief Calculate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */



#ifndef OPTICFLOW_CALCULATOR_H
#define OPTICFLOW_CALCULATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#include "std.h"
#include "inter_thread_data.h"
#include "lib/vision/image.h"
#include "lib/v4l/v4l2.h"

struct opticflow_t {
  bool got_first_img;                 ///< If we got a image to work with
  bool just_switched_method;        ///< Boolean to check if methods has been switched (for reinitialization)
  struct image_t img_gray;              ///< Current gray image frame
  struct image_t prev_img_gray;         ///< Previous gray image frame
  uint16_t window_size;               ///< Window size for the blockmatching algorithm (general value for all methods)
  float pyr_scale;
  uint16_t levels;
  uint16_t poly_n;
  float poly_sigma;
  uint16_t flags;
  uint8_t max_iterations;               ///< The maximum amount of iterations the Lucas Kanade algorithm should do
};

extern void opticflow_calc_init(struct opticflow_t *opticflow);
extern bool opticflow_calc_frame_dense(struct opticflow_t *opticflow, struct image_t *img, double *of_diff, float *divg);
extern bool calc_farneback(struct opticflow_t *opticflow, struct image_t *img, double *of_diff, float *divg);

#endif /* OPTICFLOW_CALCULATOR_H */


