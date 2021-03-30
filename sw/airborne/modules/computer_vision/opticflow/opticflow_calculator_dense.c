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
 * @file modules/computer_vision/opticflow/opticflow_calculator.c
 * @brief Estimate velocity from optic flow.
 *
 */

#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

// Own Header
#include "opticflow_calculator_dense.h"

// Computer Vision
#include "lib/vision/image.h"
#include "modules/sonar/agl_dist.h"

#include "get_flow.h"

// to get the definition of front_camera / bottom_camera
#include BOARD_CONFIG

#ifndef OPTICFLOW_WINDOW_SIZE
#define OPTICFLOW_WINDOW_SIZE 10
#endif
PRINT_CONFIG_VAR(OPTICFLOW_WINDOW_SIZE)

#ifndef OPTICFLOW_PYR_SCALE
#define OPTICFLOW_PYR_SCALE 0.1
#endif
PRINT_CONFIG_VAR(OPTICFLOW_PYR_SCALE)

#ifndef OPTICFLOW_LEVELS
#define OPTICFLOW_LEVELS 3
#endif
PRINT_CONFIG_VAR(OPTICFLOW_LEVELS)

#ifndef OPTICFLOW_POLY_N
#define OPTICFLOW_POLY_N 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_POLY_N)

#ifndef OPTICFLOW_POLY_SIGMA
#define OPTICFLOW_POLY_SIGMA 3.0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_POLY_SIGMA)

#ifndef OPTICFLOW_FLAGS
#define OPTICFLOW_FLAGS 0
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FLAGS)

#ifndef OPTICFLOW_MAX_ITERATIONS
#define OPTICFLOW_MAX_ITERATIONS 2
#endif
PRINT_CONFIG_VAR(OPTICFLOW_MAX_ITERATIONS)

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);
/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 */
void opticflow_calc_init(struct opticflow_t *opticflow)
{
  /* Set the default values */
  opticflow->window_size = OPTICFLOW_WINDOW_SIZE;
  opticflow->pyr_scale = OPTICFLOW_PYR_SCALE;
  opticflow->levels = OPTICFLOW_LEVELS;
  opticflow->poly_n = OPTICFLOW_POLY_N;
  opticflow->poly_sigma = OPTICFLOW_POLY_SIGMA;
  opticflow->flags = OPTICFLOW_FLAGS;
  opticflow->max_iterations = OPTICFLOW_MAX_ITERATIONS;

}

bool calc_farneback(struct opticflow_t *opticflow, struct image_t *img, double *of_diff, float *div)
{
	if (!opticflow->got_first_img) {
		image_create(&opticflow->img_gray, img->w, img->h, IMAGE_YUV422);
		image_create(&opticflow->prev_img_gray, img->w, img->h, IMAGE_YUV422);
		image_copy(img, &opticflow->prev_img_gray);
		opticflow->got_first_img = true;
		return false;
	}
	struct image_t flow;
	image_copy(img, &opticflow->img_gray);

	clock_t start, end;
	double cpu_time_used;

	start = clock();

	get_flow(opticflow->prev_img_gray.buf, opticflow->img_gray.buf, opticflow->pyr_scale, opticflow->levels, opticflow->window_size,
	  opticflow->max_iterations, opticflow->poly_n, opticflow->poly_sigma, opticflow->flags,
	  of_diff, div, img->w, img->h);
	end = clock();
	cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
	PRINT("Time taken %lf \n", cpu_time_used);
	image_copy(&opticflow->prev_img_gray, img);
	image_switch(&opticflow->img_gray, &opticflow->prev_img_gray);

  return true;
}

/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
bool opticflow_calc_frame_dense(struct opticflow_t *opticflow, struct image_t *img,
		double *of_diff, float *div)
{
  // Attempted to use calc_fast9_lucas_kanade as template
  bool flow_successful = false;
  flow_successful = calc_farneback(opticflow, img, of_diff, div);

  return flow_successful;
}

/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
  msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
  return msec;
}
