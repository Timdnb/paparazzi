/*
 * Copyright (C) 2015
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/opencv_approach.c
 */

// Own header
#include "modules/computer_vision/opencv_approach.h"
#include <stdio.h>

// Include the image library -> used to alter the incoming image
#include "modules/computer_vision/lib/vision/image.h"

// Include the ABI library -> used to send messages to the ground station
#include "modules/core/abi.h"

#ifndef CNN_FPS
#define CNN_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif





using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"


struct image_t gray;

// uint8_t white[4] = {127, 255, 127, 255};

// Function
static struct image_t *opencv_func(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  // TODO: freeing of images ?
  // convert to grayscale, but also keep original for live feed
  
  image_create(&gray, img->w, img->h, IMAGE_GRAYSCALE);
  image_to_grayscale(img, &gray);

  // Convert image to tensor
  // IMPORTANT: order is now [right, forward, left]
  // float tensor_output[1][3];
  // float tensor_input[1][1][TENSOR_HEIGHT][TENSOR_WIDTH];
  // convert_image_to_tensor(&gray, tensor_input);

  
  // draw rectangles on the original image corresponding to the outputs
  // TODO: improve code
  // int x_min_left = 10;
  // int x_max_left = 10 + (int)(tensor_output[0][2] * 40);
  // int y_min_left = 10;
  // int y_max_left = 30;
  // image_draw_rectangle(img, x_min_left, x_max_left, y_min_left, y_max_left, white);

  // int x_min_forward = 10;
  // int x_max_forward = 10 + (int)(tensor_output[0][1] * 40);
  // int y_min_forward = 35;
  // int y_max_forward = 55;
  // image_draw_rectangle(img, x_min_forward, x_max_forward, y_min_forward, y_max_forward, white);

  //   int x_min_right = 10;
  // int x_max_right = 10 + (int)(tensor_output[0][0] * 40);
  // int y_min_rigth = 60;
  // int y_max_right = 80;
  // image_draw_rectangle(img, x_min_right, x_max_right, y_min_rigth, y_max_right, white);

  // Print outputs
  // printf("Right: %f, Forward: %f, Left: %f\n", tensor_output[0][0], tensor_output[0][1], tensor_output[0][2]);

  // Send outputs
  // printf("VALUES TO SEND: Forward=%f, Left=%f, Right=%f\n", .2, .6 , .2);
  AbiSendMsgCNN_CONTROL_INPUTS(CNN_CONTROL_INPUTS_ID, 0.2, .6, 0.2);

  return &gray;
}

void opencv_init(void)
{
  cv_add_to_device(&CNN_CAMERA, opencv_func, CNN_FPS, 0);
}

