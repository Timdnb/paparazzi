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
 * @file modules/computer_vision/cnn.h
 */

#ifndef CNN_H
#define CNN_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Tensor dimensions
#define TENSOR_HEIGHT 40
#define TENSOR_WIDTH 12

// Dataset mean and std
#define MEAN 0.3625994920730591
#define STD 0.18103595077991486

// Module functions
extern void cnn_init(void);

void entry(const float tensor_input_1[1][1][40][12], float tensor_19[1][3]);

void convert_image_to_tensor(struct image_t *image, float tensor_input[1][1][TENSOR_HEIGHT][TENSOR_WIDTH]);

#endif /* CNN_H */
