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
#include "state.h"

#define IMAGE_WIDTH 520
#define Y_MAX 120

// namespace fs = std::filesystem;


bool Dhane_distortion(float x_n, float y_n, float* x_nd, float* y_nd, float k) {
  float R = sqrtf(x_n*x_n + y_n*y_n);
  float r = tanf( asinf( (1.0f / k) * sinf( atanf( R ) ) ) );
  float reduction_factor = r/R;
  (*x_nd) = reduction_factor * x_n;
  (*y_nd) = reduction_factor * y_n;
  return true;
}

bool Dhane_undistortion(float x_nd, float y_nd, float* x_n, float* y_n, float k) {
  float r = sqrtf( x_nd*x_nd + y_nd*y_nd );
  float inner_part = sinf( atanf( r ) ) * k;
  // we will take the asine of the inner part. It can happen that it is outside of [-1, 1], in which case, it would lead to an error.
  if(fabs(inner_part) > 0.9999) {
    return false;
  }

  float R = tanf( asinf( inner_part ) );
  float enlargement_factor = R / r;
  (*x_n) = enlargement_factor * x_nd;
  (*y_n) = enlargement_factor * y_nd;

  return true;
}

void normalized_to_pixels(float x_n_, float y_n_, float* x_p, float* y_p, const float* K) {
  (*x_p) = x_n_ * K[0] + K[2];
  (*y_p) = y_n_ * K[4] + K[5];
}

void pixels_to_normalized(float x_p, float y_p, float* x_n_, float* y_n_, const float* K) {
  (*x_n_) = (x_p - K[2]) / K[0];
  (*y_n_) = (y_p - K[5]) / K[4];
}

bool distorted_pixels_to_normalized_coords(float x_pd, float y_pd, float* x_n, float* y_n, float k, const float* K) {
  float x_nd, y_nd;
  pixels_to_normalized(x_pd, y_pd, &x_nd, &y_nd, K);
  bool success = Dhane_undistortion(x_nd, y_nd, x_n, y_n, k);
  return success;
}

bool normalized_coords_to_distorted_pixels(float x_n, float y_n, float *x_pd, float *y_pd, float k, const float* K) {
  float x_nd, y_nd;
  bool success = Dhane_distortion(x_n, y_n, &x_nd, &y_nd, k);
  if(!success) {
    return false;
  }
  else {
    normalized_to_pixels(x_nd, y_nd, x_pd, y_pd, K);
  }
  return success;
}

float K[9] = {CNN_CAMERA.camera_intrinsics.focal_x, 0.0f, CNN_CAMERA.camera_intrinsics.center_x,
              0.0f, CNN_CAMERA.camera_intrinsics.focal_y, CNN_CAMERA.camera_intrinsics.center_y,
              0.0f, 0.0f, 1.0f
             };

int length(cv::Mat &image, cv::Vec4i line) {
    //printf("Image shape: (%d, %d)\n", image.rows, image.cols);
    int MARGIN = 2;
    int count = 0;
    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
    float dydx = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
    //printf("dy: %i, dx: %i -> dy/dx: %f\n", y2 - y1, x2 - x1, dydx);
    // Iterate over the line segment and count white pixels
    for (int x = x1; x <= x2; x++) {
        int y = y1 + dydx * (x - x1);
        
        // Ensure coordinates are within image bounds
        for (int ty = y - MARGIN; ty <= y + MARGIN; ty++) {
            if (x >= 0 && x < image.cols && ty >= 0 && ty < image.rows) {
                //printf("x: %u, y: %u -> pixel: %hhu\n", x, ty, image.at<uchar>(ty, x) == 255);
                // Check if the pixel is white (assuming white is 255 in grayscale)
                //if (image.at<uchar>(ty, x) == 255) {
                if (image.at<uint8_t>(x, y) == 255) {
                    count++;
                    break;
                }
            }
        }
    }
    
    return count;
}

void fill_regions(std::array<float, 3> &regions, std::array<bool, IMAGE_WIDTH> &free_space) {
    for(int i = 0; i < IMAGE_WIDTH; i++) {
        regions[i / (IMAGE_WIDTH / 3)] += static_cast<float>(free_space[i]);
    }
    float size = regions[0] + regions[1] + regions[2]; 
    for(int i = 0; i < 3; i++) {
        regions[i] /= size;
    }
}

std::array<int, 2> findIntersection(cv::Vec4i line1, cv::Vec4i line2) {
    float x1 = line1[1], y1 = line1[0], x2 = line1[3], y2 = line1[2];
    float x3 = line2[1], y3 = line2[0], x4 = line2[3], y4 = line2[2];

    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    float x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
    float y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

    return {static_cast<int>(x), static_cast<int>(y)};
}

void remove_mess(cv::Mat &image, int window_size = 15, double thresh = 0.35) {
    int pixels_threshold = window_size * window_size * thresh;

    for (int y_start = 0; y_start < image.rows - window_size + 1; y_start += window_size) {
        for (int x_start = 0; x_start < image.cols - window_size + 1; x_start += window_size) {
            int white_pixels = 0;

            for (int y = y_start; y < y_start + window_size; y++) {
                for (int x = x_start; x < x_start + window_size; x++) {
                    // Check if the pixel is white (assuming 255 represents white)
                    if (image.at<uchar>(y, x) == 255) {
                        white_pixels += 3;
                    }
                }
            }

            // If there are pixels_threshold or more white pixels, replace the entire ROI with black pixels
            if (white_pixels >= pixels_threshold) {
                for (int y = y_start; y < y_start + window_size; y++) {
                    for (int x = x_start; x < x_start + window_size; x++) {
                        image.at<uchar>(y, x) = 0;
                    }
                }
            }
        }
    }
}


cv::Vec4f performLinearRegressionWithGuess(cv::Mat binaryImage, cv::Vec4i model, int xRange[2]) {
    const int ACCEPT_REGION = 10;

    std::vector<std::vector<int>> points;
    // TODO CHANGE ORIENTATION
    int x0 = model[1];
    int y0 = model[0];
    int x1 = model[3];
    int y1 = model[2];

    float m = static_cast<float>(y1 - y0) / static_cast<float>(x1 - x0);
    float b = y0 - m * x0;

    printf("x: %u - %u\n", xRange[0], xRange[1]);
    for (int x = xRange[0]; x <= xRange[1]; x++) {
        int distance = ACCEPT_REGION;
        int bestY = -1;
        int prediction = static_cast<int>(m * x + b);

        for (int y = std::max(0, prediction - ACCEPT_REGION); y <= std::min(binaryImage.rows - 1, prediction + ACCEPT_REGION); y++) {
            //if(binaryImage.at<uchar>(y, x) == 255) {
            if(binaryImage.at<uint8_t>(x, y) == 255) {
                int newDistance = abs(y - prediction);
                if (newDistance < distance) {
                    bestY = y;
                    distance = newDistance;
                }
            }
        }

        if (bestY != -1) {
            //for (int j = -2; j <= 2; j++) {
                //binaryImage.at<uchar>(bestY + j, x) = 255;
            //}
            //printf("new_point: %u, %u\n", x, bestY);
            points.push_back({x, bestY});
        }
    }

    if (points.size() < 2) {
        printf("No white pixels found in the image.");
        return model;
    }

    // Convert vector of vectors to 2D array for fitting
    cv::Mat pointsMat(points.size(), 2, CV_32S);
    for (size_t i = 0; i < points.size(); i++) {
        pointsMat.at<int>(i, 0) = points[i][0]; // x coordinate
        pointsMat.at<int>(i, 1) = points[i][1]; // y coordinate
        //for (int j = -2; j <= 2; j++) {
        //    binaryImage.at<uchar>(points[i][1] + j, points[i][0]) = 255;
        //}
    }

    // Perform linear regression
    cv::Vec4f newModel; // Updated model
    cv::fitLine(pointsMat, newModel, cv::DIST_L2, 0, 0.01, 0.01);

    // Convert the model from (vx, vy, x0, y0) to (x0, y0, x1, y1)
    float vx = newModel[0];
    float vy = newModel[1];
    float x_ref = newModel[2];
    float y_ref = newModel[3];

    x0 = xRange[0];
    x1 = xRange[1];
    y0 = static_cast<int> (y_ref + (x0 - x_ref) * (vy / vx));
    y1 = static_cast<int> (y_ref + (x1 - x_ref) * (vy / vx));
    
    newModel[1] = x0;
    newModel[0] = y0;
    newModel[3] = x1;
    newModel[2] = y1;

    return newModel;
}


// uint8_t white[4] = {127, 255, 127, 255};

// Function
static struct image_t *opencv_func(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  printf("position drone: (%.3f, %.3f)\n", stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y);
  printf("heading: %.3f", DegOfRad(stateGetNedToBodyEulers_f()->psi));
  // convert to grayscale, but also keep original for live feed
  cv::Mat gray;
  Mat img_mat(img->h, img->w, CV_8UC2, img->buf);

  float x_pd, y_pd;
  //for(int y = 80; y < 120; y++) {
  //  for(int x = 0; x < 80; x++) {
  //      normalized_coords_to_distorted_pixels(static_cast<float>(x), static_cast<float>(y), &x_pd, &y_pd, CNN_CAMERA.camera_intrinsics.Dhane_k, K);
  //      //printf("x: %.1f, y: %.1f\n", x_pd, y_pd);
  //      if (x_pd < img->h && y_pd < img->w) img_mat.at<cv::Vec2b>(static_cast<int>(y_pd), static_cast<int>(x_pd)) = img_mat.at<cv::Vec2b>(y, x);
  //  }
  //}

  cvtColor(img_mat, gray, COLOR_YUV2GRAY_Y422);

  // Apply Gaussian blur
  cv::GaussianBlur(gray, gray, cv::Size(15, 15), 0.9);

  // Detect edges using Canny edge detector
  // Now we could use only gray instead of creating edges
  cv::Mat edges;
  cv::Canny(gray, edges, 15, 20);

  // Ignore upper part of the image
  // Our width is the y range in this configuration
  //cv::Mat lower_image = edges(cv::Rect(0, 0, edges.cols, Y_MAX));
  cv::Mat lower_image = edges(cv::Rect(0, 0, Y_MAX, edges.rows));

  // Remove unwanted elements
  remove_mess(lower_image);

  // Ignore sides since we still have curvature
  //lower_image = edges(cv::Rect(0, 50, Y_MAX, edges.rows - 50));

  // DEBUG
  /*for (int y = 0; y < edges.rows; ++y) {
      for (int x = 0; x < edges.cols; ++x) {
          //printf("(%d, %d): %hhd", x, y, gray.at<uchar>(y, x));
          if (edges.at<uchar>(y, x) == 255) {
              // Set the corresponding pixel in img_mat to white
              img_mat.at<cv::Vec2b>(y, x) = cv::Vec2b(255, 255);
          }
      }
  }*/

  // Detect lines using Hough transform
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(lower_image, lines, 1, 1 * CV_PI / 180, 10, 30, 10);

  cv::Vec4i final_lines[2] = {cv::Vec4i(0, 0, 0, 0), cv::Vec4i(0, 0, 0, 0)};
  int lengths[2] = {0, 0};
  for (int i = 0; i < lines.size(); i++) {
      cv::Vec4i line = lines[i];
      float angle = atan2(line[2] - line[0], line[3] - line[1]) * 180 / CV_PI;
      //printf("angle: %.2f\n", angle);
      if (angle < 50 || angle > 130) {
          int len = length(edges, line);
          // CAREFUL WHEN WE HAVE 3 EDGES THE TRACKER WOULD BE BETTER HERE
          int idx = static_cast<int> (angle > 90);
          if (len > lengths[idx]) {
              lengths[idx] = len;
              final_lines[idx] = line;
          }
      cv::line(img_mat, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(100, 100), 3);
      }
      //else cv::line(img_mat, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(128, 200), 3);
  }
  
  for (int i = 0; i < 2; i++) {
      if (lengths[i] > 0) {
          //printf("i: %lu -> %u \n", i, lengths[i]);
          cv::line(img_mat, cv::Point(final_lines[i][0], final_lines[i][1]), cv::Point(final_lines[i][2], final_lines[i][3]), cv::Scalar(255, 255), 1);
      }
  }

  std::array<int, 2> center = {0, 0};
  std::array<bool, IMAGE_WIDTH> free_space = {};

  // skip if both are 0
  if (lengths[0] || lengths[1]) {
      if (lengths[0] == 0) {
          center[0] = final_lines[1][2];
          center[1] = final_lines[1][3];
      }
      else if (lengths[1] == 0) {
          center[0] = final_lines[0][2];
          center[1] = final_lines[0][3];
      }
      else {
          center = findIntersection(final_lines[0], final_lines[1]);
          // In case of error just take the middle point at random
          if (center[0] >= edges.rows or center[1] >= edges.cols) {
              printf("invalid: x: %d; y: %d\n", center[0], center[1]);
              center[0] = static_cast<float> (final_lines[0][3] + final_lines[1][1]) / 2;
              center[1] = static_cast<float> (final_lines[0][2] + final_lines[1][0]) / 2;
          }
      }

      cv::circle(img_mat, cv::Point(center[1], center[0]), 5, cv::Scalar(0, 255), -1);

      for(int i = 0; i <= 1; i++) {
          if(lengths[i] > 0) {
              int range_x[2] = {center[0] * i, center[0] * static_cast<int> (! static_cast<bool>(i)) + edges.rows * i};
              final_lines[i] = performLinearRegressionWithGuess(edges, final_lines[i], range_x);
              cv::line(img_mat, cv::Point(final_lines[i][0], final_lines[i][1]), cv::Point(final_lines[i][2], final_lines[i][3]), cv::Scalar(255, 0), 1);
              
              int x1 = final_lines[i][1], y1 = final_lines[i][0], x2 = final_lines[i][3], y2 = final_lines[i][2];
              cv::circle(img_mat, cv::Point(y1, x1), 5, cv::Scalar(255, 255), -1);
              cv::circle(img_mat, cv::Point(y2, x2), 5, cv::Scalar(255, 255), -1);
              float dydx = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
              for (int x = x1; x <= x2; x += 5) {
                  int y = y1 + dydx * (x - x1);
                  bool valid = false;
                  for (int j = -2; j <= 2; j++) {
                      int newY = y + j;
                      //img_mat.at<cv::Vec3b>(newY, x) == cv::Vec3b(255, 255, 255);
                      //img_mat.at<cv::Vec2b>(x, newY) == cv::Vec2b(255, 255);
                      if (newY >= 0 && newY < edges.rows && edges.at<uchar>(x, newY) == 255) {
                          valid = true;
                          free_space[x] = true;
                          break;
                      }
                  }
                  if(!valid && y >= 0 && y < edges.cols) {
                      //printf("%u, %u \n", x, y);
                      cv::Point center(y, x);
                      cv::circle(img_mat, center, 5, cv::Scalar(0, 255), -1);
                  }
              }
          }
      }
    }

  return img;

  std::array<float, 3> regions = {0, 0, 0};
  fill_regions(regions, free_space);
  AbiSendMsgCNN_CONTROL_INPUTS(CNN_CONTROL_INPUTS_ID, regions[2], regions[1], regions[0]);


  return img;
}

void opencv_init(void)
{
  cv_add_to_device(&CNN_CAMERA, opencv_func, CNN_FPS, 0);
}

