/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/cnn_guided/cnn_guided.c"
 * @author Tim den Blanken
 * TO COME
 */

#include "modules/cnn_guided/cnn_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define settings
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
float avoidance_heading_direction = 0;    // heading change direction for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine

// create message variable
static abi_event cnn_control;

// create control input variables
float forward = 0;
float right = 0;
float left = 0;

// callback function to save control inputs
static void save_control_inputs(uint8_t __attribute__((unused)) sender_id, float right, float forward, float left)
{
  right = right;
  forward = forward;
  left = left;
}

/*
 * Initialisation function
 */
void cnn_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgCNN_CONTROL_INPUTS(CNN_CONTROL_INPUTS_ID, &cnn_control, save_control_inputs);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void cnn_guided_periodic(void)
{
  guidance_h_set_body_vel(0.01, 0);
  // // Only run the mudule if we are in the correct flight mode
  // if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
  //   navigation_state = SEARCH_FOR_SAFE_HEADING;
  //   obstacle_free_confidence = 0;
  //   return;
  // }

  // // compute current color thresholds
  // int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  // int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  // float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  // VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  // VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  // VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // // update our safe confidence using color threshold
  // if(color_count < color_count_threshold){
  //   obstacle_free_confidence++;
  // } else {
  //   obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  // }

  // // bound obstacle_free_confidence
  // Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  // float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);

  // switch (navigation_state){
  //   case SAFE:
  //     if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
  //       navigation_state = OUT_OF_BOUNDS;
  //     } else if (obstacle_free_confidence == 0){
  //       navigation_state = OBSTACLE_FOUND;
  //     } else {
  //       guidance_h_set_body_vel(speed_sp, 0);
  //     }

  //     break;
  //   case OBSTACLE_FOUND:
  //     // stop
  //     guidance_h_set_body_vel(0, 0);

  //     // randomly select new search direction
  //     chooseRandomIncrementAvoidance();

  //     navigation_state = SEARCH_FOR_SAFE_HEADING;

  //     break;
  //   case SEARCH_FOR_SAFE_HEADING:
  //     guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

  //     // make sure we have a couple of good readings before declaring the way safe
  //     if (obstacle_free_confidence >= 2){
  //       guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
  //       navigation_state = SAFE;
  //     }
  //     break;
  //   case OUT_OF_BOUNDS:
  //     // stop
  //     guidance_h_set_body_vel(0, 0);

  //     // start turn back into arena
  //     guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));

  //     navigation_state = REENTER_ARENA;

  //     break;
  //   case REENTER_ARENA:
  //     // force floor center to opposite side of turn to head back into arena
  //     if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
  //       // return to heading mode
  //       guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

  //       // reset safe counter
  //       obstacle_free_confidence = 0;

  //       // ensure direction is safe before continuing
  //       navigation_state = SAFE;
  //     }
  //     break;
  //   default:
  //     break;
  // }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
  } else {
    avoidance_heading_direction = -1.f;
  }
  return false;
}

/* Navigation states
FORWARD ->        fly forward with speed based on confidence and direction based on right / left confidence (within bounds)
LEFT ->           set forwared sped to 0 and start turning left, if forward is more confident than left, state = FORWARD,
                  if left did more than 360 deg, go state UP or DOWN (randomly)
UP ->             go up x distance, then go state LEFT or RIGHT (randomly)
DOWN ->           go down x distance, then go state LEFT or RIGHT (randomly)
RIGHT ->          set forward speed to 0 and start turning right, if forward is more confident than right, state = FORWARD,
                  if right did more than 360 deg, go state UP or DOWN (randomly)
OUT_OF_BOUNDS ->  stop, turn back into arena, state = REENTER_ARENA
REENTER_ARENA ->  rotate until a safe forward direction into arena is found, state = FORWARD
*/