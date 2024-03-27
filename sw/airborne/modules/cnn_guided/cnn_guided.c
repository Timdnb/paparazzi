/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/cnn_guided/cnn_guided.c"
 * @author Tim den Blanken
 * CNN guided module receives control inputs from the CNN and uses these to navigate the drone
 */

#include "modules/cnn_guided/cnn_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>


uint8_t random_direction(void);
bool out_of_bounds(float* speed_conf);

// define settings
float oag_max_speed = 0.7f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(30.f);  // heading change setpoint for avoidance [rad/s]
float base_speed_fraction = 0.3f;         // base_speed = oag_max_speed * base_speed_fraction

// define and initialise global variables
enum navigation_state_t navigation_state = STATIONARY;  // current state in state machine
float avoidance_heading_direction = 0;                  // heading change direction for avoidance [rad/s]

// initialize confidence values at 0
float forward_conf = 0.f;
float right_conf = 0.f;
float left_conf = 0.f;
float speed_conf = 1.f;

// create memory variables
int times_left = 0;
int times_right = 0;
int times_forward = 0;
int times_out_of_bounds = 0;
int times_reentering = 0;

// radius variable
float radius = 0.f;

// periodic function frequency
int frequency = 8;

// callback function to save control inputs
#ifndef CNN_CONTROL_INPUTS_ID
#error "CNN_CONTROL_INPUTS_ID is not defined"
#endif

static abi_event cnn_control; // create message variable __attribute__((unused))
static void save_control_inputs(uint8_t __attribute__((unused)) sender_id, float left, float forward, float right)
{
  left_conf = left;
  forward_conf = forward;
  right_conf = right;
}

// Control navigation state
void update_navigation_state(float forward_conf, float right_conf, float left_conf, enum navigation_state_t* navigation_state, int* times_forward, int* times_left, int* times_right, float* radius) {
  if (out_of_bounds(&speed_conf)) {
    *navigation_state = OUT_OF_BOUNDS;
    printf("Out of bounds\n");
  } else if (forward_conf > right_conf && forward_conf > left_conf) {
    *navigation_state = FORWARD;
    printf("Forward\n");
  } else if (right_conf > forward_conf && right_conf > left_conf) {
    *navigation_state = RIGHT;
    printf("Right\n");
  } else if (left_conf > forward_conf && left_conf > right_conf) {
    *navigation_state = LEFT;
    printf("Left\n");
  } else {
    *navigation_state = STATIONARY;
    printf("Stationary\n");
  }
}

// Check if the drone is out of bounds, update speed_conf based on distance to center
bool out_of_bounds(float *speed_conf) {
  float x_pos = stateGetPositionEnu_f()->x;
  float y_pos = stateGetPositionEnu_f()->y;
  float r = x_pos * x_pos + y_pos * y_pos;  // not taking the square root to save computation time

  if (r < 6) {
    *speed_conf = 1;
  } else if (r < 7) {
    *speed_conf = 0.8;
  } else if (r < 8) {
    *speed_conf = 0.6;
  } else if (r < 9) {
    *speed_conf = 0.4;
  } else
  if (r > 9) {
    return true;
  }
  return false;
}

/*
 * Initialisation function
 */
void cnn_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  random_direction();

  // Bind our CNN callback to save the control inputs
  AbiBindMsgCNN_CONTROL_INPUTS(CNN_CONTROL_INPUTS_ID, &cnn_control, save_control_inputs);
}

/*
 * Function that updates motion based on CNN confidence values
 */
void cnn_guided_periodic(void)
{
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = STATIONARY;
    return;
  }

  switch (navigation_state){
    case STATIONARY:
      guidance_h_set_body_vel(0, 0);
      guidance_h_set_heading_rate(RadOfDeg(0));
      update_navigation_state(forward_conf, right_conf, left_conf, &navigation_state, &times_forward, &times_left, &times_right, &radius);

    case FORWARD:
      float speed = speed_conf*fmin(oag_max_speed, (base_speed_fraction + forward_conf) * oag_max_speed);
      guidance_h_set_body_vel(speed, 0);
      update_navigation_state(forward_conf, right_conf, left_conf, &navigation_state, &times_forward, &times_left, &times_right, &radius);

      break;
    case LEFT:
      guidance_h_set_body_vel(base_speed_fraction*oag_max_speed, 0);
      guidance_h_set_heading_rate(-oag_heading_rate);
      times_left++;
      update_navigation_state(forward_conf, right_conf, left_conf, &navigation_state, &times_forward, &times_left, &times_right, &radius);

      break;
    case RIGHT: 
      guidance_h_set_body_vel(base_speed_fraction*oag_max_speed, 0);
      guidance_h_set_heading_rate(oag_heading_rate);
      times_right++;
      update_navigation_state(forward_conf, right_conf, left_conf, &navigation_state, &times_forward, &times_left, &times_right, &radius);

      break;
    case OUT_OF_BOUNDS:
      guidance_h_set_body_vel(0, 0);
      guidance_h_set_heading_rate(avoidance_heading_direction * 2 * oag_heading_rate);
      times_out_of_bounds++;

      if (times_out_of_bounds > (M_PI / (2*oag_heading_rate))*frequency) {
        times_out_of_bounds = 0;
        navigation_state = REENTER_ARENA;
      }

      break;
    case REENTER_ARENA:
      guidance_h_set_heading_rate(RadOfDeg(0));
      guidance_h_set_body_vel(2*base_speed_fraction*oag_max_speed, 0);
      times_reentering++;

      if (!out_of_bounds(&speed_conf) && times_reentering > 2*frequency) {
        times_reentering = 0;
        navigation_state = FORWARD;
        random_direction();
      }

      break;
    default:
      break;
  }
  return;
}


/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t random_direction(void)
{
  // Randomly choose CW or CCW avoiding direction
  avoidance_heading_direction = (rand() % 2 == 0) ? 1.f : -1.f;
  return false;
}