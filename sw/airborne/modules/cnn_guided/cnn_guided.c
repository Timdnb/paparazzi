/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/cnn_guided/cnn_guided.c"
 * @author Tim den Blanken
 * CNN guided module receives control inputs from the CNN and uses these to navigate the drone. The code assumes
 * that the CNN return confidence values for 'left', 'forward' and 'right'. Based on these values the drone is
 * set to a certain navigation state. Currently this is simply done by checking which confidence value is the
 * highest and setting the drone to the corresponding state. This does cause some jittery behaviour, so a more
 * refined approach should be considered. For example by keeping memory variables of previous inputs and using 
 * those to determine the next state. An interesting approach could be to add a RNN on the outputs of the CNN.
 * 
 * This module uses the guidance_h module to control the drone. Based on the state the velocity and heading rate
 * are set. The drone can be in one of the following states:
 * - STATIONARY: drone is stationary
 * - FORWARD: drone is moving forward
 * - LEFT: drone is moving forward and turning left
 * - RIGHT: drone is moving forward and turning right
 * - OUT_OF_BOUNDS: drone is out of bounds
 * - REENTER_ARENA: drone is reentering the arena
 * 
 * The check whether the drone is still in bounds is done by calculating the distance to the center of the arena.
 * If this exceeds the threshold the drone is set to the OUT_OF_BOUNDS state. The drone will then stop and turn
 * 180 degrees and go back into the arena. This approach works rather well, as the circular obstacle zone together
 * with the 180 degree turns make sure that the drone is pushed back into a good trajectory.
 */

#include "modules/cnn_guided/cnn_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>

// Function declarations
uint8_t random_direction(void);
bool out_of_bounds(float* speed_conf);

// Define standard values, can be changed in flight in GCS
float oag_max_speed = 0.7f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(30.f);  // heading change setpoint for avoidance [rad/s]
float base_speed_fraction = 0.3f;         // base_speed = oag_max_speed * base_speed_fraction

// Define and initialise global variables
enum navigation_state_t navigation_state = STATIONARY;  // current state in state machine
float avoidance_heading_direction = 0;                  // heading change direction for avoidance [rad/s]

// Initialize confidence values at 0
float forward_conf = 0.f;
float right_conf = 0.f;
float left_conf = 0.f;

// Initialize speed confidence at 1 (multiplication factor for speed)
float speed_conf = 1.f;

// Initialize speed at 0
float speed = 0.f;

// Create memory variables
int times_out_of_bounds = 0;
int times_reentering = 0;

// Radius variable, used to check if the drone is out of bounds
float radius = 0.f;

// Periodic function frequency
int frequency = 8; // should have the same frequency as cnn_guided_periodic (defined in cnn_guided.xml)

// callback function to save control inputs
#ifndef CNN_CONTROL_INPUTS_ID
#error "CNN_CONTROL_INPUTS_ID is not defined"
#endif

static abi_event cnn_control;
static void save_control_inputs(uint8_t __attribute__((unused)) sender_id, float left, float forward, float right)
{
  left_conf = left;
  forward_conf = forward;
  right_conf = right;
}

// Control navigation state
void update_navigation_state(float forward_conf, float right_conf, float left_conf, enum navigation_state_t* navigation_state) {
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

// Check if the drone is out of bounds, also update speed_conf based on distance to center
bool out_of_bounds(float *speed_conf) {
  float x_pos = stateGetPositionEnu_f()->x;
  float y_pos = stateGetPositionEnu_f()->y;
  float r = x_pos * x_pos + y_pos * y_pos;  // not taking the square root to save some computation time

  // Assign speed_conf based on distance to center, prevents overshoot into red zone
  if (r < 8) {
    *speed_conf = 1;
  } else if (r < 9) {
    *speed_conf = 0.8;
  } else if (r < 10) {
    *speed_conf = 0.6;
  } else if (r < 11) {
    *speed_conf = 0.4;
  } else
  if (r > 12) {
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
    // STATIONARY logic: stop and set heading rate to 0, then update navigation state
      guidance_h_set_body_vel(0, 0);
      guidance_h_set_heading_rate(RadOfDeg(0));
      update_navigation_state(forward_conf, right_conf, left_conf, &navigation_state);

      break;
    case FORWARD:
    // FORWARD logic: set forward speed to base speed + the max speed times the confidence value, then update navigation state
      speed = speed_conf*fmin(oag_max_speed, (base_speed_fraction + forward_conf) * oag_max_speed);
      guidance_h_set_body_vel(speed, 0);
      update_navigation_state(forward_conf, right_conf, left_conf, &navigation_state);

      break;
    case LEFT:
    // LEFT logic: set forward speed to base speed and turn left, then update navigation state
      guidance_h_set_body_vel(base_speed_fraction*oag_max_speed, 0);
      guidance_h_set_heading_rate(-oag_heading_rate);
      update_navigation_state(forward_conf, right_conf, left_conf, &navigation_state);

      break;
    case RIGHT: 
    // RIGHT logic: set forward speed to base speed and turn right, then update navigation state
      guidance_h_set_body_vel(base_speed_fraction*oag_max_speed, 0);
      guidance_h_set_heading_rate(oag_heading_rate);
      update_navigation_state(forward_conf, right_conf, left_conf, &navigation_state);

      break;
    case OUT_OF_BOUNDS:
    // OUT_OF_BOUNDS logic: stop, turn 180 degrees, go into REENTER_ARENA
      guidance_h_set_body_vel(0, 0);
      guidance_h_set_heading_rate(avoidance_heading_direction * 2 * oag_heading_rate);
      times_out_of_bounds++;

      if (times_out_of_bounds > (M_PI / (2*oag_heading_rate))*frequency) {
        times_out_of_bounds = 0;
        navigation_state = REENTER_ARENA;
      }

      break;
    case REENTER_ARENA:
    // REENTER_ARERA logic: stop turning, go forward and check if we are back in the arena, if yes, go back to FORWARD
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
 * Sets the variable avoidance_heading_direction randomly positive/negative
 */
uint8_t random_direction(void)
{
  avoidance_heading_direction = (rand() % 2 == 0) ? 1.f : -1.f;
  return false;
}