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
/**
 * Follow path and RFL mode
 * waypoints need to be implemented for follow_path
 */
#include "modules/cnn_guided/cnn_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>

#define CNN_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[CNN_VERBOSE->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if CNN_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  FOLLOW_PATH,
  RFL_MODE,
  OUT_OF_BOUNDS,
  REENTER_ARENA, 
  STATIONARY
};

// define settings
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state = STATIONARY;   // current state in state machine
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// create control input variables
uint8_t send = 0 ;
float forward_conf = 0;
float right_conf = 0;
float left_conf = 0;

// create memory variables
int times_left;
int times_right;
int times_forward;
int turns_made;

// callback function to save control inputs
// #ifndef CNN_CONTROL_INPUTS_ID
// #define CNN_CONTROL_INPUTS_ID ABI_BROADCAST
// #endif
static abi_event cnn_control; // create message variable __attribute__((unused))
static void save_control_inputs(uint8_t sender_id, float right, float forward, float left)
{
  send = sender_id;
  right_conf = right;
  forward_conf = forward;
  left_conf = left;
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
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = STATIONARY;
    obstacle_free_confidence = 0;
    return;
  }

   // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);


  // compute current forward threshold
  int32_t forward_threshold = 0.35;

  VERBOSE_PRINT("Left: %f  Forward: %f Right: %f \n", left_conf, forward_conf, right_conf);
  // VERBOSE_PRINT("Obstacle free confidence: %d \n", obstacle_free_confidence);
  // printf("ID: %u \n", send);
  

    // update our safe confidence using color threshold
  if(forward_conf > right_conf && forward_conf > left_conf){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 1;  // be more cautious with positive obstacle detections
  }

  if(obstacle_free_confidence==0){
    navigation_state =RFL_MODE;
  } else {
    navigation_state = FOLLOW_PATH;
  }
  


  switch (navigation_state){
    case FOLLOW_PATH:
      //FOLLOWING PATH 
      speed = forward_conf * oag_max_speed;
      guidance_h_set_body_vel(speed, 0);
      times_forward++;
      break;
    case RFL_MODE:
      if (right_conf > forward_conf && right_conf > left_conf && times_forward > 2){
      speed = 0.1 * forward_conf * oag_max_speed;
      guidance_h_set_body_vel(speed, 0);
      guidance_h_set_heading_rate(RadOfDeg(15));
      times_forward=0;
    } else if (left_conf > forward_conf && left_conf > right_conf && times_forward > 2){
        speed = 0.1 * forward_conf * oag_max_speed;
        guidance_h_set_body_vel(speed, 0);
        guidance_h_set_heading_rate(-RadOfDeg(15));
        times_forward=0;
    } else {
      navigation_state =  FOLLOW_PATH;
    }
    // case RIGHT:
    //   speed = 0.1 * forward_conf * oag_max_speed;
    //   guidance_h_set_body_vel(speed, 0);
    //   guidance_h_set_heading_rate(RadOfDeg(15));
    //   times_right++;
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));

      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      // if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = FOLLOW_PATH;
      // }
      break;
    default:
      break;
  }
  return;
}


/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  // if (right_conf>left_conf) {
  //   
  // } else {
  //   avoidance_heading_direction = -1.f;
  // }
  avoidance_heading_direction = 1.f;
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