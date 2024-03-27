/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Felipe Bononi Bello
 */


#include "modules/cnn_guided/cnn_guided.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define CNN_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[CNN_VERBOSE->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if CNN_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

void select_navigation_direction(const float forward_conf, const float right_conf, const float left_conf);
bool shouldProceedForward(const float forward_conf, const float right_conf, const float left_conf);
bool shouldTurnRight(const float forward_conf, const float right_conf, const float left_conf);
bool shouldTurnLeft(const float forward_conf, const float right_conf, const float left_conf);
void reset_turn_counters(void);
void reset_forward_counter(void);


static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);
// void nav_set_heading_towards(float x, float y);

// define settings
float oa_color_count_frac = 0.18f;
int head = 45;
// define and initialise global variables
enum navigation_state_t navigation_state = PATH_FOLLOWING;
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 1.25;               // max waypoint displacement [m]

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

// create control input variables
uint8_t sendqw = 0 ;
float forward_conf = 0;
float right_conf = 0;
float left_conf = 0;

float turning_threshold = 0.45;
float turning_confidence = 0.2;

// create memory variables
int times_left;
int times_right;
int times_forward;
int times_oob = 0; //outofbounds
// create counters
int count_left;
int count_right;
int count_forward;

float new_heading;

float current_heading;
float x_dr;
float y_dr;

// define settings
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
// callback function to save control inputs
// #ifndef CNN_CONTROL_INPUTS_ID
// #define CNN_CONTROL_INPUTS_ID ABI_BROADCAST
// #endif
static abi_event cnn_control; // create message variable __attribute__((unused))
static void save_control_inputs(uint8_t sender_id, float left, float forward, float right)
{
  sendqw = sender_id;
  left_conf = left;
  forward_conf = forward;
  right_conf = right;
}

// Control navigation state
// Improved Version of update_navigation_state
void update_navigation_state(const float forward_conf, const float right_conf, const float left_conf) {
    VERBOSE_PRINT("Before Update: State=%d, Forward=%d/%d, Left=%d/%d, Right=%d/%d\n", navigation_state, times_forward,count_forward, times_left,count_left, times_right, count_right);

     if (!InsideObstacleZone(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y)) {
        navigation_state = OUT_OF_BOUNDS;
    } else {
      select_navigation_direction(forward_conf, right_conf, left_conf);
    }

    VERBOSE_PRINT("After Update: State=%d, Forward=%d/%d, Left=%d/%d, Right=%d/%d\n", navigation_state, times_forward,count_forward, times_left,count_left, times_right, count_right);
}

// New helper function for direction selection
void select_navigation_direction(const float forward_conf, const float right_conf, const float left_conf) {
    // Logic to select navigation direction based on confidence levels
    if (shouldProceedForward(forward_conf, right_conf, left_conf)) {
        navigation_state = FORWARD;
        reset_turn_counters();
    } else if (shouldTurnRight(forward_conf,right_conf, left_conf)) {
        navigation_state = RIGHT;
        reset_forward_counter();
    } else if (shouldTurnLeft(forward_conf,right_conf, left_conf)) {
        navigation_state = LEFT;
        reset_forward_counter();
    } else {
        navigation_state = PATH_FOLLOWING;
    }
}

// Further break down into smaller decision-making functions
bool shouldProceedForward(const float forward_conf, const float right_conf, const float left_conf) {
    return forward_conf > right_conf && forward_conf > left_conf && forward_conf>turning_threshold;
}

bool shouldTurnRight(const float forward_conf, const float right_conf, const float left_conf) {
    return right_conf > forward_conf && right_conf > left_conf && navigation_state != LEFT && right_conf > turning_threshold ;//&& times_forward > 3;
}

bool shouldTurnLeft(const float forward_conf, const float right_conf, const float left_conf) {
    return left_conf > forward_conf && left_conf > right_conf && navigation_state != RIGHT && left_conf > turning_threshold;// && times_forward > 3;
}

// Utility functions to reset counters
void reset_turn_counters() {
    times_left = 0;
    times_right = 0;
}

void reset_forward_counter() {
    times_forward = 0;
}


/*
 * Initialisation function,
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
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void cnn_guided_periodic(void)
{

  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = PATH_FOLLOWING;
    obstacle_free_confidence = 0;
    return;
  }

  // float time = get_sys_time_float();
  // fprintf(stderr, "New control input at %f\n", time);
 
  // x_dr = stateGetPositionEnu_f()->x;
  // y_dr = stateGetPositionEnu_f()->y;
  // // normalize heading to [-pi, pi]
 

  // VERBOSE_PRINT("heading to %f, rad:%f\n", DegOfRad(current_heading), current_heading);
  // VERBOSE_PRINT("<><>----Heing=%f, pos_x=%f, pos_y=%f\n",DegOfRad(nav.heading), x_dr, y_dr);
  
  // update our safe confidence
  // if (forward_conf > right_conf && forward_conf > left_conf){
  //   obstacle_free_confidence++;
  // } else {
  //   obstacle_free_confidence -= 1;  // be more cautious with positive obstacle detections
  // }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);
  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);
  
  
  switch (navigation_state){
    case PATH_FOLLOWING:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 0.5f);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
        speed = forward_conf * oag_max_speed;
        guidance_h_set_body_vel(speed, 0);
        guidance_h_set_heading_rate(oag_heading_rate*(right_conf-left_conf));
        // nav.heading += 20*(right_conf-left_conf);
        update_navigation_state(forward_conf, right_conf, left_conf);
      }
      
      
      break;

    case STATIONARY:
      guidance_h_set_body_vel(0, 0);    
      
      update_navigation_state(forward_conf, right_conf, left_conf);


    case FORWARD:
      //FORWARD ONLY
      moveWaypointForward(WP_TRAJECTORY, 0.5f);


      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
      speed = forward_conf * oag_max_speed;
      guidance_h_set_body_vel(speed, 0);
      times_forward++;
      count_forward++;
      update_navigation_state(forward_conf, right_conf, left_conf);
      }
      break;
    case LEFT:
      // LEFT SLIGHT FORWARD
      moveWaypointForward(WP_TRAJECTORY, 0.5f );


      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
      speed = turning_confidence* forward_conf * oag_max_speed;
      guidance_h_set_body_vel(speed, 0);
      guidance_h_set_heading_rate(-oag_heading_rate);
      // nav.heading-=45;
      times_left++;
      count_left++;
      update_navigation_state(forward_conf, right_conf, left_conf);
      } 
      break;
    case RIGHT:
      moveWaypointForward(WP_TRAJECTORY, 0.5f);


      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else {
      speed = turning_confidence * forward_conf * oag_max_speed;
      guidance_h_set_body_vel(speed, 0);
      guidance_h_set_heading_rate(oag_heading_rate);
      // nav.heading+=45;
      times_right++;
      count_right++;
      update_navigation_state(forward_conf, right_conf, left_conf);
      }
      break;
    
    case OUT_OF_BOUNDS:
      guidance_h_set_body_vel(0, 0);
      increase_nav_heading(oag_heading_rate);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(oag_heading_rate);
        VERBOSE_PRINT("IM IN!!!. heading to %f, rad:%f\n", DegOfRad(current_heading), current_heading);
  
        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = PATH_FOLLOWING;
      } 
      break;
    default:
      break;
  }
  return;
}


/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
//  */

uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + incrementDegrees;

  // normalize heading to [-pi, pi]
  // FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // nav.heading = new_heading;
  // VERBOSE_PRINT("new_heading*oag_heading_rate):%f",new_heading*oag_heading_rate);
  guidance_h_set_heading_rate(oag_heading_rate);
  // VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

// /*
//  * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
//  */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);

  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
//  */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  // VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
  //              POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
  //              stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

// /*
//  * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
//  */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  // VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                // POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  //if waypoint inside r of middle go forward x amount of meters.
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    // VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

