/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/cnn_guided/cnn_guided.h"
 * @author Tim den Blanken
 * Header file for cnn_guided module
 */

#ifndef CNN_GUIDED_H
#define CNN_GUIDED_H

// Add the enum declaration here
enum navigation_state_t {
  LEFT,
  FORWARD,
  RIGHT,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  STATIONARY
};

// settings
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]
extern float base_speed_fraction;   // base_speed = oag_max_speed * base_speed_fraction

// External declarations to access the confidence values for logger file
extern float forward_conf;
extern float right_conf;
extern float left_conf; 

extern void cnn_guided_init(void);
extern void cnn_guided_periodic(void);

void update_navigation_state(float forward_conf, float right_conf, float left_conf, enum navigation_state_t* navigation_state);

#endif /* CNN_GUIDED_H */

