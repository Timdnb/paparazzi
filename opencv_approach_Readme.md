### Code overview
Below all files are listed that have either been added or modified in order to run the OpenCV approach on the drone.

#### Added files
- `paparazzi/conf/airframes/tudelft/bebop_cnn_guided.xml`
    - defines the airframe and connects cnn and cnn_guided modules
- `paparazzi/conf/flight_plans/tudelft/bebop_non_cnn_guided.xml`
    - defines the flight_plan, does NOT include the circular zone that is defined on newer versions of the CNN approach
- `paparazzi/conf/modules/opencv_approach.xml`
    - defines opencv_approach module
- `paparazzi/conf/modules/cnn_guided.xml`
    - defines the cnn_guided module, includes definition of global parameters. This module is outdated compared to the CNN, as the group changed focus when the decision to use the CNN was taken. 
- `paparazzi/sw/airborne/modules/computer_vision/opencv_approach.cpp`
    -  module that takes images as input and sends control commands as outputs.
- `paparazzi/sw/airborne/modules/computer_vision/opencv_approach.h`
    - header file for opencv_approach module. Includes code for compatibility with c++
- `paparazzi/sw/airborne/modules/cnn_guided/cnn_guided.c`
    - control module that takes outputs from the copencv_approach module and uses those to actually control the drone. Outdated compared to CNN approach. 
- `paparazzi/sw/airborne/modules/cnn_guided/cnn_guided.h`
    - header file for cnn_guided module

#### Modified files
- `paparazzi/conf/userconf/tudelft/course_conf.xml`
    - added a new aircraft