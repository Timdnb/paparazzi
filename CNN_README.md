### Code overview
Below all files are listed that have either been added or modified in order to run the convolutional neural networks on the drone.
#### Added files
- `paparazzi/conf/airframes/tudelft/bebop_cnn_guided.xml`
    - defines the airframe and connects cnn and cnn_guided modules
- `paparazzi/conf/flight_plans/tudelft/bebop_cnn_guided.xml`
    - defines the flight_plan, includes the circular obstacle zone
- `paparazzi/conf/modules/cnn.xml`
    - defines cnn module
- `paparazzi/conf/modules/cnn_guided.xml`
    - defines the cnn_guided module, includes definition of global parameters
- `paparazzi/sw/airborne/modules/computer_vision/cnn.c`
    -  module that takes images as input and sends control commands as outputs, includes the entire cnn
- `paparazzi/sw/airborne/modules/computer_vision/cnn.h`
    - header file for cnn module
- `paparazzi/sw/airborne/modules/cnn_guided/cnn_guided.c`
    - control module that takes outputs from the cnn module and uses those to actually control the drone
- `paparazzi/sw/airborne/modules/cnn_guided/cnn_guided.h`
    - header file for cnn module

#### Modified files
- `paparazzi/conf/userconf/tudelft/course_conf.xml`
    - added a new aircraft

#### CNN training code
All CNN training efforts have been done in Python, before translating the final model to C. All code including explanations for reproduction can be found in a [separate repository](https://github.com/Timdnb/CNN-for-Micro-Air-Vehicles).