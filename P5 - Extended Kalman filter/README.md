# Extended Kalman Filter
## Overview
![image](https://github.com/adamalavi/Self_driving_cars_Udacity-ND/blob/master/P5%20-%20Extended%20Kalman%20filter/Recordings/EKF-closeup.gif)

This code implements the extended Kalman filter in C++ where simulated lidar and radar measurements were provided. The red dots are used to indicate lidar measurements and the blue dots indicate radar measurements. A Kalman filter, lidar measurements and radar measurements were used to perform **sensor fusion** abd track the vehicle's position which is denoted by green dots on the track.
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The uWebSocketIO can be dowloaded [here](https://github.com/uWebSockets/uWebSockets).
## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions
1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
3. Run it: `./ExtendedKF `
