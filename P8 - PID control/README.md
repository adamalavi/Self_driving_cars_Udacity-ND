# PID control project
![image](https://github.com/adamalavi/Self_driving_cars_Udacity-ND/blob/master/P8%20-%20PID%20control/Videos/PID-control.gif)

## Working
##### Effect of each PID component
 * P stands for proportional where the error at every point is directly multiplied by a constant called Kp and this product is then added to the error with the other two components. It plays the most important role in rectifying the error and getting it down to zero. But even after the best tuning value for Kp is achieved, it causes oscillations because it leds to overshooting.
 * I stands for integral and it is used to rectify very small errors and also accounts for systematic bias if any. It is the product of a constant Ki and the sum of all errors upto the point of calculation. The value of Ki is always very small because it is multiplied with a very large number and a high value may throw the vehicle off the track.
 * D stands for derivative and it is used to prevent overshooting due to the P component, The D component is the product of a cinstant Kd and the difference between the current error and the error in the previous cycle. This avoids overshooting by slowly decreasing the error value as the car comes closer to its target value.
 
##### Tuning of Kp, Ki, Kd
First, the Ki and Kd values were set to zero and the best possible behaviour was obtained by only tuning Kp. Then Kd was gradually increased from 0 to minimize the oscillations. Once a favourable outcome was obtained, Ki was set to correct minute errors and prevent jerks.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
