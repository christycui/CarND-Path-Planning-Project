# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

## Basic Build Instructions

1. Clone this repo.
2. Enter the build directory: `cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Path Planning Report
In this project, I used the car's sensor fusion data for behavior planning. Specifically, I defined five motion states for the car: keep lane, lane change left, lane change right, prepare lane change left, and prepare lane change right. The default behavior is to follow the current lane (keep lane), but when the car is getting too close to a vehicle in front of it, it will choose to enter other states. A cost function is used to determine which state to enter. It takes as inputs the car's target speed, intended_lane, final_lane, and all lane speeds. It is designed such that there is a high cost for states with slow intended lane and final lane. As a result, the car will switch to a faster lane when it finds another vehicle in front of it.

Using the sensor fusion data, the algorithm also detects whether it is safe for the car to change lane right now. If a car is passing by the car in high speed, although the left lane is faster, our car will not change to the left lane immediately. Instead, it will prepare to change to left lane by matching the speed of the left lane and wait to change lane when it's safe to do so.

For generating the trajectories, I used the spline library. I took five points to construct the spline, two from the car's current position and three from highway waypoints 50, 100, and 120 meters away. I experimented with those numbers to minimize jerk and acceleration. Next, I poppped the previous path points back into the vector and created new path points alongside the spline to fill up the vector of size 50. In generating those new path points, I slowly increased or decreased the reference speed so the car experiences minimal jerk and acceleration.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
   