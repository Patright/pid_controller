# Project for the CarND-Controls-PID course in Udacity's Self-Driving Car Nanodegree.

---

## Dependencies

* cmake >= 3.5
* make >= 4.1(mac, linux), 3.81(Windows)
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases)


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Workflow for the PID-Controller

In this project we'll revisit the lake race track from the Behavioral Cloning Project. This time, however, I implemented a PID controller in C++ to maneuver the vehicle around the track! 

In `PID.cpp` I put the methods for Initializing the PID coefficients, update the PID errors based on the Cross Track Error (CTE) and the Calculation and return of the total error.