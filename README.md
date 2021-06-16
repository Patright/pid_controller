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

The PID-Controller consists of three elements:   
1. The Proportional element, in the case of steering it does steer in proportion to the Cross Track Error, the lateral distance to the desired driving trajectory.
2. The Differential elements task is to countersteer the proportional steering, to avoid overshooting, which would otherwise result in meandering around the intended driving trajectory
3. The Integral coefficient, comes in play in case of a systematic bias. A systematic bias might be a steering shift that comes from a not 100% accurate alignment of the wheels. This way the systematic bias is compensated.   

I used this formula to adjust the Steering angle to the trajectory that leads around the lake race track:
 
    steer_angle = -tau_p * CTE -tau_i * sum(CTE) -tau_d * d/dt CTE   

With tau_p, tau_i and tau_d beeing hyperparameter to tune for the specific application.   

The hyperparameter can be chose by trial-and-error, but a far better method is to tune the hyperparameter by using an algorithm called `twiddle` (also: coordinate descent). The algorithm optimises the hyperparameter in terms of a "local hill climber", according to the following logic (programmed in python:   

1. Initialize the parameter for tau:   

       p = [0, 0, 0]    
   
2. Set the change values for the algorithm:   

       dp = [1, 1, 1]   

3. Calculate the error:   

       best_err = A(p)   

       threshold = 0.001   

       while sum(dp) > threshold:   
         for i in range(len(p)):   
          p[i] += dp[i]   
          err = A(p)   

          if err < best_err:   # if improvement store it and increase it
            best_err = err
            dp[i] *= 1.1
          else:                # if no improvement go in other direction
            p[i] -= 2 * dp[i]
            err = A(p)

            if err < best_err: # if improvement store it and increase it
              best_err = err
              dp[i] *= 1.05
            else:              # if no improvement decrease value
              p[i] += dp[i]
              dp[i] *= 0.95

By using twiddle I found got good results for the hyperparameter tau_p, tau_i, tau_d.