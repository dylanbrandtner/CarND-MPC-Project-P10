# Model Predictive Control Project

[//]: # (Image References)
[image1]: ./doc/Intro.JPG  "intro"
[image2]: ./doc/MPC.JPG  "MPC"

## Project Introduction
In this project, I implemented Model Predictive Control (MPC) in C++ to drive a car around a track. The model takes in a reference trajectory and state information like vehicle position, orientation, and speed.  It then determines the appropriate actuator outputs (i.e. steering angle and throttle) that minimize the "cost" of the predicted trajectory.  The "cost" function is tuned to give us the smoothest driving experience around the track.  In the project simulator, the result looks like this:

![alt text][image1]

The reference trajectory is shown in yellow and the predicted trajectory from the model is shown in green.  

## [Rubric Points](https://review.udacity.com/#!/rubrics/896/view)

Here I will consider the rubric points individually.  

### Compilation
This project uses the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

The main program can be built and run by doing the following from the project's top level directory.

1. mkdir build && cd build
2. cmake .. && make
3. ./mpc

Note: Before doing so you must also run install_ipopt.sh.  See [this documentation](./install_Ipopt_CppAD.md) for more details.

### Implementation

#### The Model

The MPC model is broadly described in the course materials on the following slide:

![alt text][image2]

The state information kept at each step includes the following:

| Variable | Description        |
|:--------:|:------------------:|
|     x    | X position         |
|     y    | Y position         |
|     ψ    | Orientation        |
|     v    | Velocity           |
|    cte   | Cross Track Error  |
|    eψ    | Orientation Error  |

This state information is part of the "Kinematic Model" for the vehicle.  We calculate the values for these states at time t+1 using the equations shown in the above slide.

We then use the `CppAD` library to optimize our control inputs (i.e. "actuators"), which in this case are steering angle and throttle, to minimize the "cost" of the predicted trajectory. The "cost" is calculated at each step by assigning penalties to various aspects of the state that are unfavorable.  Using the examples from the course materials, I considered 7 such costs.  Below is a table that lists those costs, and their relative weights in the cost calculation. 

| Cost                                          | Weight |
|:---------------------------------------------:|:------:|
| CTE                                           |    500 |
| Orientation Error                             |    500 |
| Difference from reference velocity (40mph)    |      2 |
| Steering Angle                                |  5,000 |
| Throttle Value                                |     10 |
| Change in Steering Angle                      |  5,000 |
| Change in Throttle                            |      1 |

Each cost is squared and then multiplied by the weight.  I chose these weights based on experimentation and general intuition about their importance for a smooth and safe driving pattern. More on this is covered in the "Discussion" section below.  

The model can be found in the `MPC.cpp` file.  The majority of the work was taken from the [MPC Quizzes lab](https://github.com/udacity/CarND-MPC-Quizzes).  I marked most of my own changes with a comment that starts with "//Dylan:".

#### Timestep Length and Elapsed Duration

I chose a time delta of 100ms and 10 total time steps for calculating the predicted trajectory. When experimenting with smaller time deltas, I noticed impacts to performance that affected my latency.  When experimenting with more time steps, I saw some issues with performance, but also noticed the prediction was looking too far ahead and thus data that was too far in the future was impacting my current actuators in ways it should not.

#### Polynomial Fitting and MPC Preprocessing

The first step in the control loop is to take the input list of reference waypoints and convert them into the vehicles own coordinate system.  We do this using trigonometry and the input x,y, and orientation values. We reset our x, y, and orientation values to 0 now because we are in the car's perspective. 

After this, we fit a 3rd order polynomial to these transformed points, and this becomes our reference trajectory.  We use this to calculate initial CTE and Orientation error. We then pass all of this initial state information into our model, along with the reference trajectory, and use the algorithms described in "The Model" section to find our optimal actuator values.  We send these actuator values to the controller.  

Polynomial fitting and MPC preprocessing can be seen in the `main()` function of `main.cpp`.  Again, I've marked the area modified with a code comment that has my name.

#### Model Predictive Control with Latency

A real car would have latency between the time an actuation command is sent and when it actually propagates through the system. As noted in the lesson materials, a realistic value for this is 100ms, so this is the value used. To account for this, when calculating the state information, I used an older set of actuation values.  Since 100ms is the same as my timestep, this results in using the actuation values from 1 step behind.  Additionally, I had to compensate for imperfect measurements by heavily penalizing over-steering in my cost calculation.  I'm not sure this was the best way to compensate for latency, but I experimented with other methods and had no success.  

Also, it should be noted that although the amount of latency applied was a constant 100ms, the use of a thread sleep to simulate this is non-deterministic.  A thread sleep will tell the OS to yield the CPU, and the thread will remain blocked for amount of time specified.  However, the thread will not necessarily be re-scheduled again immediately after the 100ms expires.  Various other factors (ex. scheduling algorithm, time slices, thread priorities, etc.) could increase this simulated latency. I noticed my model preformed very poorly if there were too many other processes using resources on my machine.

More on Latency is covered in the Discussion section below.

### Simulation

The rurbic requires "No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle)."  I was able to achieve this using the code submitted. 

Here's a [link to a video recording of my final result](./doc/project_recording.mp4).  

Again, note the above details about latency.  My model will not preform optimally if the "mpc" process does not have sufficient CPU cycles to execute, as this will artificially increase the latency.

## Discussion

I found this to be a very difficult project for several reasons.  Here are some of the concepts I struggled with along the way:

### Starter/example code
In general, there was little guidance on how the model needed to be implemented besides the source code provided in the MPC quizes.  This source was highly specialized to the examples in some non-obvious ways. For example,

## Initial transforms

I had to go back to notes about transforming coordinates in the Particle filter lesson and update the equations 


### Data formats

Sequential data formats confusing

### Polynomial order

Had to update some equations to use 3rd order instead of 1st order.

### Latency

Unclear best way to deal with this.  Non-deterministic.
