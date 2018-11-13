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

A real car would have latency between the time an actuation command is sent and when it actually propagates through the system. As noted in the lesson materials, a realistic value for this is 100ms, so this is the value used. To account for this, when calculating the state information, I used an older set of actuation values.  Since 100ms is the same as my timestep, this results in using the actuation values from 1 step behind.  This solution was adapted from one suggested by other students in the [slack channel for this project](https://carnd.slack.com/messages/C54DV4BK6/convo/C54DV4BK6-1520754320.000033/).

Additionally, I had to compensate for imperfect measurements by heavily penalizing over-steering in my cost calculation.  I'm not sure this was the best way to compensate for latency, but I experimented with other methods and had no success. More on Latency is covered in the Discussion section below.

Also, it should be noted that although the amount of latency applied was a constant 100ms, the use of a thread sleep to simulate this is non-deterministic.  A thread sleep will tell the OS to yield the CPU, and the thread will remain blocked for amount of time specified.  However, the thread will not necessarily be re-scheduled again immediately after the 100ms expires.  Various other factors (ex. scheduling algorithm, time slices, thread priorities, etc.) could increase this simulated latency.  I noticed my model preformed very poorly if there were too many other processes using resources on my PC.

### Simulation

The rurbic requires "No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle)."  I was able to achieve this using the code submitted. 

Here's a [link to a video recording of my final result](./doc/project_recording.mp4).  

Again, note the above details about latency.  My model will not preform optimally if the "mpc" process does not have sufficient CPU cycles to execute, as this will artificially increase the latency.

## Discussion

I found this to be a very difficult project for several reasons.  Here are some of the concepts I struggled with along the way:

### Starter/example code
In general, there was little guidance on how the model needed to be implemented besides the source code provided in the MPC quizzes.  This source was highly specialized to the examples in some non-obvious ways. In particular, the sample code used 1st order polynomials, and it took some time to realize this issue in the `FG_eval` class.  I had to go back and check all the calculations, and at that point I realized `ψdes` was using a derivative, and the `psides0` var had the derivative of a 1st order polynomial. 

### Data formats

The sequential data formats for passing information between functions was very confusing.  I'm guessing this was an artifact of the CppAD API, but it made the various indexing operations tedious and error prone.  It would have been much more logical to me to store things in multiple vectors than a single `fg` vector. 

### Initial transforms

It took a while to figure out how to transform the initial data points into the car's perspective. I had to go back to notes about transforming coordinates in the Particle filter lesson and then form new equations from that idea (since those were to preform the opposite transformation). I again had to rely on the slack channel to check my final work when I got stuck.  I wish there had been a quick tutorial on this in the lessons to avoid this confusing bit of math.

### Latency

As noted above, the most complex part of this lesson was dealing with the latency.  The fact that it was non-deterministic, and impacted by CPU utilization made experimentation results very unreliable.  Processor usage had such an impact, that I initially was unable to record a clip of my passing course run.  I had to remove all debug output, and close everything else on my PC to get a clean recording.  

To account for the latency, I tried following the general suggestion from the course materials (i.e. running a simulation from the current state for the duration of the latency), but I never got any good results with this.  I suspect this increased processor load and thus further impacted latency and thus variability (or perhaps I just made a mistake somewhere in the calculations?).  My final approach of using older actuator values was a bit simpler to code up and had almost no processor penalty, but it still seemed to cause my predicted path to over-steer in the short term, which lead to very uneven steering on straight roads.  I suspect this was because I could only use this approach _after_ my first step in the predicted path, which meant my first prediction point was still using the wrong actuator values.  I eventually had to compensate for this with heavy steering penalties, but wasn't particularly satisfied with the result.    
