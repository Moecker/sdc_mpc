# Reflection Intro

This file contains the Rubric's questions and answers to those where requested.

---

# Rubrics

## Compilation

### Your code should compile.

Code must compile without errors with cmake and make.

Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

* Code does compile using cmake && make.
* Tested using Ubuntu 16.04 and Term 2 Udacity Simulator and the installation script.

The code has been restructured a bit, so that there is now basically four parts:

* The `Main` file which contains all the simulator code and triggers the solve method of the MPC
* The `MPC` itself with definition of all constraints
* The `FgEvaluator` for the cost computation of a prediction
* The `CoordinateSystems` class used for coordinate transformations as discussed later
* The `Auxiliary` file containing helpers for all modules

The code and approach was influenced by two contributions:

* The MPC quizzes found under `https://github.com/udacity/CarND-MPC-Quizzes`. In particular the constraints and fg_evaluator implementation was influenced from here.
* This blog entry `https://medium.com/@NickHortovanyi/carnd-controls-mpc-2f456ce658f`. In particular, the idea to first coordinate transform the waypoints and to keep track of historic controls
* These GitHub repos: `https://github.com/hortovanyi/CarND-MPC-Project` and `https://github.com/ajsmilutin/CarND-MPC-Project`. In particular the coordinate transformation was influences from here.


## Implementation

### The Model

Student describes their model in detail. This includes the state, actuators and update equations.

* The state contains of four variables: The position `x`, the position `y`, the heading (or orientation) `phi` and the velocity `v`. This choice of values comes close to real car dynamics, though more advanced considerations as a drift-angle or wind-resistance (such as a cw value) are not taken into account.
* The coordinate system of the vehicle is defined as such: The x-axis point to the front, the y-axis point to the left by taking the rear axis' center as the system's origin.
* The model uses two actuators: Steering and throttle. Also this comes close to what can be observed in a real car - in fact those are the only two options how a real driver controls the car.

The update equations can be found in the `auxiliary.h` file; they are defined as:

* `px += velocity * cos(phi) * dt;`
* `py += velocity * sin(phi) * dt;`
* `phi += velocity / Lf * steering * dt;`
* `velocity += throttle * dt;`

### Timestep Length and Elapsed Duration (N & dt)

Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

* In general there is a trade-off between an exact solution (or best prediction) and required runtime.
* The variable `N` (timestep length) influences the foresight of the model prediction. An increased value results in a longer distance where a prediction can be made.
* The variable `dt` (elapsed duration between timesteps) influences the exactness and smoothness of the prediction. An increased value results in smaller gaps between predicted trajectory points and hence increases the prediction quality.
* An ideal solution would set both variables to quite high values. Real time requirements, however, restrict this. 
* The final choice was: `N = 8` and `dt = 0.5`
* During lectures the first choice was N = 10 and dt = 0.5. Those do not really deviate from the initial suggestion.
* Interestingly, another effect could be observed during parameter tuning: A too high value of N (starting at around 12) made the MPC prediction worse as it tried to fit the polynomial for too distant points. 
* As expected, the dt value influenced the smoothness of the prediction.
* With chosen values the systems runs quite smoothly and very fast.

### Polynomial Fitting and MPC Preprocessing

A polynomial is fitted to waypoints.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

* The approach taken does a full coordinate transformation of waypoint prior to the actual MPC procedure. 
* My first approaches were struggling with different coordinate systems (basically map vs.. vehicle) THis is why the decision was made to compute everything in the vehicle coordinate system (or so called local system).
* The received ptxs and ptys in map coordinate system from the json message is transformed into local coordinate system. This approach also influenced the state of the vehicle, so that px, py and phi was by definition set to zero.

### Model Predictive Control with Latency

The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

* To account for the 100ms delay, a countermeasure must be installed in the MPC procedure. The choice taken in this project is to estimate the vehicle's position 100ms to the front, as if the car has actually already moved there. The result is a trajectory which is closer to the desired one.
* More remarks: The latency did not directly influenced my approach or chosen parameters. The prediction was good enough to have a reasonable long foresight so that the latency did not kick in too much.
* Also my choice of max speed constraint (around 50 km/h) was set that low that the effect of latency was low.
* One addition to this, one feature has been added: The solution uses a weighted average of previous control outputs (it is called historic steering and throttle value) with current predicted ones. This increased smoothness and robustness for partial bad model predictions.

## Simulation

The vehicle must successfully drive a lap around the track.

No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

* As can be seen in the result/ folder, the car does drive multiple laps on the track.

