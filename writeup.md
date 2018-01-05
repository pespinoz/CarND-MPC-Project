
## **MPC Project**
The goals of this project are the following:

* The code should compile with `cmake` or `make` without errors.
* Describe the vehicle model in detail (state, actuators and update equations). 
* List reasons for the chosen `N` (timestep length) and `dt` (elapsed duration 
between timesteps).
* Describe pre-processing of waypoints, the vehicle state, and/or actuators 
before the MPC procedure.
* Implement Model Predictive Control that, additionally, handles a 100 millisecond 
latency.
* The car should complete a whole lap around the track in a safe manner.

[//]: # (Image/Video References)
[vi01]: ./video.mp4

---

### Files Included:
Here I list the files I modified to complete this project:

* `src/main.cpp:` The main file of the project. Here the mpc object is initialized, 
we pre-process waypoints and transform coordinate systems, we call the MPC solver, 
and interaction with the simulator takes place.  
* `src/MPC.h:` Header file of the MPC class.
* `src/MPC.cpp:` Source file of the MPC class. Here the parameters of the controller 
are defined, and the method `Solve` (which calls on `Ipopt`) is implemented.
* `./writeup.md:` You're reading it!
* `./video.mp4:` A video successfully showing the vehicle driving a lap around 
the track.

--- 

### Running the Code:

The code was implemented in CLion / Ubuntu 16.04.3 LTS.

To execute, do `cmake-build-debug/./mpc`, start the Term 2 simulator, and choose 
Project 5: MPC Controller. 

---

### [Rubric Points](https://review.udacity.com/#!/rubrics/896/view)

Here I will consider the rubric points individually and describe how I addressed each 
one in my implementation.  

---
### Writeup / README

#### 1. The Model

In this project MPC reframes the task of following a defined path over the track as 
an optimization problem. This involves i) simulating actuator inputs (throttle and 
steering), ii) predicting the resulting trajectory, and iii) selecting the trajectory 
with minimum cost (and thus extracting optimal actuations). This process is iterative by nature, as inputs are constantly computed over a future horizon; in a self driving car this is called "Receding Horizon Control". This horizon should not be greater than a few seconds (given how radically the environment can change at each time step).

The state of the car is given by the vector: 
```
[x, y, psi, v, cte, epsi]^T
```
that contains the variables for position, orientation, velocity, cross-track error and orientation error. The actuator vector is given by: 
```
[a, delta]^T 
```
which represents the throttle and steering, respectively.

The vehicle model kinematics (or update equations) come from a simplified approach we followed on the class lectures. These group of equations are implemented in `MPC.cpp`:
```
x(t+1) = x(t) + v(t) * cos(psi(t)) * dt
y(t+1) = y(t) + v(t) * sin(psi(t)) * dt
psi(t+1) = psi(t) - v(t) * delta(t) / Lf * dt
v(t+1) = v(t) + a(t) * dt
cte(t+1) = (f(t) - y(t)) + (v(t) * sin(epsi(t)) * dt)
epsi(t+1) = (psi(t) - psides(t)) - v(t) * delta(t) / Lf * dt
```

The optimization engine used in this MPC is called Ipopt (Interior Point OPTimizer) and receives as input i) the prior kinematic model, ii) constraints, and iii) the cost function to be minimized (more on this later!). 

For now, **regarding constraints**, we can say each variable of the state vector can have any value, in principle. The actuators on the other hand are limited by the physics of the car. The throttle has a limited range of [-1, 1], while the steering goes between (-25, 25) degrees (Unity limit). The last set of constraints come from the adoption of the kinematic model given above. This is valid for any if the six equations, but we'll exemplify with the position here:
  
```
x(t+1) - x(t) - v(t) * cos(psi(t)) * dt = 0
```
 
 Thus the left hand side has the constraint of being zero for any of the state equations.
 
#### 2. Timestep Length and Elapsed Duration 

The values of `N` and `dt` define the horizon `T`. The horizon control `T` can not be large, especially in a mobile application where the prediction can strongly be affected by environmental changes. The chosen values are shown below:

| N |   dt	 |   T horizon (s)    |
|:---:|:-----:|:--------:|
| 12 |  0.1  | 1.2  |


Essentially, what the Ipopt optimization engine does is to tune control inputs `(a, delta)` until a low cost vector of control inputs is found. The length of this vector is determined by `N`. Thus a large `N` yields to a large number of optimized variables, which is a computationally intensive task. Correspondingly, a small `dt` is also computationally intensive, but a large one results in less frequent actuations (discretization) error. 

I began by trying the values in the "Mind the Line" quiz of Lesson 19, which were `N = 25`and `dt = 0.05`. However this proved to be too intensive for the simulation and the car was less apt to stay around the track. Diminishing `N` and doubling `dt` to the values shown in the Table above made the smoother lap around the track, given the choice of other parameters (mainly related to the cost function). 
 
#### 3. Polynomial Fitting and MPC Pre-processing
    
The first thing we want to do in this Project is to make a coordinate transformation into the car's reference system. Having the car at the origin of our system considerably simplifies the equations and how we handle state values such as the cross-track error and the orientation error (and their evolution).

Thus we perform a rotation and reflection to go from (x, y) global positions (where the waypoints are defined) to the car's system. Here, the x-axis points in the direction of the car's heading and the y-axis points to the left. Therefore `x, y, and psi` are all zero in this system. We fit a 3rd order polynomial to the transformed waypoints, which consequently, lets us define both `cte` and `psi`.

We then pass our state and polynomial coefficients to the MP controller. Given a cost function that consider the minimization of the following quantities:

| Cost Function Factors	 |   Weights/Importance (relative units)   |
|:----:|:--------:|
|cross track error | 1000|
|orientation error | 1000|
|velocity w.r.t. 60 mph reference | 1|
|steering | 10|
|throttle | 10|
|steering transitions | 150|
|throttle transitions | 15|

Then Ipopt selects the trajectory with minimum cost -given the constraints of the model and actuators- and deliver us a vector with the corresponding control inputs. The idea is we apply the first control input (steering angle & throttle) and then repeat the loop in each timestep.

#### 4. Model Predictive Control with Latency

Latency refers to the delay of actuator inputs propagating through a real physical system (e.g. cars). MPC can handle latency graciously, as the delay can be incorporated into the vehicle model we described above.

Simply, we predict the state variables into a future given by the latency (in this case 100 ms) before feeding it to the MPC solver. This code is included in `main.cpp`: 
```
// Predicting state parameters for a latency of 100 ms
double latency = 0.1;
px = px + v * cos(psi) * latency;
py = py + v * sin(psi) * latency;
psi = psi - v * steer_value / Lf * latency;
v = v + throttle_value * latency;
```

It's worth noting that the PID controller of the previous Project ignores this issue.

#### 5. Simulation

Here's a [link to my video result][vi01], with the car driving around the track at a 
reference speed of 60 mph. Yellow line represents the reference path (given by waypoints), while the green line represents the MPC trajectory.
