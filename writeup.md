
## **PID Control Project**
The goals / steps of this project are the following:

* Your code should compile with `cmake` or `make` without errors.
* Implement a PID controller for the steering angle of the car. 
* Optionally implement another PID to control the speed of the car.
* Tune/Optimize the parameters of the controller(s) and explain your choice.
* The car should complete a whole lap around the track in a safe manner.
* Record a video of the car in the simulator. 

[//]: # (Image References)

[im01]: ./figures/cte_and_steering.jpg
[im02]: ./figures/speed_and_throttle.jpg
[vi01]: ./video.mp4

---

### Files Included:
My project includes the following files:
* `src/main.cpp:` The main file of the project; here I define the parameters of the controllers, call the PID class, interact with the simulator, and output variables to plot later with `matplotlib`.  
* `src/PID.h:` Header file of the PID class.
* `src/PID.cpp:` Source file of the PID class.
* `figures/plots.py:` Python script to plot the controlled variables of the PID controllers implemented in `main.cpp`.
* `./writeup.md:` You're reading it!
* `./video.mp4:` A video successfully showing the vehicle driving a lap around the track.

--- 

### Running the Code:

The code was implemented in CLion / Ubuntu 16.04 LTS.

To execute, do `cmake-build-debug/./pid` and then start the Term2 simulator. 

Additionally, we produce plots by executing `python figures/plots.py`. 

---

### [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points

Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Creating a PID controller for the steering angle

This is quite straightforward, and is implemented in the following lines of the `PID.cpp` (class).
```
void PID::UpdateError(double cte, double delta_timestamp) {
    d_error = (cte - p_error) / delta_timestamp;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    return control_value = Kp*p_error + Kd*d_error + Ki*i_error; 
}
```
The parameters used for this PID controller were:

| Kp |   Ki	 |   Kd    |
|:---:|:-----:|:--------:|
| -0.06 |  0.0  | -0.098  |

#### 2. Implementing a PID controller for the throttle

I found, as it will be shown later, that to fine-tune the parameters of the steering PID it was easier to maintain the car in a specific operating point. This is equivalent to including a cruise-control for the car so a constant speed can be mantained.

The control action here is upon the throttle. I re-used the code from the PID class shown in the previous Section. The controller parameters I used were:

| Kp |   Ki	 |   Kd    |
|:---:|:-----:|:--------:|
| -0.30 |  0.0  | -0.001  |
    
#### 3. Tuning the PIDs hyper-parameters

The parameters of the PID controllers were set manually instead of using an algorithmic approach such as Twiddle. 
The reason for this is two-folded: i) Twiddle would be rather time-consuming to implement as in each iteration of the algorithm
the simulator would have to run in a significant portion of the track, ii) Manually tuning the parameters is useful 
to develop an intuition on what each of these parameters (and their variations) mean. 

We choose the parameters as follows:  First, all **Kp**, **Ki**, and **Kd** are set to zero. The first one I start 
increasing (in magnitude) is the proportional constant. This reduces the error initially (putting the car in the center 
of the lane), but in corners the controller is affected by significant oscillations. Next step is to increase the 
magnitude of the derivative constant. This reduces oscillations, but I find that in the corners the steering response
 is still not enough to carry the car through the whole track.
 
At that point I increase the magnitude of **Kp** to improve the steering response, reaching a value of -0.06. For 
that value of the proportional constant, I find a value of **Kd=-0.098** that manages to minimize the overshoot. 
Increasing or diminishing this last value can bring back the overshoot.

By this point (and through a similar process for the throttle controller) I find the car drives successfully around 
the track at ~40 mph. In this context varying the integral constant **Ki** is not needed, as the car steering doesn't
 show signs of bias. If there was, the value of **Ki** would be extremely small, but for me it works well set as zero.

Here we show the plots obtained with the Python script in `figures/plots.py:`. The first plot shows the cross-track 
error (controlled variable) and steering angle (action control) as a function of time.

![alt text][im01]

The second plot shows the car's speed (controlled variable) and throttle value (action control) as a function of time.
![alt text][im02]

#### 4. Video Implementation

Here's a [link to my video result][vi01], with the car driving around the track at ~40 mph.
