# PID-Controller
Control algorithm to control throttle/brake and steering to control a vehicle smoothly around a track

![](results/pid_brake.gif)

---
# Introduction
### Goal:
This project implements a PID controller in C++ to maneuver the vehicle around the track in the [simulator](https://github.com/udacity/self-driving-car-sim.git)! Two PID controllers are to be implemented one for steering control and another for throttle control.

# Fundamentals
PID stands for Proportional-Integral-Derivative. It is a controller with three coeffients P, I and D. The effects of these coefficients are discussed below.

 ### Cross Track Error (CTE)
 * It is a distance between the vehicle´s actual trajectory and the groundtruth trajectory. It is best suited to control the vehicle by steering in proportion to CTE.
 
 ### Systematic Bias
* This is a term used in Robotics that tells you how much the vehicle´s steerable wheels are aligned.
 
### P- Controller 
* It sets the steering angle in proportion to CTE by virtue of a gain parameter called tau_p.

`steering angle = -tau_p * cte`

* This controller steeers the vehicle towords the trajectory but when it reaches the trajectory it overshoots.
* The following graph demonstrates the overshooting due to P controller.

<p align="center">
  <img width="400" height="200" src="results/p.png">
</p>

### PD- Controller 
* In this controller the steering angle is not just proportional to CTE by virtue of tau_p but also the temporal derivative of the CTE with a gain parameter called tau_d.

`steering angle = -tau_p * cte - tau_d * diff_cte`

* This helps the vehicle to not overshoot the trajectoy as the the derivative component counter steers the vehicle.
* The following graph demonstrates the behaviour of P controller and PD controller.

<p align="center">
  <img width="400" height="200" src="results/pd.png">
</p>

### PID- Controller
* Finally this controller overcomes the problem of overshooting and systematic bias by adding one more term.

`steering angle = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte`

* This time, addition to PD controller parameters the steering angle is prportional to integral of all CTE over time. 
* The following graph summerizes the P-, PD-, and PID- controllers.

<p align="center">
  <img width="400" height="200" src="results/pid.png">
</p>

# Implementation
Manual tunning of PID coefficients for steering and throttle values.

#### Tunning P- Controller
The vehicle drives along the trajectory with oscillations.

![](results/p.gif)

#### Tunning PD- Controller
The vehicle follows the trajectory with relatively low oscillations.
![](results/d.gif)

#### Tunning PID- Controller 
The vehicle follows the trajectory but due to overspeeding it shakes while taking turns.
![](results/pid.gif)

#### PID- Controller for throttle value
The braking while taking turns allows the vehicle to take the turns smoothly.
![](results/pid_brake.gif)
