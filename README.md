# Self-Balancing-Robo

This repository contains the code for balancing a single-link, single-axle, 2-wheel robot.


1. I have implemented a naive-version of Proportional-Integral-Derivative (PID) control algorithm for self-balancing.

2. I also implemented an assistive offset calibration algorithm, which helps to customize equilibrium position at the start of the arduino without hard-coding the sensor value. This can helps to automatically sense the equilibrium position if the robot is held manually at the equilibrium position for first few seconds.


I used a MPU6050 IMU sensor to sense angular position and rate of change of angular position. I2C lib (I2C.ino) for communication with the sensor was taken from 

Kristian Lauszus, TKJ Electronics
Web      :  http://www.tkjelectronics.com
e-mail   :  kristianl@tkjelectronics.com
 
Kalman Filter library (Source: https://github.com/TKJElectronics/KalmanFilter) was used to implement Kalman filtering. 
