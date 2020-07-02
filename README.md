# Self_driving_cars_Udacity-ND
[Udacity's self driving cars nano degree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) that teaches a variety of concepts in different fields like Artificial intelligence, computer vision, deep learning, path planning, sensor fusion, behavior modelling, prediction and system integration. This repository contains all the nanodegree projects from all these different fields.

## Project 1 - Lane finding
Detecting lane lines using the basic concepts of computer vision like masking, thresholding, color conversions, Hough lines, etc.
## Project 2 - Advanced lane finding
Create a pipeline for advanced lane lines  and radius of curvature detection using concepts like camera calibration, distortion correction, color transforms, gradients, perspective transform, etc. The radius of curvature is obtained by using an algorithm called sliding window search.
## Project 3 - Traffic sign classification using LeNet
German traffic signs dataset was used to build a classifier with around 96% accuracy using the [LeNet architecture](http://yann.lecun.com/exdb/publis/pdf/lecun-01a.pdf) and other deep learning concepts using Tensorflow.
## Project 4 - Behavioral cloning
A deep convolutional neural network was trained using Keras framework to mimic the driving behavior of humans. The dataset was created with around 25000 images captured from 3 cameras on the vehicle and labelled with the steering angle at that point of time. The architecture used was the one presented by Nvidia in their paper on [end to end learning for self driving cars](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf).
## Project 5 - Extended Kalman filter
An extended Kalman filter that is widely used in the real world for sensor fusion was implemented in C++ and then tested out on a simulator.
## Project 6 - Particle filter
A particle filter that is widely used for localisation was implemented in C++ and tested on a simulator to see how well a vehicle localises itself in a known map with landmarks using the particle filter.
## Project 7 - Highway driving
A path planner was designed and implemented in C++ to drive an autonomous vehicle on a highway that can sense and predict the movement of other vehicles around it and considering all the data, plan a path to traverse through the highway without any collisions, jerk minimisation and no speed violations.
## Project 8 - PID control
A PID controller was designed and implemented in C++ to control the steering value of a car driving down a track in a simulator.
