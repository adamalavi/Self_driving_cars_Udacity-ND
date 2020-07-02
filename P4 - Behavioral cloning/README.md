# Behavioral Cloning Project
## Overview
![image](https://github.com/adamalavi/Self_driving_cars_Udacity-ND/blob/master/P4%20-%20Behavioral%20cloning/Recordings/Behavior%20cloning.gif)

A deep convolutional neural network was trained using Keras framework to mimic the driving behavior of humans. The dataset was created with around 25000 images captured from 3 cameras on the vehicle and labelled with the steering angle at that point of time. The architecture used was the one presented by Nvidia in their paper on end to end learning for self driving cars. A [simulator](https://github.com/udacity/self-driving-car-sim) was provided where you can steer a car around a track for data collection. 

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior 
* Design, train and validate a model that predicts a steering angle from image data
* Use the model to drive the vehicle autonomously around the first track in the simulator. The vehicle should remain on the road for an entire loop around the track.

## Working
#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md or writeup_report.pdf summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable
The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model consists of a convolution neural network with 3x3 and 5x5 filter sizes and depths between 24 and 64
The model includes RELU layers to introduce nonlinearity, and the data is normalized in the model using a Keras lambda layer. 

#### 2. Attempts to reduce overfitting in the model

The model contains dropout layers in order to reduce overfitting. The model was trained and validated on different data sets to ensure that the model was not overfitting. The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 25).

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road and traversing the whole track in the opposite direction as well.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

My first step was to use a convolution neural network model similar to the one used by NVIDIA in their self driving cars and published in their paper on [end to end deep learning for autonomous vehicles](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf). I thought this model might be appropriate because it is previously tried and tested on self driving cars and contained multiple convolutional layers to obtain the geometric details that are required for obtaining the steering angles.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. I found that my model had a low mean squared error on my training as well as validation set because I had used a dropout layer to avoid overfitting.
The final step was to run the simulator to see how well the car was driving around track one. The car drove perfectly well around all the curves and the whole track.

##### 2. Model Architecture
My final model consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 160x320x3 RGB image   						| 
| Lambda         		| 160x320x3 RGB image normalised				| 
| Cropping         		| 65x320x3 RGB image   							| 
| Convolution 5x5     	| 2x2 stride, depth = 24 						|
| RELU					|												|
| Convolution 5x5     	| 2x2 stride, depth = 36 						|
| RELU					|												|
| Convolution 5x5     	| 2x2 stride, depth = 48 						|
| RELU					|												|
| Convolution 3x3     	| 1x1 stride, depth = 64 						|
| RELU					|												|
| Convolution 3x3     	| 1x1 stride, depth = 64 						|
| RELU					|												|
| Flatten				|												|
| Fully connected		| Outputs 100									|
| Dropout				| Probability 25%								|
| Fully connected		| Outputs 50									|
| Output				| Output 1 value								|
|						|												|

#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded one lap on track one using center lane driving.
I then recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to rectify its course if it ever goes off-track. Then I repeated this process on track two in order to get more data points. To augment the data sat, I also flipped images and angles thinking that this would lead to more generalisation.
I also used the images captured from the left and the right cameras and applied a correction factor of 0.2 to the steering angle to account for the placement of the camera.

After the collection process, I had approximately 46000 data points including the augmented data. I then preprocessed this data by normalising the pixel values so that all values lie between -1 and 1. Then I cropped the images to ignore the parts of the frame that were not useful. I finally randomly shuffled the data set and put 20% of the data into a validation set. 
