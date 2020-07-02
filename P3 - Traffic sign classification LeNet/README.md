## Project: Build a Traffic Sign Recognition Program
Overview
---
In this project, concepts of deep neural network and convolutional neural network was used to train and validate a model so it can classify traffic sign images using the [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset). After the model is trained, a few images of German traffic signs were downloaded from the web that were not a part of the dataset and then tested. A modified version of the [Lenet architecture](http://yann.lecun.com/exdb/publis/pdf/lecun-01a.pdf) was used to achieve an accuracy of approximately 95%.
![image_viscnn](https://github.com/adamalavi/Self_driving_cars_Udacity-ND/blob/master/P3%20-%20Traffic%20sign%20classification%20LeNet/visualize_cnn.png)

The goals / steps of this project are the following:
* Load the data set
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


## Working

### Data Set Summary & Exploration
This was the first step of the project and the size of the train, test and validation data was explored. Also a few random images were plotted in the notebook and their label was printed as the title.

### Design and Test a Model Architecture
##### 1. Preprocessing
For pre-processing the images, the pixel values were normalized to a value between -1 to 1. This was done so that convergence was achieved sooner. The images were not converted to grayscale because I believe the red and the blue colour in the images could have been benificial in the classification process. The data was then shuffled.

##### 2. Model Architecture

![image_lenet](https://github.com/adamalavi/Self_driving_cars_Udacity-ND/blob/master/P3%20-%20Traffic%20sign%20classification%20LeNet/lenet.jpg)

A modified version of this Lenet architecture was used and the changes can be noticed in the table given below. My final model consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 32x32x3 RGB image   							| 
| Convolution 5x5     	| 1x1 stride, valid padding, outputs 28x28x6 	|
| RELU					|												|
| Max pooling 2x2		| 2x2 stride,  outputs 14x14x6 				|
| Convolution 5x5	    | 1x1 stride, valid padding, outputs 10x10x16	|
| Activation            |												|   
| Convolution 5x5		| 1x1 stride, valid padding, outputs 1x1x400	|
| RELU					|												|
| Concatenation			| flattened pooling 1 and conv3, outputs 800	|
| Dropout				|												|
| Fully connected		|Input: 800, outputs 43							|
|						|												|

##### 3. Training
For training the logits were applied to the softmax function and cross entropy was used to calculate the cost. This cost was then minimised using the Adam optimiser. Epoch value was set to 100, learning rate 0.001 and batch size was chosen as 300.

##### 4. Approach
The LeNet approach discussed in the lab was first tried but even after tuning the hyperparameters, it failed to provide a validation accuracy above 93%. Then the architecture was slightly modified so that the flattened output of the layer1 and the last convolutional layer both of size 800 were concatenated to incorporate some geometric features that mighthave been detected in the first layerin the final fully connected layer as well. This layer of size 800 was then reduced to 43 to obtain the final results.

My final model results were:
* validation set accuracy of 95.4 
* test set accuracy of 94.5

### Test a Model on New Images
Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| Right of way			| Right of way 									| 
| General caution		| General caution								|
| Left turn				| Beware of ice									|
| 60 km/h	      		| 60 km/hr						 				|
| 30 km/h	      		| 30 km/hr						 				|
| Keep Right			| Keep right      								|

The model correctly classified 5 out of 6 pictures getting an accuracy of 83%.
The softmax values for the predictions were observed and all the values were 1.00 except for the one image that was misclassified which had a probability of 0.75.
The model seems to be compatible with changing contrasts, saturation, etc. One image was misclassified and that was probably because of the different angle the image was captured from. To account for this, maybe a little augmentation can be performed on the training data at the beginning.
