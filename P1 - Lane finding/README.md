# Finding Lane Lines on the Road 
## Overview
---
![image](https://github.com/adamalavi/Self_driving_cars_Udacity-ND/blob/master/P1%20-%20Lane%20finding/solidWhiteRight.gif)

Being the first project, this was a fairly simple project where lane lines were detected using basic concepts of OpenCV in Python. The lines were detected using the Hough lines function and other color transform and masking functions.

## Working
The pipeline consisted of various steps. First, the image is converted to grayscale and a gaussian blur is applied on the image. This blurred gray image is then applied as input to convert it into a canny image. Then a set of vertices for a quadrilateral are set that will set the vertices for a region of interest. This is done because the region of interest almost always stays the same because of the fixed position of the camera. Then the Hough lines function is used to find the lines in the ROI. Now that the lines are obtained, there is a need for filtering and extrapolating the lane lines. This is done in the draw lines function where first the slope for all the lines obtained in the Hough function is calculated and based on the polarity of the slope the line segment is sort into two groups: right or left lane. The segments are then filtered and the line segments whose difference between the slope and mean slope is not less than 1.5 times the standard deviation are rejected. The mean slope and intercept is calculated again after filtering and these coordinates are then used to plot the lane lines.
