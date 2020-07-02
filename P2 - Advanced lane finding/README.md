# Advanced Lane Finding
## Overview
![image](https://github.com/adamalavi/Self_driving_cars_Udacity-ND/blob/master/P2%20-%20Advanced%20lane%20finding/project-output.gif)

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

## Working
#### 1. Camera Calibration
---

Camera calibration is performed on the images of a chessboard of size 9x6. Multiple images of the board from different angles are provided. For all these images, we first try to find the corners and if found, these coordinates are appended to the list of image points. Then the object and image points are used to get the camera matrix and the distortion coefficients. These are then used to undistort the images. Examples are available in the notebook.

---
#### 2. Perspective transform
---

The source points are obtained manually by eye balling and then recheking by drawing the points on a test image. The destination points are then decided in such a way that the two lanes are parallel in the output image.

---
#### 3. Thresholding
---

Sobel function is used to find different gradients and then a combined condition is used to find the output for the gradients. Then a threshold is applied on the saturation and lightness channel which is then pooled with the gradient output which leads to pretty satisfactory results.

---
#### 4. Fitting a polynomial
---

After finding the thresholded frame in a video the polynomial coefficients to fit the lane line can be found out using two methods, namely, sliding window search or search from prior. If the lines were obtained in the previous frame then the search from prior method is used to detect the lines in the new frame. If satisfactory results are not obtained for this frame, then the brute search that is the sliding window search is performed. The points then obtained are used to fit a polynomial using the polyfit function.

---
#### 5. Radius of curvature
---

Once the pixels corresponding to the lanelines have been obtained, they are used in the polyfit function together with the conversion factor for conversion from pixel to meters and the radius of curvature is determined from a formula. The radius of curvature is averaged over the last 10 iterations to avoid jerks in the reading. Also, the offset is calculated as the difference between the center of the two lane lines at the bottom of the image and the image center.

---
#### 6. Drawing the lane line
---

The coordinates obtained on the bird's eye view image are then converted back to the real space using the inversion matrix after the lane is drawn from the polynomial coefficients obtained in the previous step.

---

