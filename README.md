# Multiple Bugs Tracking  using Kalman Filter

This project is a python implementation of the Kalman filter algorithm for multiple bugs tracking in 2D.

## Algorithem steps
1. Detecting Multiple Bugs using Laplacian of Gaussian
2. Intializing Current state estimation `q_estimate` in class `KalmanF` with first frame detections `q`.
3. Strat Tracking in frame `s_frame`
4. Assign new detections to Tracking `q_estimate` using Hungarian Algorithem.
5. Predict the next step in function `predictK(self,tracke)`.
6. Calculate Kalman gain `calcKalmanGain()`
7. Correct current state `kalmanEstUpdate()`.

## Libraries Used
* Numpy
* skimage
* scipy
* random
* moviepy
* OpenCV

## Project Files
* KalmanF.py contain Kalman Tracking class
* Bug_Tracking.ipynb contains detection and tracking pipline