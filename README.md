# Extended Kalman Filter
Self-Driving Car Engineer Nanodegree Program @ udacity.com

[//]: # (Image References)
[image1]: ./media/ekf.png
[image2]: ./media/ekf2.png
[image3]: ./media/ekf3.png

This project uses a Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Extended Kalman filter algorithm is used to process radar data.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]


## Sample Output
Examples of algorithm performance. Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

![alt text][image1]

![alt text][image2]

![alt text][image3]
