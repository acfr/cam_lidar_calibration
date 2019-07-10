# ROS package to determine the extrinsic calibration parameters (3D rotation and translation) between a camera and a lidar.

![alt text](img/sensors.png "Sensor setup")

## Experimental Setup
1. To make a calibration target, firmly attach a checkerboard on a rigid, opaque, and rectangular board such that both their centres align and their edges remain parallel to one another.
2. Mount the target on a stand such that it is tilted at an angle of 45-60 degrees with respect to the ground plane. 
3. Choose a stand to hang the target in a way that it does not hold the board with significant protruding elements close to the board boundaries or corners.

![alt text](img/Experimental_area.png "Experimental Setup")

## Data Collection
1. For calibration, it is required to place the target in different locations and orientations relative to the camera-lidar sensor pair.
2. Make sure that the 3D experimental region in which the target will be placed is free from any other objects apart from the board and its stand. 
3. Ensure that the entire board is visible to both sensors. 
3. A minimum of 3 checkerboard poses or samples are to be collected. Satisfactory calibration results are obtained with around 9 samples. 
In general, calibration accuracy improves as more samples are collected.

![alt text](img/Checker.png "checkerboard poses")

## Pre-requisites (to be entered in cfg/initial_params.txt in the described order)
1. Name of the image topic 
2. Name of the point cloud topic 
3. Type of camera lens (1 for fisheye; 0 for pinhole)
4. Number of lidar beams (Eg. 16, 32, 64 etc)
5. Size of the checkerboard (Eg. 8X6 written as 8 6)
6. Side length of each square in the checkerboard in millimetres (mm) (Eg. 65)
7. Length and breadth of the target in mm (Eg. 905 600 ; length = 905 mm, breadth = 600 mm)
8. Error in checkerboard placement along the length and breadth of the rectangular board. 
Ideally, the checkerboard centre should align with the board centre. However, if that's not the case, you can account for the translational errors along the length 
and breadth of the board atop which the checkerboard is fixed. 
For error along the length, if the checkerboard centre is above the board centre, the error (in mm) is positive else it is negative.
For error along the breadth, if the checkerboard centre is shifted to the right of the board centre, the error (in mm) is positive else it is negative. 
(Eg. 20 0; +20 mm error along the length of the board and 0 mm error along the breadth of the board. i.e in our case the checkerboard is shifted up from its 
original position by 20 mm)

![alt text](img/positive_length.png "Experimental Setup")
![alt text](img/xpositive.png "Experimental Setup")

9. 3X3 Camera intrinsic matrix (Units in mm)
10. Number of camera distortion coefficients (Our distortion function uses 4 coefficients for fisheye lens and 5 coefficients for pinhole lens)
11. Camera distortion coefficients
12. Image Size in pixels (Eg. 1920 1208)

## Procedure 
1. Start the Rosmaster and open Rviz for visualization. 

`$roscore`

`$rviz`


2. Firstly, we determine the 3D experimental region where the target will be placed. This is done with the help of a GUI where slider bars can be varied to 
set the minimum and maximum bounds of the 3D experimental region along the lidar's x, y, and z axis. The visualization can be simulataneously seen in Rviz.
In Fig. 2, the region is illustrated by a orange box.
Run the commands below to open up a GUI with slider bars (with x, y, z bounds) and to visualize the features on the checkerboard (board plane normal vector, 
board centre and corner points).  

`$rosrun nodelet nodelet manager __name:=nodelet_manager`

`$rosrun nodelet nodelet load extrinsic_calibration/feature_extraction nodelet_manager`

`rosrun rqt_reconfigure rqt_reconfigure`

3. In Rviz, subscribe to the topics "Experimental_region", "velodyne_features", "camera_features", "visualization_marker", and "boardcorners"



