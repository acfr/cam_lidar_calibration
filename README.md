ROS package to determine the extrinsic calibration parameters between a camera and a lidar.

Experimental Setup:
1. To make a calibration target, firmly attach a checkerboard on a rigid, opaque, and rectangular board such that both their centres align and their edges remain parallel to one another.
2. Mount the target on a stand such that it is tilted at an angle of 45-60 degrees with respect to the ground plane. 
3. Choose a stand to hang the target in a way that it does not hold the board with significant protruding elements close to the board boundaries or corners.

Data Collection: 
1. For calibration, it is required to place the target in different locations and orientations relative to the camera-lidar sensor pair.
2. Make sure that the 3D experimental region in which the target will be placed is free from any other objects apart from the board and its stand. 
2. Ensure that the entire board is visible to both sensors. 

Pre-requisites: (to be entered in cfg/initial_params.txt in the described order)
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