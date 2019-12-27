# line_detector

this ROS package is an open source version of line_detector from BITFSD.

Hough Transform is used to get the boundary of acceleration based on the rules of FSAC.

## Important Dependencies
* ROS
* catkin_tools

---
## ROS topic:
* subscriber:
	- /perception/lidar_cluster (sensor_msgs/PointCloud)
* publisher:	
	- /planning/global_path (nav_msgs/Path)

---
# Key parameters

All parameters are set in the `./config/line_detector.yaml`  

`path_length` : Distance of the track.  

## Step

* 1. Move package to your workspace  
* 2. Go to your workspace,  `catkin build`  
* 3. `source devel/setup.bash`  
* 4. `roslaunch line_detector line_detector.launch`  
