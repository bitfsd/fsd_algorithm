# skidpad_detector

this ROS package is an open source version of skidpad_detector from BITFSD.

ICP matching is used to get the transform between lidar cluster points and known maps based on the rules of FSAC.

## Important Dependencies
* ROS
* Eigen
* PCL
* catkin_tools

---
## ROS topic:
* subscriber:
	- /perception/lidar_cluster (sensor_msgs/PointCloud)
* publisher:	
	- /planning/global_path (nav_msgs/Path)

---
# Key parameters

All parameters are set in the `./config/skidpad_detector.yaml`  

`length/start` : Distance between track center and starting line  
`length/lidar2imu` : Distance between LiDAR and imu  
`length/threshold` : max threshold distance for points matching  

## Step

* 1. Move package to your workspace, Copy ` ./config/path_x.txt ` , ` ./config/path_y.txt` ` ./config/skidpad.pcd` to `/home/usrname/.ros/skidpad_detector` (if not exist, create it)
* 2. Go to your workspace,  `catkin build`
* 3. `source devel/setup.bash`
* 4. `roslaunch skidpad_detector skidpad_detector.launch`
