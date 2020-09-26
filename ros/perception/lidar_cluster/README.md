# Lidar Cluster

This ROS package is an open source version of lidar_cluster from BITFSD. We removed some components to keep it simple but it still has great performance.  

## Algorithm

The core idea of this algorithm is euclidean clustering. First, we filter out the point clouds on the ground, and filter out the point clouds that meet the feature of the cones, then cluster them.

## Important Dependencies

* ROS
* PCL

---

## ROS topic:

* subscriber:
  - /velodyne_points (sensor_msgs/PointCloud2)
* publisher:	
  - /perception/lidar_cluster (sensor_msgs/PointCloud)

---

# Key parameters

All parameters are set in the `./config/lidar_cluster.yaml` 

## Prerequisites

You need a LiDAR sensor to generate point clouds.

## Step

* Move package to your workspace.
* Go to your workspace,  `catkin build`
* `source devel/setup.bash`
* `roslaunch lidar_cluster lidar_cluster.launch`