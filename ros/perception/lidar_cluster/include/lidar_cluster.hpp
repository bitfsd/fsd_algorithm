/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2019:
     - chentairan <killasipilin@gmail.com>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef LIDAR_CLUSTER_HPP
#define LIDAR_CLUSTER_HPP

#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include <chrono>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ns_lidar_cluster {

class LidarCluster {

 public:
  // Constructor
  LidarCluster(ros::NodeHandle &nh);

  // Getters
  sensor_msgs::PointCloud getLidarCluster();

  bool is_ok() const;

  // Setters
  void setRawLidar(const sensor_msgs::PointCloud2 &msg);

  void runAlgorithm();

 private:
  ros::NodeHandle &nh_;

  void loadParameters();

  bool getRawLidar_, is_ok_flag_;

  sensor_msgs::PointCloud cluster_;

  sensor_msgs::PointCloud2 raw_pc2_;

  sensor_msgs::PointCloud2 filter_ground_, filter_cones_;

  pcl::PointCloud<pcl::PointXYZI> raw_pc_;

  void preprocessing(pcl::PointCloud<pcl::PointXYZI> &raw,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones);
  void ClusterProcessing(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold);
};
} // namespace ns_lidar_cluster

#endif // LIDAR_CLUSTER_HPP
