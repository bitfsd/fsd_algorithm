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

#include "lidar_cluster.hpp"
#include <ros/ros.h>
#include <sstream>
#include <utility>

namespace ns_lidar_cluster {
// Constructor
LidarCluster::LidarCluster(ros::NodeHandle &nh) : nh_(nh) { loadParameters(); };

// load Param
void LidarCluster::loadParameters() {
  getRawLidar_ = false;
  is_ok_flag_ = false;
}

// Getters
sensor_msgs::PointCloud LidarCluster::getLidarCluster() { return cluster_; }

bool LidarCluster::is_ok() const { return is_ok_flag_; }

// Setters
void LidarCluster::setRawLidar(const sensor_msgs::PointCloud2 &msg) {
  raw_pc2_ = msg;
  getRawLidar_ = true;
}

void LidarCluster::runAlgorithm() {
  if (raw_pc2_.fields.empty() || !getRawLidar_) {
    return;
  }
  getRawLidar_ = false;

  pcl::fromROSMsg(raw_pc2_, raw_pc_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cones(
      new pcl::PointCloud<pcl::PointXYZI>);

  // segment the groud point cloud and get the raw_flatten
  preprocessing(raw_pc_, cloud_ground, cloud_cones);

  // use cluster and multi filter to get the cones position
  ClusterProcessing(cloud_cones, 0.5);

  cluster_.header.frame_id = "/base_link";
  cluster_.header.stamp = raw_pc2_.header.stamp;
  is_ok_flag_ = true;
}

void LidarCluster::preprocessing(
    pcl::PointCloud<pcl::PointXYZI> &raw,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones) {

  pcl::PointCloud<pcl::PointXYZI> filtered;

  for (auto &iter : raw.points) {
    if (std::hypot(iter.x, iter.y) < sqrt(2) || iter.z > 0.7 ||
        iter.x < 0 || (std::hypot(iter.x, iter.y) > 7 && iter.z < 0.03))
      continue;
    filtered.points.push_back(iter);
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // set Distance Threshold
  seg.setDistanceThreshold(0.07);
  seg.setInputCloud(filtered.makeShared());
  seg.segment(*inliers, *coefficients);

  /* Debug
    for (auto iter : coefficients->values) {
      std::cout << iter << " ";
    }
    std::cout << "\n-------------\n";
   */

  // extract ground
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(raw.makeShared());
  extract.setIndices(inliers);
  extract.filter(*cloud_ground);

  // extract cone
  extract.setNegative(true);
  extract.filter(*cloud_cones);

  pcl::toROSMsg(*cloud_ground, filter_ground_);
  pcl::toROSMsg(*cloud_cones, filter_cones_);
}

void LidarCluster::ClusterProcessing(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold) {

  cluster_.points.clear();

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(threshold);
  ec.setMinClusterSize(2);
  ec.setMaxClusterSize(200);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (const auto &iter : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cone(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto it : iter.indices) {
      cone->points.push_back(cloud->points[it]);
    }
    cone->width = cone->points.size();
    cone->height = 1;
    cone->is_dense = true;

    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::compute3DCentroid(*cone, centroid);
    pcl::getMinMax3D(*cone, min, max);

    float bound_x = std::fabs(max[0] - min[0]);
    float bound_y = std::fabs(max[1] - min[1]);
    float bound_z = std::fabs(max[2] - min[2]);

    // filter based on the shape of cones
    if (bound_x < 0.5 && bound_y < 0.5 && bound_z < 0.4 && centroid[2] < 0.4) {
      geometry_msgs::Point32 tmp;
      tmp.x = centroid[0];
      tmp.y = centroid[1];
      tmp.z = centroid[2];
      cluster_.points.push_back(tmp);
    }
  }
}

} // namespace ns_lidar_cluster
