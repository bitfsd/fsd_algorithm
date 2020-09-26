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

#include <ros/ros.h>
#include "lidar_cluster_handle.hpp"

namespace ns_lidar_cluster {

// Constructor
LidarClusterHandle::LidarClusterHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    lidar_cluster_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int LidarClusterHandle::getNodeRate() const { return node_rate_; }

// Methods
void LidarClusterHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("raw_lidar_topic_name",
                                      raw_lidar_topic_name_,
                                      "/velodyne_points_init")) {
    ROS_WARN_STREAM("Did not load raw_lidar_topic_name. Standard value is: " << raw_lidar_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("lidar_cluster_topic_name",
                                      lidar_cluster_topic_name_,
                                      "/perception/lidar_cluster")) {
    ROS_WARN_STREAM("Did not load lidar_cluster_topic_name. Standard value is: " << lidar_cluster_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 10)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void LidarClusterHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  rawLidarSubscriber_ = nodeHandle_.subscribe(raw_lidar_topic_name_, 1, &LidarClusterHandle::rawLidarCallback, this);
}

void LidarClusterHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  lidarClusterPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud>(lidar_cluster_topic_name_, 1);
}

void LidarClusterHandle::run() {
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  lidar_cluster_.runAlgorithm();
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
  std::cout<<"time:"<<time_round<<std::endl;
  sendMsg();
}

void LidarClusterHandle::sendMsg() {
  if(!lidar_cluster_.is_ok())
    return;
  lidarClusterPublisher_.publish(lidar_cluster_.getLidarCluster());
}

void LidarClusterHandle::rawLidarCallback(const sensor_msgs::PointCloud2 &msg) {
  lidar_cluster_.setRawLidar(msg);
}
}