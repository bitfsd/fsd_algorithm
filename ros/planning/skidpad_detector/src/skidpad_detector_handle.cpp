/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2020:
     - chentairan <tairanchen@bitfsd.cn>

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
#include "skidpad_detector_handle.hpp"

namespace ns_skidpad_detector {

// Constructor
SkidpadDetectorHandle::SkidpadDetectorHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    skidpad_detector_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int SkidpadDetectorHandle::getNodeRate() const { return node_rate_; }

// Methods
void SkidpadDetectorHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("cluster_filtered_topic_name",
                                      cluster_filtered_topic_name_,
                                      "/perception/lidar_cluster")) {
    ROS_WARN_STREAM("Did not load cluster_filtered_topic_name. Standard value is: " << cluster_filtered_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("transform_matrix_topic_name",
                                      transform_matrix_topic_name_,
                                      "/transform_matrix")) {
    ROS_WARN_STREAM("Did not load transform_matrix_topic_name. Standard value is: " << transform_matrix_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 50)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void SkidpadDetectorHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  clusterFilteredSubscriber_ = nodeHandle_.subscribe(cluster_filtered_topic_name_, 1, &SkidpadDetectorHandle::clusterFilteredCallback, this);
}

void SkidpadDetectorHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  transformMatrixPublisher_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>(transform_matrix_topic_name_, 1);
}

void SkidpadDetectorHandle::run() {
	skidpad_detector_.runAlgorithm();
  sendMsg();
}

void SkidpadDetectorHandle::sendMsg() {
  transformMatrixPublisher_.publish(skidpad_detector_.getTransMatrix());
}

void SkidpadDetectorHandle::clusterFilteredCallback(const sensor_msgs::PointCloud& msg) {
  skidpad_detector_.setclusterFiltered(msg);
}
}