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
#include "line_detector_handle.hpp"

namespace ns_line_detector {

// Constructor
LineDetectorHandle::LineDetectorHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    line_detector_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int LineDetectorHandle::getNodeRate() const { return node_rate_; }

// Methods
void LineDetectorHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("lidar_cluster_topic_name",
                                      lidar_cluster_topic_name_,
                                      "/lidar_cluster")) {
    ROS_WARN_STREAM("Did not load lidar_cluster_topic_name. Standard value is: " << lidar_cluster_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("line_path_topic_name",
                                      line_path_topic_name_,
                                      "/global_path")) {
    ROS_WARN_STREAM("Did not load line_path_topic_name. Standard value is: " << line_path_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 50)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void LineDetectorHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  lidarClusterSubscriber_ = nodeHandle_.subscribe(lidar_cluster_topic_name_, 1, &LineDetectorHandle::lidarClusterCallback, this);
}

void LineDetectorHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  linePathPublisher_ = nodeHandle_.advertise<nav_msgs::Path>(line_path_topic_name_, 1);
}

void LineDetectorHandle::run() {
	line_detector_.runAlgorithm();
  sendMsg();
}

void LineDetectorHandle::sendMsg() {
  linePathPublisher_.publish(line_detector_.getlinePath());
}

void LineDetectorHandle::lidarClusterCallback(const sensor_msgs::PointCloud &msg) {
  line_detector_.setlidarCluster(msg);
}

}