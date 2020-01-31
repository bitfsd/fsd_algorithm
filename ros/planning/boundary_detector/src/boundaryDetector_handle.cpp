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
#include "boundaryDetector_handle.hpp"

namespace ns_boundaryDetector {

// Constructor
BoundaryDetectorHandle::BoundaryDetectorHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    boundaryDetector_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int BoundaryDetectorHandle::getNodeRate() const { return node_rate_; }

// Methods
void BoundaryDetectorHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("local_map_topic_name",
                                      local_map_topic_name_,
                                      "/map")) {
    ROS_WARN_STREAM("Did not load local_map_topic_name. Standard value is: " << local_map_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("boundary_detections_topic_name",
                                      boundary_detections_topic_name_,
                                      "/planning/boundary_detections")) {
    ROS_WARN_STREAM("Did not load boundary_detections_topic_name. Standard value is: " << boundary_detections_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("visual_triangles_topic_name",
                                      visual_triangles_topic_name_,
                                      "/visualization/visual_triangles")) {
    ROS_WARN_STREAM("Did not load visual_triangles_topic_name. Standard value is: " << visual_triangles_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("visual_boundary_topic_name",
                                      visual_boundary_topic_name_,
                                      "/visualization/visual_boundary")) {
    ROS_WARN_STREAM("Did not load visual_boundary_topic_name. Standard value is: " << visual_boundary_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("visual_tree_topic_name",
                                      visual_tree_topic_name_,
                                      "/visualization/visual_tree")) {
    ROS_WARN_STREAM("Did not load visual_tree_topic_name. Standard value is: " << visual_tree_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("visual_path_topic_name",
                                      visual_path_topic_name_,
                                      "/visualization/visual_path")) {
    ROS_WARN_STREAM("Did not load visual_path_topic_name. Standard value is: " << visual_path_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 50)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void BoundaryDetectorHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  localMapSubscriber = nodeHandle_.subscribe(local_map_topic_name_, 1, &BoundaryDetectorHandle::localMapCallback, this);
}

void BoundaryDetectorHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  boundaryDetectionsPublisher = nodeHandle_.advertise<fsd_common_msgs::Map>(boundary_detections_topic_name_, 1);
  visualTrianglesPublisher = nodeHandle_.advertise<visualization_msgs::Marker>(visual_triangles_topic_name_, 1);
  visualBoundaryPublisher = nodeHandle_.advertise<visualization_msgs::MarkerArray>(visual_boundary_topic_name_, 1);
  visualTreePublisher = nodeHandle_.advertise<visualization_msgs::MarkerArray>(visual_tree_topic_name_, 1);
  visualPathPublisher = nodeHandle_.advertise<visualization_msgs::Marker>(visual_path_topic_name_, 1);
}

void BoundaryDetectorHandle::run() {
	boundaryDetector_.runAlgorithm();
  sendMsg();
}

void BoundaryDetectorHandle::sendMsg() {
  boundaryDetectionsPublisher.publish(boundaryDetector_.getboundaryDetections());
  visualTrianglesPublisher.publish(boundaryDetector_.getVisualTriangles());
  visualBoundaryPublisher.publish(boundaryDetector_.getVisualBoundary());
  visualTreePublisher.publish(boundaryDetector_.getVisualTree());
  visualPathPublisher.publish(boundaryDetector_.getVisualPath());
}

void BoundaryDetectorHandle::localMapCallback(const fsd_common_msgs::Map &msg) {
  boundaryDetector_.setLocalMap(msg);
}
}