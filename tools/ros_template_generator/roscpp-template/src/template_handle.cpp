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
#include "template_handle.hpp"

namespace ns_template {

// Constructor
TemplateHandle::TemplateHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    template_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int TemplateHandle::getNodeRate() const { return node_rate_; }

// Methods
void TemplateHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("input_pose_topic_name",
                                      input_pose_topic_name_,
                                      "/localization/pose_in")) {
    ROS_WARN_STREAM("Did not load input_pose_topic_name. Standard value is: " << input_pose_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("output_pose_topic_name",
                                      output_pose_topic_name_,
                                      "/localization/pose_out")) {
    ROS_WARN_STREAM("Did not load output_pose_topic_name. Standard value is: " << output_pose_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void TemplateHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  inputPoseSubscriber_ = nodeHandle_.subscribe(input_pose_topic_name_, 1, &TemplateHandle::inputPoseCallback, this);
}

void TemplateHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  ouputPosePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose2D>(output_pose_topic_name_, 1);
}

void TemplateHandle::run() {
	template_.runAlgorithm();
  sendMsg();
}

void TemplateHandle::sendMsg() {
  ouputPosePublisher_.publish(template_.getPose());
}

void TemplateHandle::inputPoseCallback(const geometry_msgs::Pose2D &msg) {
  template_.setPose(msg);
}
}