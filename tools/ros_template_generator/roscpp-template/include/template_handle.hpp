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

#ifndef TEMPLATE_HANDLE_HPP
#define TEMPLATE_HANDLE_HPP

#include "template.hpp"

namespace ns_template {

class TemplateHandle {

 public:
  // Constructor
  TemplateHandle(ros::NodeHandle &nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();
//  void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber inputPoseSubscriber_;
  ros::Publisher ouputPosePublisher_;

  void inputPoseCallback(const geometry_msgs::Pose2D &msg);

  std::string input_pose_topic_name_;
  std::string output_pose_topic_name_;

  int node_rate_;

  Template template_;

};
}

#endif //TEMPLATE_HANDLE_HPP
