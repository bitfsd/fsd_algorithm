/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2021:
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

#ifndef CONTROL_HANDLE_HPP
#define CONTROL_HANDLE_HPP

#include "control.hpp"
#include "ros/ros.h"
#include <iostream>
#include <string>

namespace ns_control {

class ControlHandle {

 public:
  ControlHandle(ros::NodeHandle &nodeHandle);

  int getNodeRate() const;

  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();

 private:
  void carStateCallback(const fsd_common_msgs::CarState &msg);
  void refTrajCallback(const fsd_common_msgs::Trajectory &msg);


 private:
  ros::NodeHandle nodeHandle_;

  ros::Subscriber carStateSubscriber_;
  ros::Subscriber refTrajectorySubscriber_;

  ros::Publisher cmdPublisher_;
  ros::Publisher prePathPublisher_;

  std::string car_state_topic_name_;
  std::string map_topic_name_;
  std::string ctrl_cmd_topic_name_;
  std::string predict_path_topic_name_;
  std::string ref_path_topic_name_;

  int node_rate_{};

  Control control_;
};
}

#endif //CONTROL_HANDLE_HPP
