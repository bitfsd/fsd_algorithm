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

#ifndef SKIDPAD_DETECTOR_HANDLE_HPP
#define SKIDPAD_DETECTOR_HANDLE_HPP

#include "skidpad_detector.hpp"

namespace ns_skidpad_detector {

class SkidpadDetectorHandle {

 public:
  // Constructor
  SkidpadDetectorHandle(ros::NodeHandle &nodeHandle);

  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber clusterFilteredSubscriber_;
  ros::Publisher transformMatrixPublisher_;

  void clusterFilteredCallback(const sensor_msgs::PointCloud& msg);

  std::string cluster_filtered_topic_name_;
  std::string transform_matrix_topic_name_;

  int node_rate_;

  SkidpadDetector skidpad_detector_;

};
}

#endif //SKIDPAD_DETECTOR_HANDLE_HPP
