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

#ifndef BOUNDARYDETECTOR_HANDLE_HPP
#define BOUNDARYDETECTOR_HANDLE_HPP

#include "boundaryDetector.hpp"

namespace ns_boundaryDetector {

class BoundaryDetectorHandle {

 public:
  // Constructor
  BoundaryDetectorHandle(ros::NodeHandle &nodeHandle);

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
  ros::Subscriber localMapSubscriber;

  ros::Publisher boundaryDetectionsPublisher;
  ros::Publisher visualTrianglesPublisher;
  ros::Publisher visualBoundaryPublisher;
  ros::Publisher visualTreePublisher;
  ros::Publisher visualPathPublisher;

  void localMapCallback(const fsd_common_msgs::Map &msg);

  std::string local_map_topic_name_;
  std::string boundary_detections_topic_name_;
  std::string visual_triangles_topic_name_;
  std::string visual_boundary_topic_name_;
  std::string visual_tree_topic_name_;
  std::string visual_path_topic_name_;

  int node_rate_;

  BoundaryDetector boundaryDetector_;

};
}

#endif //BOUNDARYDETECTOR_HANDLE_HPP
