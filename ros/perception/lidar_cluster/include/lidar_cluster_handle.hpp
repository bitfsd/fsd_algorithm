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

#ifndef LIDAR_CLUSTER_HANDLE_HPP
#define LIDAR_CLUSTER_HANDLE_HPP

#include "lidar_cluster.hpp"

namespace ns_lidar_cluster {

class LidarClusterHandle {

 public:
  // Constructor
  LidarClusterHandle(ros::NodeHandle &nodeHandle);

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
  ros::Subscriber rawLidarSubscriber_;

  ros::Publisher lidarClusterPublisher_;
  ros::Publisher filterGroundPublisher_;
  ros::Publisher filterConesPublisher_;
  ros::Publisher filterIntensityPublisher_;
  ros::Publisher coneReconstructPublisher_;

  void rawLidarCallback(const sensor_msgs::PointCloud2 &msg);

  std::string raw_lidar_topic_name_;
  std::string lidar_cluster_topic_name_;
  std::string filter_ground_topic_name_;
  std::string filter_cones_topic_name_;
  std::string filter_intensity_topic_name_;
  std::string cone_reconstruct_topic_name_;

  int node_rate_;

  LidarCluster lidar_cluster_;

};
}

#endif //LIDAR_CLUSTER_HANDLE_HPP
