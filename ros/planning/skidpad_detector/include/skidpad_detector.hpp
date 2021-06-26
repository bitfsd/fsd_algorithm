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

#ifndef SKIDPAD_DETECTOR_HPP
#define SKIDPAD_DETECTOR_HPP

#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "pcl/io/pcd_io.h"
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include "fstream"
#include "cmath"

namespace ns_skidpad_detector {

class SkidpadDetector {

 public:
  // Constructor
  SkidpadDetector(ros::NodeHandle& nh);

	// Getters
  std_msgs::Float64MultiArray getTransMatrix();

	// Setters
  void setclusterFiltered(sensor_msgs::PointCloud msg);


  void runAlgorithm();

private:

	ros::NodeHandle& nh_;

  std::string path_pcd_, path_x_, path_y_;
  double start_length_, lidar2imu_, threshold_;

  bool getClusterFlag, matchFlag;
  sensor_msgs::PointCloud cluster, skidpad_map;
  nav_msgs::Path trans_path, standard_path;
  std_msgs::Float64MultiArray trans_matrix_in_1D;
  Eigen::Matrix4f transformation;

  void loadParameters();
  void loadFiles();
};
}

#endif //SKIDPAD_DETECTOR_HPP
