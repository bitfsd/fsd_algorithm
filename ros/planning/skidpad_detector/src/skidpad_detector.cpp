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
#include "skidpad_detector.hpp"
#include <sstream>

namespace ns_skidpad_detector {
// Constructor
SkidpadDetector::SkidpadDetector(ros::NodeHandle &nh) : nh_(nh) {
  getClusterFlag = false;
  matchFlag = false;
  loadParameters();
  loadFiles();
};

// Getters
std_msgs::Float64MultiArray SkidpadDetector::getTransMatrix() {
  trans_matrix_in_1D.data.clear();
  for (int i = 0; i < transformation.rows(); i++)
    for (int j = 0; j < transformation.cols(); j++) {
      trans_matrix_in_1D.data.push_back(transformation(i, j));
    }
  return trans_matrix_in_1D;
}

// Setters
void SkidpadDetector::setclusterFiltered(sensor_msgs::PointCloud msg) {
  getClusterFlag = true;
  cluster = msg;
}
void SkidpadDetector::loadParameters() {
  ROS_INFO("loading parameters");
  if (!nh_.param<std::string>("path/skidpad_map",
                                      path_pcd_,
                                      "skidpad_detector/skidpad.pcd")) {
    ROS_WARN_STREAM("Did not load path/skidpad_map. Standard value is: " << path_pcd_);
  }
  
 	if (!nh_.param<std::string>("path/path_x",
                                      path_x_,
                                      "skidpad_detector/path_x.txt")) {
    ROS_WARN_STREAM("Did not load path/path_x. Standard value is: " << path_x_);
  }

  if (!nh_.param<std::string>("path/path_y",
                                      path_y_,
                                      "skidpad_detector/path_y.txt")) {
    ROS_WARN_STREAM("Did not load path/path_y. Standard value is: " << path_y_);
  }

	if (!nh_.param("length/start", start_length_, 15.0)) {
    ROS_WARN_STREAM("Did not load start_length. Standard value is: " << start_length_);
  }

	if (!nh_.param("length/lidar2imu", lidar2imu_, 1.87)) {
    ROS_WARN_STREAM("Did not load lidar2imu. Standard value is: " << lidar2imu_);
  }

  if (!nh_.param("length/threshold", threshold_, 0.7)) {
    ROS_WARN_STREAM("Did not load length/threshold. Standard value is: " << threshold_);
  }
}

void SkidpadDetector::loadFiles() {
  /* load pcd skidpad map */
  pcl::PointCloud<pcl::PointXYZ> source_cloud;
	geometry_msgs::Point32 tmp_cloud;
	pcl::io::loadPCDFile (path_pcd_,source_cloud);
  ROS_INFO_STREAM("load files");
  // The front is the x-axis, and the left is the y-axis
	for(int i = 0; i < source_cloud.points.size(); i++)
	{
		tmp_cloud.x = source_cloud.points[i].y + start_length_ + lidar2imu_;
		tmp_cloud.y = -source_cloud.points[i].x;
		skidpad_map.points.push_back(tmp_cloud);
	}

  /* load skidpad path */
  std::ifstream infile_x,infile_y;
	infile_x.open(path_x_);
	infile_y.open(path_y_);
	double path_x,path_y;

	while(!infile_x.eof() && !infile_y.eof())
	{
		infile_x>>path_x;
		infile_y>>path_y;
		geometry_msgs::PoseStamped temp;
		temp.pose.position.x = path_x + lidar2imu_;
		temp.pose.position.y = path_y;
		standard_path.poses.push_back(temp);
	}
	infile_x.close();
	infile_y.close();
}

void SkidpadDetector::runAlgorithm() {
  if(!getClusterFlag || matchFlag)
    return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> Final;
  pcl::PointXYZ in_temp;
	pcl::PointXYZ out_temp;

  // find match points between skidpad map and cluster, match distance < threshold_
  for(int i = 0; i < skidpad_map.points.size(); i++)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    int index = -1;
    for(int j = 0; j < cluster.points.size(); j++)
    {
      double dist = std::hypot(skidpad_map.points[i].x - cluster.points[j].x, skidpad_map.points[i].y - cluster.points[j].y);
      if(min_dist > dist) {
        min_dist = dist;
        index = j;
      }
    }
    if(min_dist < threshold_) {
      in_temp.x = skidpad_map.points[i].x;
      in_temp.y = skidpad_map.points[i].y;
      in_temp.z = 0;
      cloud_in->points.push_back(in_temp);

      out_temp.x = cluster.points[index].x;
		  out_temp.y = cluster.points[index].y;
		  out_temp.z = 0;
      cloud_out->points.push_back(out_temp);
    }
  }

  // icp match
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  icp.setMaxCorrespondenceDistance(1);  
	icp.setTransformationEpsilon(1e-10); 
	icp.setEuclideanFitnessEpsilon(0.001); 
	icp.setMaximumIterations(100);
  icp.align(Final);

  ROS_INFO_STREAM("icp finish");

  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	transformation = icp.getFinalTransformation();
	std::cout <<transformation<<std::endl;
	std::cout <<"------------------------------------------------\n";

  matchFlag = true;
}

}
