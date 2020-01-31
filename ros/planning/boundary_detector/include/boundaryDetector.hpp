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

#ifndef BOUNDARYDETECTOR_HPP
#define BOUNDARYDETECTOR_HPP

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include "fsd_common_msgs/Cone.h"
#include "fsd_common_msgs/Map.h"
#include "opencv2/imgproc.hpp"
#include "eigen3/Eigen/Core"
#include "std_msgs/String.h"
#include "type.hpp"
#include "visual_path.hpp"

namespace ns_boundaryDetector {

using ConePos = FSD::ConePos;
using PathPoint = FSD::PathPoint;
using SearchTree = FSD::SearchTree;
using Cost_index = FSD::Cost_index;
using Cost_n = FSD::Cost_n;
using Cost_p = FSD::Cost_p;

class BoundaryDetector {

 public:
  // Constructor
  BoundaryDetector(ros::NodeHandle& nh);

	// Getters
  fsd_common_msgs::Map getboundaryDetections();
  visualization_msgs::Marker getVisualTriangles();
  visualization_msgs::MarkerArray getVisualBoundary();
  visualization_msgs::MarkerArray getVisualTree();
  visualization_msgs::Marker getVisualPath();

	// Setters
  void setLocalMap(fsd_common_msgs::Map msg);


  void runAlgorithm();

private:

	ros::NodeHandle& nh_;
	
	fsd_common_msgs::Map map_current;
  fsd_common_msgs::Map map;

  visualization_msgs::Marker visualTriangles;
  visualization_msgs::MarkerArray visualTree;
  visualization_msgs::MarkerArray visualBoundary;
  visualization_msgs::Marker visualPath;
  fsd_common_msgs::Map boundaryDetections;


  double max_beam_cost_;
  int max_iter_num_, max_search_num_;
  Cost_n beam_weight_;
  Cost_p path_weight_;

  void loadParameters();
  bool filter(fsd_common_msgs::Map &init_map);
  void initSet(fsd_common_msgs::Map map, cv::Subdiv2D &coneSet, std::map<ConePos, char> &colorMap);
  void getMidPoint(cv::Subdiv2D coneSet, std::map<ConePos, char> colorMap, std::map<int, PathPoint> &MidSet);
  void searchPath(std::map<int, PathPoint> MidSet, SearchTree &Path);
  void selectBestPath(SearchTree Path, std::vector<PathPoint> &BestPath);
  void generateBoundary(std::vector<PathPoint> BestPath, fsd_common_msgs::Map &Boundary);
};
}

#endif //BOUNDARYDETECTOR_HPP
