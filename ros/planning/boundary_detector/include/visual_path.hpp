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

#ifndef VISUAL_PATH_HPP
#define VISUAL_PATH_HPP

#include "type.hpp"
#include <string>
#include "opencv2/imgproc.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "fsd_common_msgs/Map.h"

namespace FSD {
    void visual(cv::Subdiv2D coneSet, SearchTree Path, 
        fsd_common_msgs::Map boundaryDetections, 
        std::vector<PathPoint> BestPath,
        visualization_msgs::Marker &visualTriangles, 
        visualization_msgs::MarkerArray &visualTree, 
        visualization_msgs::MarkerArray &visualBoundary,
        visualization_msgs::Marker &visualPath
        );
};

#endif