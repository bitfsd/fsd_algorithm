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
#include "template.hpp"
#include <sstream>

namespace ns_template {
// Constructor
Template::Template(ros::NodeHandle &nh) : nh_(nh) {
  
};

// Getters
geometry_msgs::Pose2D Template::getPose() { return pose_current; }

// Setters
void Template::setPose(geometry_msgs::Pose2D msg) {
    pose_current = msg;
}

void Template::runAlgorithm() {

}

}
