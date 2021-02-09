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

#include <ros/ros.h>
#include "control.hpp"
#include <sstream>

namespace ns_control {
Control::Control(ros::NodeHandle &nh) : nh_(nh) {

  mission_ = nh_.param<std::string>("mission", "acceleration");
  controller_ = nh_.param<std::string>("controller", "pure_pursuit");
  param_.getParams(nh_, mission_, controller_);

  if (mission_ == "trackdrive") { track_ = &trackdrive_track_; }
  else if (mission_ == "acceleration") { track_ = &line_track_; }
  else if (mission_ == "skidpad") { track_ = &skidpad_track_; }
  else {
    ROS_ERROR("Undefined Mission name !");
  }

  if (controller_ == "pure_pursuit") { solver_ = &pure_pursuit_solver_; }
  else if (controller_ == "mpc") { solver_ = &mpc_solver_; }
  else {
    ROS_ERROR("Undefined Solver name !");
  }
}

void Control::setTransMat(const std_msgs::Float64MultiArray &msgs) {
  int element_counter = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      transMat_(i, j) = msgs.data[element_counter];
      element_counter++;
    }
  }
}

void Control::setEndPoint(const geometry_msgs::Point &msgs) { endPoint_ = msgs; }

void Control::setMap(const fsd_common_msgs::Map &msgs) { local_map_ = msgs; }

void Control::setCarState(const fsd_common_msgs::CarState &msgs) { car_state_ = msgs; }

fsd_common_msgs::ControlCommand Control::getCmd() { return cmd_; }

visualization_msgs::MarkerArray Control::getRefPath() { return RefPath_; }

visualization_msgs::MarkerArray Control::getPrePath() { return PrePath_; }

bool Control::Check() {
  if (mission_ == "trackdrive") {
    if (local_map_.cone_red.empty() || local_map_.cone_blue.empty()) {
      ROS_WARN_STREAM("Local Map Empty !");
      return false;
    }
  }
  if (mission_ == "acceleration") {
    if (fabs(endPoint_.y) > 6 || endPoint_.x <= 40.0) {
      ROS_WARN_STREAM("Acceleration end point is error !");
      return false;
    }
  }
  if (mission_ == "skidpad") {
    if (transMat_(3, 3) != 1) {
      ROS_WARN_STREAM("transMatrix is not correct !");
      return false;
    }
  }
  ROS_DEBUG_STREAM("Successfully passing check");
  return true;
}

void Control::setTrack() {
  // Just need set once
  if (!is_init) {
    if (mission_ == "acceleration") {
      track_->setEndPoint(endPoint_);
    }
    if (mission_ == "skidpad") {
      track_->setTransMat(transMat_);
    }
    track_->genTraj();
  }
  is_init = true;

  // Need set every time
  if (mission_ == "trackdrive") {
    track_->setMap(local_map_);
    track_->genTraj();
  }
}

void Control::runAlgorithm() {
  if (!Check()) {
    ROS_WARN_STREAM("Check Error");
    return;
  }

  setTrack();

  track_->setState(VehicleState(car_state_, cmd_));
  track_->CalculateTraj(refline_);

  solver_->setState(VehicleState(car_state_, cmd_));
  solver_->setTrajectory(refline_);
  solver_->solve();

  cmd_ = solver_->getCmd();

  std::vector<float> color_ref = {1, 0, 0};
  std::vector<float> color_pre = {0, 1, 0};
  std::vector<float> color_init = {0, 0, 1};

  visual_trajectory(refline_, RefPath_, "/base_link", color_ref,
                    car_state_.header, true);
  if (controller_ == "mpc")
  visual_trajectory(solver_->getTrajectory(), PrePath_, "/base_link",
                    color_pre, car_state_.header, true);

  std::cout << "steering: " << cmd_.steering_angle.data << std::endl;
  std::cout << "throttle: " << cmd_.throttle.data << std::endl;
}
}
