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

#include "Track/skidpad_track.h"
#include "ros/ros.h"

Param param_;

namespace ns_path_generator {

bool Skidpad_Track::genTraj() {

  // The front side to the gravity point
  const double car_length = param_.simulation ? 0 : param_.car_length;

  double interval = param_.interval;
  double forward_distance = param_.forward_distance;
  double circle_radius = param_.circle_radius;
  double right_circle_x = forward_distance + car_length;
  double right_circle_y = -circle_radius;
  double left_circle_x = right_circle_x;
  double left_circle_y = circle_radius;

  TrajectoryPoint tmp_pt;
  trajectory_.clear();

  //line need discrete
  for (double i = -2.0; i < (car_length + forward_distance); i += interval) {
    tmp_pt.pts.x = i;
    tmp_pt.pts.y = 0;
    tmp_pt.yaw = 0;
    trajectory_.push_back(tmp_pt);
  }

  //right_circle
  for (double i = 0; i < 4 * M_PI; i += interval / circle_radius) {
    tmp_pt.pts.x = circle_radius * std::cos(90 * M_PI / 180 - i) + right_circle_x;
    tmp_pt.pts.y = circle_radius * std::sin(90 * M_PI / 180 - i) + right_circle_y;
    tmp_pt.yaw = -i;
    trajectory_.push_back(tmp_pt);
  }

  //left_circle
  for (double i = 0; i < 4 * M_PI; i += interval / circle_radius) {
    tmp_pt.pts.x = circle_radius * std::cos(-90 * M_PI / 180 + i) + left_circle_x;
    tmp_pt.pts.y = circle_radius * std::sin(-90 * M_PI / 180 + i) + left_circle_y;
    tmp_pt.yaw = i;
    trajectory_.push_back(tmp_pt);
  }

  //line again
  for (float i = car_length + forward_distance; i < (car_length + forward_distance) + 15; i += interval) {
    tmp_pt.pts.x = i;
    tmp_pt.pts.y = 0;
    tmp_pt.yaw = 0;
    trajectory_.push_back(tmp_pt);
  }

  // Transform the trajectory
  for (size_t i = 0; i < trajectory_.size(); i++) {
    double temp_x, temp_y;
    temp_x = trajectory_[i].pts.x;
    temp_y = trajectory_[i].pts.y;
    Eigen::Vector4f temp(temp_x, temp_y, 0, 1);
    Eigen::Vector4f result = transMat_ * temp;
    trajectory_[i].pts.y = result[1] / result[3];
    trajectory_[i].pts.x = result[0] / result[3];
  }

  return true;
}

bool Skidpad_Track::CalculateTraj(Trajectory &refline) {

  if (trajectory_.empty()) {
    ROS_WARN("Trajectory is empty !");
    return false;
  }
  refline.clear();
  TrajectoryPoint tmp_pt;

  double px = state_.x;
  double py = state_.y;
  double psi = state_.yaw;

  int count = 0;
  int index_min = -1;
  double min = 999;
  for (int i = now_state; i < trajectory_.size() && i < now_state + 400; i++) {
    double delta_x = trajectory_[i].pts.x - px;
    double delta_y = trajectory_[i].pts.y - py;
    double dist = std::hypot(delta_x, delta_y);
    if (dist < min) {
      min = dist;
      index_min = i;
    }
  }
  if (index_min < 0)
    return false;

  now_state = index_min;

  const double desired_velocity = param_.desire_vel;
  const int N = param_.N;
  const double dt = param_.dt;
  TrajectoryPoint tmp;
  double v = std::fmax(state_.v, param_.initial_velocity);

  double s_tmp = 0;

  for (int i = 0; i < N; i++) {
    if (i != 0)
      s_tmp += v * dt;

    int index = int(s_tmp / param_.interval);
    int next = (index + index_min) > trajectory_.size() ? trajectory_.size() - 1 : index + index_min;

    double delta_x = trajectory_[next].pts.x - px;
    double delta_y = trajectory_[next].pts.y - py;
    double delta_yaw = trajectory_[next].yaw - psi;

    while ((delta_yaw) >= M_PI)
      delta_yaw -= M_PI * 2.0;
    while ((delta_yaw) <= -1.0 * M_PI)
      delta_yaw += M_PI * 2.0;

    double temp_x, temp_y;
    temp_x = delta_x * cos(psi) + delta_y * sin(psi);
    temp_y = delta_y * cos(psi) - delta_x * sin(psi);

    tmp.curvature = fabs(trajectory_[next].curvature);
    tmp.pts = cv::Point2f(temp_x, temp_y);
    tmp.yaw = delta_yaw;
    tmp.velocity = std::min(sqrt(param_.max_lat_acc / tmp.curvature), desired_velocity);
    refline.push_back(tmp);
  }
}

}//namespace ns_path_generator
