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

#include "Track/trackdrive_track.h"
#include "ros/ros.h"

namespace ns_path_generator {

bool Autox_Track::genTraj() {
  if (map_.cone_blue.empty() || map_.cone_red.empty())
    return false;

  fsd::Vec_f wx, wy;
  wx.push_back(0);
  wy.push_back(0);
  // Find Center Line
  {
    for (const auto &red : map_.cone_red) {

      const auto it_blue = std::min_element(
          map_.cone_blue.begin(), map_.cone_blue.end(),
          [&](const fsd_common_msgs::Cone &a, const fsd_common_msgs::Cone &b) {
            const double da = std::hypot(red.position.x - a.position.x,
                                         red.position.y - a.position.y);
            const double db = std::hypot(red.position.x - b.position.x,
                                         red.position.y - b.position.y);

            return da < db;
          });
      cv::Point2f tmp;
      tmp.x = static_cast<float>((red.position.x + it_blue->position.x) / 2.0);
      tmp.y = static_cast<float>((red.position.y + it_blue->position.y) / 2.0);
      wx.push_back(tmp.x);
      wy.push_back(tmp.y);
    }
  }

  fsd::Spline2D spline(wx, wy);

  TrajectoryPoint tmp_pt;
  trajectory_.clear();
  const double interval = param_.interval;

  for (float i = 0; i < spline.s.back(); i += interval) {
    std::array<float, 2> point_ = spline.calc_postion(i);
    tmp_pt.pts.x = point_[0];
    tmp_pt.pts.y = point_[1];
    tmp_pt.yaw = spline.calc_yaw(i);
    tmp_pt.r = 0;
    tmp_pt.curvature = spline.calc_curvature(i);
    trajectory_.push_back(tmp_pt);
  }
  double dis_min = 1; // judge only in 1 meters
  bool flag_1 = false;
  bool flag_2 = false;
  double index = 0;
  for (float i = trajectory_.size() - 1; i>=0; i-= interval) {
    double dis = std::hypot(trajectory_[trajectory_.size()-1].pts.x - trajectory_[i].pts.x, 
                            trajectory_[trajectory_.size()-1].pts.y - trajectory_[i].pts.y);
    if (flag_1 == false && dis <= dis_min)
    continue;
    if (flag_1 == false && dis > dis_min)
    flag_1 = true;
    if (flag_1 == true && dis <= dis_min)
    flag_2 = true;
    if (flag_2 == true && dis <= dis_min)
    dis_min = dis;
    if (flag_2 == true && dis > dis_min) {
      index = i;
      break;
    }
  }
  Trajectory tmp;
  for (float i=index; i<trajectory_.size(); i++) {
    tmp.push_back(trajectory_[i]);
  }
  trajectory_.clear();
  trajectory_ = tmp;
}

bool Autox_Track::CalculateTraj(Trajectory &refline) {

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
  for (int i = 0; i < trajectory_.size(); i++) {
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

    double delta_x = trajectory_[(index + index_min) % trajectory_.size()].pts.x - px;
    double delta_y = trajectory_[(index + index_min) % trajectory_.size()].pts.y - py;
    double delta_yaw = trajectory_[(index + index_min) % trajectory_.size()].yaw - psi;

    while ((delta_yaw) >= M_PI)
      delta_yaw -= M_PI * 2.0;
    while ((delta_yaw) <= -1.0 * M_PI)
      delta_yaw += M_PI * 2.0;

    double temp_x, temp_y;
    temp_x = delta_x * cos(psi) + delta_y * sin(psi);
    temp_y = delta_y * cos(psi) - delta_x * sin(psi);

    tmp.curvature = fabs(trajectory_[(index + index_min) % trajectory_.size()].curvature);
    tmp.pts = cv::Point2f(temp_x, temp_y);
    tmp.yaw = delta_yaw;
    tmp.velocity = std::min(sqrt(param_.max_lat_acc / tmp.curvature), desired_velocity);
    refline.push_back(tmp);
  }
}

}//namespace ns_path_generator
