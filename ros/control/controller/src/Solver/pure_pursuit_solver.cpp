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

#include "Solver/pure_pursuit_solver.h"
#include "ros/ros.h"
#include <cmath>

namespace ns_control {
    
void Pure_Pursuit_Solver::solve() {
    ROS_INFO_STREAM("begin solve");
    if (trajectory_.empty()) {
        control_command_.throttle.data       = static_cast<float>(-1.0);//类型转换
        control_command_.steering_angle.data = 0.0;
        ROS_INFO_STREAM("trajectory empty");
        return;
    }

    double desire_vel = control_param_.desire_vel;

    const auto i_next = control_param_.look_ahead;
    geometry_msgs::Point32 next_point;

    { // Steering Control
        const double beta_est = control_command_.steering_angle.data * 0.5;
        next_point.x                            = trajectory_[i_next].pts.x*std::cos(state_.yaw) 
                                                    - trajectory_[i_next].pts.y*std::sin(state_.yaw);
        next_point.y                            = trajectory_[i_next].pts.x*std::sin(state_.yaw) 
                                                    + trajectory_[i_next].pts.y*std::cos(state_.yaw);
        const double eta                        = std::atan2(next_point.y, next_point.x)
                                                    - (state_.yaw + beta_est);
        const double length                     = std::hypot(next_point.y, next_point.x);

        control_command_.steering_angle.data    = static_cast<float>(1.5 * std::atan(2.0 / length * std::sin(eta)));
        std::cout<<"steering:  "<<control_command_.steering_angle.data<<std::endl;
    }
    { // Speed Controller
        const double vel = state_.v;
        control_command_.throttle.data = static_cast<float>(desire_vel - vel);
    }
}

}