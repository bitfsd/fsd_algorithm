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
#include "Utils/param.h"
#include "control.hpp"
#include <sstream>



namespace ns_control {

    Param control_param_;

    Control::Control(ros::NodeHandle &nh) : nh_(nh) {

        controller_ = nh_.param<std::string>("controller", "pure_pursuit");
        control_param_.getParams(nh_, controller_);

        if (controller_ == "pure_pursuit") { solver_ = &pure_pursuit_solver_; }
        else if (controller_ == "mpc") { solver_ = &mpc_solver_; }
        else {
            ROS_ERROR("Undefined Solver name !");
        }
    }

    void Control::setCarState(const fsd_common_msgs::CarState &msgs) { car_state_ = msgs; }

    void Control::setTrack(const Trajectory &msgs) { refline_ = msgs; }

    fsd_common_msgs::ControlCommand Control::getCmd() { return cmd_; }

    visualization_msgs::MarkerArray Control::getPrePath() { return PrePath_; }

    bool Control::Check() {
        if (refline_.empty()) {
            ROS_DEBUG_STREAM("Successfully passing check");
            return false;
        }
        return true;
    }

    void Control::runAlgorithm() {
        if (!Check()) {
            ROS_WARN_STREAM("Check Error");
            return;
        }

        solver_->setState(VehicleState(car_state_, cmd_));
        solver_->setTrajectory(refline_);
        solver_->solve();

        cmd_ = solver_->getCmd();

        std::vector<float> color_ref = {1, 0, 0};
        std::vector<float> color_pre = {0, 1, 0};
        std::vector<float> color_init = {0, 0, 1};

        if (controller_ == "mpc")
            visual_trajectory(solver_->getTrajectory(), PrePath_, "/base_link",
                              color_pre, car_state_.header, true);

        std::cout << "steering: " << cmd_.steering_angle.data << std::endl;
        std::cout << "throttle: " << cmd_.throttle.data << std::endl;
    }
}
