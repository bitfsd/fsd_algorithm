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
#include "path_generator.hpp"
#include "Utils/visual.h"
#include <sstream>

namespace ns_path_generator {
// Constructor
    PathGenerator::PathGenerator(ros::NodeHandle &nh) : nh_(nh) {
        mission_ = nh_.param<std::string>("mission", "acceleration");
        param_.getParams(nh_, mission_);
        if (mission_ == "trackdrive") { track_ = &trackdrive_track_; }
        else if (mission_ == "acceleration") { track_ = &line_track_; }
        else if (mission_ == "skidpad") { track_ = &skidpad_track_; }
        else {
            ROS_ERROR("Undefined Mission name !");
        }
    };

// Getters
    visualization_msgs::MarkerArray PathGenerator::getRefPath() { return RefPath_; }

// Setters
    void PathGenerator::setCarState(const fsd_common_msgs::CarState &state) {
        car_state_ = state;
    }

    void PathGenerator::setLocalMap(const fsd_common_msgs::Map &map) {
        local_map_ = map;
        ROS_INFO_STREAM("set Local Map OK");
    }

    void PathGenerator::setEndPoint(const geometry_msgs::Point &point) {
        endPoint_ = point;
        ROS_INFO("setEndPoint OK: %.3f, %.3f", point.x, point.y);
    }

    void PathGenerator::setTransMat(const std_msgs::Float64MultiArray &array) {
        int element_counter = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                transMat_(i, j) = array.data[element_counter];
                element_counter++;
            }
        }
    }

// Methods
    void PathGenerator::runAlgorithm() {
        if (!Check()) {
            ROS_ERROR("Message Check ERROR!");
            return;
        }
        ROS_INFO_STREAM("set Track Start.");
        setTrack();
        ROS_INFO_STREAM("set Track Done.");
        track_->setState(VehicleState(car_state_, cmd_));
        track_->CalculateTraj(refline_);

        std::vector<float> color = {1, 0, 0};
        visual_trajectory(refline_, RefPath_, "/base_link", color,
                          car_state_.header, true);

    }

    bool PathGenerator::Check() {
        if (mission_ == "trackdrive") {
            if (local_map_.cone_red.empty() || local_map_.cone_blue.empty()) {
                ROS_WARN_STREAM("Local Map Empty !");
                return false;
            }
        }
        if (mission_ == "acceleration") {
            if (fabs(endPoint_.y) > 6 || endPoint_.x <= 40.0) {
                ROS_WARN("Acceleration end point is error, current end point is (%.3f,%.3f).", endPoint_.x,
                         endPoint_.y);
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

    void PathGenerator::setTrack() {
        // Just need to set once
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

        // Need to set every time
        if (mission_ == "trackdrive") {
            track_->setMap(local_map_);
            track_->genTraj();
        }
    }

    fsd_common_msgs::Trajectory PathGenerator::getRefTrajectory() {
        fsd_common_msgs::Trajectory refTraj_;
        refTraj_.trajectory.clear();
        for (auto point : refline_) {
            fsd_common_msgs::TrajectoryPoint ref_pt;
            ref_pt.pts.x = point.pts.x;
            ref_pt.pts.y = point.pts.y;
            ref_pt.acc.data = point.acc;
            ref_pt.curvature.data = point.curvature;
            ref_pt.r.data = point.r;
            ref_pt.yaw.data = point.yaw;
            ref_pt.velocity.data = point.velocity;

//            ROS_INFO_STREAM("point vel = "<<point.velocity);

            refTraj_.trajectory.push_back(ref_pt);
        }
        refTraj_.header = car_state_.header;
        return refTraj_;
    }
}
