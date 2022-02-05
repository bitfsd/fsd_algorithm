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
#include "control_handle.hpp"
#include <chrono>

namespace ns_control {

// Constructor
    ControlHandle::ControlHandle(ros::NodeHandle &nodeHandle) :
            nodeHandle_(nodeHandle),
            control_(nodeHandle) {
        ROS_INFO("Constructing Handle");
        loadParameters();
        subscribeToTopics();
        publishToTopics();
    }

// Getters
    int ControlHandle::getNodeRate() const { return node_rate_; }

// Methods
    void ControlHandle::loadParameters() {
        ROS_INFO("loading handle parameters");
        if (!nodeHandle_.param<std::string>("car_state_topic_name",
                                            car_state_topic_name_,
                                            "/estimation/slam/state")) {
            ROS_WARN_STREAM("Did not load car_state_topic_name. Standard value is: " << car_state_topic_name_);
        }
        if (!nodeHandle_.param("node_rate", node_rate_, 10)) {
            ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
        }
        if (!nodeHandle_.param<std::string>("map_topic_name",
                                            map_topic_name_,
                                            "/map")) {
            ROS_WARN_STREAM("Did not load map_topic_name. Standard value is : " << map_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("ctrl_cmd_topic_name",
                                            ctrl_cmd_topic_name_,
                                            "/control/pure_pursuit/control_command")) {
            ROS_WARN_STREAM("Did not load ctrl_cmd_topic_name. Standard value is : " << ctrl_cmd_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("ref_path_topic_name",
                                            ref_path_topic_name_,
                                            "/planning/ref_path")) {
            ROS_WARN_STREAM("Did not load ref_path_topic_name_. Standard value is: " << ref_path_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("predict_path_topic_name",
                                            predict_path_topic_name_,
                                            "/visual/pre_path")) {
            ROS_WARN_STREAM("Did not load visual_map_topic_name. Standard value is: " << predict_path_topic_name_);
        }
    }

    void ControlHandle::subscribeToTopics() {
        ROS_INFO("subscribe to topics");
        refTrajectorySubscriber_ =
                nodeHandle_.subscribe(ref_path_topic_name_, 1, &ControlHandle::refTrajCallback, this);

        carStateSubscriber_ =
                nodeHandle_.subscribe(car_state_topic_name_, 10, &ControlHandle::carStateCallback, this);
    }

    void ControlHandle::publishToTopics() {
        ROS_INFO("publish to topics");
        cmdPublisher_ = nodeHandle_.advertise<fsd_common_msgs::ControlCommand>(ctrl_cmd_topic_name_, 1);
        prePathPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(predict_path_topic_name_, 1);
    }

    void ControlHandle::carStateCallback(const fsd_common_msgs::CarState &msg) {
        control_.setCarState(msg);
    }

    void ControlHandle::refTrajCallback(const fsd_common_msgs::Trajectory &msg) {

        Trajectory ref_path;
        for (auto point : msg.trajectory) {
            TrajectoryPoint tpt;
            tpt.pts.x = point.pts.x;
            tpt.pts.y = point.pts.y;
            tpt.acc = point.acc.data;
            tpt.curvature = point.curvature.data;
            tpt.yaw = point.yaw.data;
            tpt.r = point.r.data;
            tpt.velocity = point.velocity.data;
            ref_path.push_back(tpt);
        }

        control_.setTrack(ref_path);

    }

    void ControlHandle::run() {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        control_.runAlgorithm();
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        std::cout << "time cost = " << time_round << ", frequency = " << 1 / time_round << std::endl;
        sendMsg();
    }

    void ControlHandle::sendMsg() {
        cmdPublisher_.publish(control_.getCmd());
        prePathPublisher_.publish(control_.getPrePath());
    }
}
