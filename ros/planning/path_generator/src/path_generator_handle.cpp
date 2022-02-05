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
#include "path_generator_handle.hpp"

namespace ns_path_generator {

// Constructor
    PathGeneratorHandle::PathGeneratorHandle(ros::NodeHandle &nodeHandle) :
            nodeHandle_(nodeHandle), path_generator_(nodeHandle) {
        ROS_INFO("Constructing Handle");
        loadParameters();
        subscribeToTopics();
        publishToTopics();
    }

// Getters
    int PathGeneratorHandle::getNodeRate() const { return node_rate_; }

// Methods
    void PathGeneratorHandle::loadParameters() {
        ROS_INFO("loading handle parameters");
        if (!nodeHandle_.param<std::string>("car_state_topic_name",
                                            car_state_topic_name_,
                                            "/estimation/slam/state")) {
            ROS_WARN_STREAM("Did not load car_state_topic_name. Standard value is: " << car_state_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("transform_matrix_topic_name",
                                            transform_matrix_topic_name_, "/transform_matrix")) {
            ROS_WARN_STREAM(
                    "Did not load transform_matrix_topic_name. Standard value is: "
                            << transform_matrix_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("end_point_topic_name",
                                            end_point_topic_name_, "/planning/end_point")) {
            ROS_WARN_STREAM(
                    "Did not load end_point_topic_name. Standard value is: "
                            << end_point_topic_name_);
        }
        if (!nodeHandle_.param("node_rate", node_rate_, 10)) {
            ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
        }
        if (!nodeHandle_.param<std::string>("map_topic_name",
                                            map_topic_name_,
                                            "/map")) {
            ROS_WARN_STREAM("Did not load map_topic_name. Standard value is : " << map_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("ref_path_topic_name",
                                            ref_path_topic_name_,
                                            "/visual/ref_path")) {
            ROS_WARN_STREAM("Did not load ref_path_topic_name_. Standard value is: " << ref_path_topic_name_);
        }
    if (!nodeHandle_.param<std::string>("path_generate_topic_name",
                                        path_generate_topic_name_,
                                        "/planning/ref_path")) {
        ROS_WARN_STREAM("Did not load path_generate_topic_name. Standard value is: "<< path_generate_topic_name_);
    }
    }

    void PathGeneratorHandle::subscribeToTopics() {
        ROS_INFO("subscribe to topics");
        //from line_detector
        endPointSubscriber_ = nodeHandle_.subscribe(
                end_point_topic_name_, 1, &PathGeneratorHandle::endPointCallback, this);
        //from skidpad_detector
        transMatSubscriber_ = nodeHandle_.subscribe(
                transform_matrix_topic_name_, 1, &PathGeneratorHandle::transMatCallback, this);
        //for trackdrive
        localMapSubscriber_ =
                nodeHandle_.subscribe(map_topic_name_, 10, &PathGeneratorHandle::localMapCallback, this);
        //others
        carStateSubscriber_ =
                nodeHandle_.subscribe(car_state_topic_name_, 10, &PathGeneratorHandle::carStateCallback, this);
    }

    void PathGeneratorHandle::publishToTopics() {
        ROS_INFO("publish to topics");
        refPathVisualPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(ref_path_topic_name_, 10);
        refPathPublisher_ = nodeHandle_.advertise<fsd_common_msgs::Trajectory>(path_generate_topic_name_, 10);
    }

    void PathGeneratorHandle::run() {
        path_generator_.runAlgorithm();
        sendMsg();
    }

    void PathGeneratorHandle::sendMsg() {
        refPathVisualPublisher_.publish(path_generator_.getRefPath());
        refPathPublisher_.publish(path_generator_.getRefTrajectory());
    }

    void PathGeneratorHandle::endPointCallback(const geometry_msgs::Point &msg) {
        path_generator_.setEndPoint(msg);
    }

    void PathGeneratorHandle::localMapCallback(const fsd_common_msgs::Map &msg) {
        path_generator_.setLocalMap(msg);
    }

    void PathGeneratorHandle::carStateCallback(const fsd_common_msgs::CarState &msg) {
        path_generator_.setCarState(msg);
    }

    void PathGeneratorHandle::transMatCallback(const std_msgs::Float64MultiArray &msg) {
        path_generator_.setTransMat(msg);
    }
}