#pragma once

#include "ros/ros.h"
struct Param {

    int N;
    double dt;
    bool simulation;

    double car_length;

    double initial_velocity;

    // Trajectory Interval
    double interval;

    double forward_distance;

    double circle_radius;

    double max_lat_acc;

    // For Pure Pursuit
    double desire_vel;

    // Get Parameters from yaml
    void getParams(ros::NodeHandle &nh, const std::string &mission) {
        car_length = nh.param("car_length", 1.88);
        N = nh.param("N", 40);
        dt = nh.param("dt", 0.04);
        simulation = nh.param("simulation", true);
        interval = nh.param("interval", 0.08);
        forward_distance = nh.param("forward_distance", 15.0);
        circle_radius = nh.param("circle_radius", 9.125);
        max_lat_acc = nh.param("max_lat_acc", 3.0);
        initial_velocity = nh.param("initial_velocity", 2.0);
        desire_vel = nh.param("desire_vel", 3);
    }
};

extern Param param_;
