#pragma once
#include "ros/ros.h"

struct Weight {
  double px;
  double py;
  double pyaw;
  double cte;
  double epsi;
  double v;
  double steer;
  double throttle;
  double steer_rate;
  double throttle_rate;
};

struct Param {
  // MPC Solver N, dt
  int N;
  double dt;
  // Weight
  Weight weight;

  bool simulation;

  double car_length;

  double initial_velocity;

  // Trajectory Interval
  double interval;

  double forward_distance;

  double circle_radius;

  double look_ahead;

  double max_lat_acc;

  // For Pure Pursuit
  double desire_vel;

  // Get Parameters from yaml
  void getParams(ros::NodeHandle &nh, const std::string &mission, const std::string &controller) {
    car_length = nh.param("car_length", 1.88);
    N = nh.param("N", 40);
    dt = nh.param("dt", 0.04);
    weight.px = nh.param("weight/px", 3);
    weight.py = nh.param("weight/py", 10);
    weight.pyaw = nh.param("weight/pyaw", 8);
    weight.cte = nh.param("weight/cte", 1);
    weight.epsi = nh.param("weight/epsi", 4);
    weight.v = nh.param("weight/v", 0.4);
    weight.steer = nh.param("weight/steer", 10);
    weight.throttle = nh.param("weight/throttle", 10);
    weight.steer_rate = nh.param("weight/steer_rate", 2000);
    weight.throttle_rate = nh.param("weight/throttle_rate", 10);
    simulation = nh.param("simulation", true);
    interval = nh.param("interval", 0.08);
    forward_distance = nh.param("forward_distance", 15.0);
    circle_radius = nh.param("circle_radius", 9.125);
    look_ahead = nh.param("look_ahead", 10);
    max_lat_acc = nh.param("max_lat_acc", 3.0);
    initial_velocity = nh.param("initial_velocity", 2.0);

    if (controller == "mpc")
      desire_vel = nh.param("weight/desire_vel", 15);
    else
      desire_vel = nh.param("desire_vel", 3);
  }
};

extern Param param_;
