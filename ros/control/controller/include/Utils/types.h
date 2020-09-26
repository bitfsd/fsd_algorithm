#ifndef TYPE_H
#define TYPE_H

#include <iostream>
#include "opencv2/opencv.hpp"

// ros package
#include "std_msgs/Float64MultiArray.h"

// custom messages
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"

// STL
#include <cmath>
#include <vector>

namespace ns_control {
struct VehicleState {
  double x;
  double y;
  double yaw;
  double v;
  double r;
  double a;
  double w;
  double Delta;
  double D;

  VehicleState(fsd_common_msgs::CarState state, fsd_common_msgs::ControlCommand cmd) {
    x = state.car_state.x;
    y = state.car_state.y;
    yaw = state.car_state.theta;
    v = std::hypot(state.car_state_dt.car_state_dt.x, state.car_state_dt.car_state_dt.y);
    r = state.car_state_dt.car_state_dt.theta;
    a = std::hypot(state.car_state_dt.car_state_a.x, state.car_state_dt.car_state_a.y);
    w = state.car_state_dt.car_state_a.theta;

    D = cmd.throttle.data;
    Delta = cmd.steering_angle.data;
  }
  VehicleState() {

  }
};

struct TrajectoryPoint {
  cv::Point2f pts;
  double yaw;
  double curvature;
  double velocity;
  double r;
  double acc;
};

typedef std::vector<TrajectoryPoint> Trajectory;

} // namespace waypoint_follower

#endif