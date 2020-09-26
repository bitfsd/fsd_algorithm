#pragma once
#include "fsd_common_msgs/Map.h"
#include "Utils/param.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "Utils/types.h"

namespace ns_control {
void visual_trajectory(const Trajectory &traj,
                       visualization_msgs::MarkerArray &visual,
                       const std::string &frame, const std::vector<float> &color,
                       const std_msgs::Header &header, bool is_vel);
void color_map(double vel, std::vector<float> &color);
void visual_map(const fsd_common_msgs::Map &map,
                visualization_msgs::MarkerArray &visual);
} // namespace ns_mpc