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

#pragma once
#include "fsd_common_msgs/ControlCommand.h"
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include "Utils/types.h"
#include "Utils/param.h"

namespace ns_control {

class Solver {
public:
  void setTrajectory(const Trajectory &trajectory); // only for mpc
  void setState(const VehicleState &state);
  Trajectory getTrajectory();
  Trajectory predictive_path;
  fsd_common_msgs::ControlCommand getCmd();
  virtual void solve() = 0;

protected:
  Trajectory trajectory_;
  VehicleState state_;    // a marker point for skidpad
  fsd_common_msgs::ControlCommand control_command_;
};

}; // namespace ns_control
