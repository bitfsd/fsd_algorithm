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

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "Utils/types.h"
#include "Utils/param.h"
#include "Utils/visual.h"

#include "fsd_common_msgs/Trajectory.h"
#include "fsd_common_msgs/TrajectoryPoint.h"

#include "Solver/solver_base.h"
#include "Solver/mpc_solver.h"
#include "Solver/pure_pursuit_solver.h"

namespace ns_control {

    class Control {

    public:
        Control(ros::NodeHandle &nh);

        void runAlgorithm();

        void setCarState(const fsd_common_msgs::CarState &msgs);

        void setTrack(const Trajectory &msgs);

        visualization_msgs::MarkerArray getPrePath();

        fsd_common_msgs::ControlCommand getCmd();

        visualization_msgs::MarkerArray PrePath_;

    private:

        bool Check();

    private:

        ros::NodeHandle &nh_;
        std::string controller_;

        Solver *solver_;
        MPC_Solver mpc_solver_;
        Pure_Pursuit_Solver pure_pursuit_solver_;

        fsd_common_msgs::CarState car_state_;
        fsd_common_msgs::ControlCommand cmd_;

        Trajectory refline_;

        bool is_init = false;
    };
}

#endif //CONTROL_HPP
