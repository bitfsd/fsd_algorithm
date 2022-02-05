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

#ifndef PATH_GENERATOR_HPP
#define PATH_GENERATOR_HPP

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/Trajectory.h"
#include "fsd_common_msgs/TrajectoryPoint.h"
#include "std_msgs/Float64MultiArray.h"
#include "Utils/param.h"
#include "Utils/types.h"
#include "Track/track_base.h"
#include "Track/line_track.h"
#include "Track/skidpad_track.h"
#include "Track/trackdrive_track.h"
#include "visualization_msgs/MarkerArray.h"

namespace ns_path_generator {

    class PathGenerator {

    public:
        // Constructor
        PathGenerator(ros::NodeHandle &nh);

        // Getters

        visualization_msgs::MarkerArray getRefPath();
        fsd_common_msgs::Trajectory getRefTrajectory();

        // Setters
        void setCarState(const fsd_common_msgs::CarState &state);

        void setEndPoint(const geometry_msgs::Point &point);

        void setLocalMap(const fsd_common_msgs::Map &map);

        void setTransMat(const std_msgs::Float64MultiArray &array);

        void runAlgorithm();

    private:
        ros::NodeHandle &nh_;
        std::string mission_;
        geometry_msgs::Point endPoint_;
        Eigen::Matrix4f transMat_;
        fsd_common_msgs::Map local_map_;
        fsd_common_msgs::CarState car_state_;
        Trajectory refline_;

        fsd_common_msgs::ControlCommand cmd_;
        Track *track_;
        Autox_Track trackdrive_track_;
        Line_Track line_track_;
        Skidpad_Track skidpad_track_;

        visualization_msgs::MarkerArray RefPath_;
        bool is_init = false;

//Methods
        bool Check();

        void setTrack();
    };
}

#endif //PATH_GENERATOR_HPP
