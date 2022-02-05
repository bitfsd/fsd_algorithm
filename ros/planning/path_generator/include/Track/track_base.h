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

#include "fsd_common_msgs/Map.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "fsd_tools/cubic_spline.h"
#include "std_msgs/Int8.h"
#include "Utils/types.h"
#include "Utils/param.h"

#include <vector>

namespace ns_path_generator {

    class Track {
    public:
        Track() = default;

        ~Track() = default;

        virtual bool genTraj() = 0;

        virtual bool CalculateTraj(Trajectory &refline) = 0;

        void setMap(const fsd_common_msgs::Map &map);

        void setState(const VehicleState &state);

        void setTransMat(const Eigen::Matrix4f &transMat);

        void setEndPoint(const geometry_msgs::Point &endPoint);

    protected:
        fsd_common_msgs::Map map_;
        VehicleState state_;
        Trajectory trajectory_;
        geometry_msgs::Point endPoint_; // an endpoint for acceleration
        Eigen::Matrix4f transMat_;

    };

} // namespace ns_path_generator