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

#ifndef PATH_GENERATOR_HANDLE_HPP
#define PATH_GENERATOR_HANDLE_HPP

#include "path_generator.hpp"

namespace ns_path_generator {

    class PathGeneratorHandle {

    public:
        // Constructor
        PathGeneratorHandle(ros::NodeHandle &nodeHandle);

//  // Getters
        int getNodeRate() const;

        // Methods
        void loadParameters();

        void subscribeToTopics();

        void publishToTopics();

        void run();

        void sendMsg();
//  void sendVisualization();

    private:
        ros::NodeHandle nodeHandle_;
        ros::Subscriber endPointSubscriber_;
        ros::Subscriber transMatSubscriber_;
        ros::Subscriber localMapSubscriber_;
        ros::Subscriber carStateSubscriber_;

        ros::Publisher refPathVisualPublisher_;
        ros::Publisher refPathPublisher_;

        void endPointCallback(const geometry_msgs::Point &msg);
        void transMatCallback(const std_msgs::Float64MultiArray &msg);
        void carStateCallback(const fsd_common_msgs::CarState &msg);
        void localMapCallback(const fsd_common_msgs::Map &msg);


        std::string car_state_topic_name_;
        std::string transform_matrix_topic_name_;
        std::string end_point_topic_name_;
        std::string map_topic_name_;
        std::string ref_path_topic_name_;
        std::string path_generate_topic_name_;

        PathGenerator path_generator_;
        int node_rate_;

    };


}
#endif //PATH_GENERATOR_HANDLE_HPP
