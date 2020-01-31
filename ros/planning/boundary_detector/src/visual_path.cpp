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

#include "visual_path.hpp"

namespace FSD {
    void visual(cv::Subdiv2D coneSet, SearchTree Path, 
        fsd_common_msgs::Map boundaryDetections, 
        std::vector<PathPoint> BestPath,
        visualization_msgs::Marker &visualTriangles, 
        visualization_msgs::MarkerArray &visualTree, 
        visualization_msgs::MarkerArray &visualBoundary,
        visualization_msgs::Marker &visualPath
        ) {
        
        // visualization delaunay triangles

        std::vector<cv::Vec4f> edges;

		coneSet.getEdgeList(edges);

		// the virtual triangle three edges
		cv::Point2f outer_vtx[3];
		for (int i = 0; i < 3; i++)
		{
			outer_vtx[i] = coneSet.getVertex(i + 1);
		}

		visualization_msgs::Marker line_list;
		line_list.header.frame_id = "/base_link";
		line_list.header.stamp = ros::Time();
		line_list.ns = "points_and_lines";
		line_list.action = visualization_msgs::Marker::ADD;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		line_list.scale.x = 0.1;
		line_list.color.g = 1.0;
		line_list.color.a = 1.0;

		geometry_msgs::Point p1, p2;

		for (int i = 0; i < edges.size(); i++)
		{
			// abandon the far points of virtual triangle
			if (edges[i][0] == outer_vtx[0].x && edges[i][1] == outer_vtx[0].y ||
				edges[i][0] == outer_vtx[1].x && edges[i][1] == outer_vtx[1].y ||
				edges[i][0] == outer_vtx[2].x && edges[i][1] == outer_vtx[2].y ||
				edges[i][2] == outer_vtx[0].x && edges[i][3] == outer_vtx[0].y ||
				edges[i][2] == outer_vtx[1].x && edges[i][3] == outer_vtx[1].y ||
				edges[i][2] == outer_vtx[2].x && edges[i][3] == outer_vtx[2].y)
				continue;
			p1.x = edges[i][0];
			p1.y = edges[i][1];
			p1.z = 0;
			p2.x = edges[i][2];
			p2.y = edges[i][3];
			p2.z = 0;
			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
		}
		visualTriangles = line_list;
		
        // visualization Tree Path
        std::vector<std::vector<PathPoint>> tree_path;
        std::vector<SearchTree*> next;
        std::vector<SearchTree*> tmp;

        next.push_back(&Path);

        while(next.size() != 0) {
            tmp.clear();
            for(auto &iter :next) {
                if(iter->next.size() == 0) {
                    tree_path.push_back(iter->history);
                    //print_path(iter.history);
                }
                else {
                    for(auto &it:iter->next)
                        tmp.push_back(&it);
                }
            }
            next = tmp;
        }
        
        int path_num = 0;
        visualTree.markers.clear();
        for(const auto &iter: tree_path) {
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "/base_link";
            line_list.header.stamp = ros::Time();
            line_list.ns = "path" + std::to_string(path_num);
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.type = visualization_msgs::Marker::LINE_STRIP;
            line_list.scale.x = 0.1;
            line_list.color.r = 1.0;
            line_list.color.a = 1.0;
            geometry_msgs::Point p;
            for(const auto &it: iter) {
                p.x = it.x;
                p.y = it.y;
                line_list.points.push_back(p);
            }
            visualTree.markers.push_back(line_list);
            path_num++;
        }


        // visualization Boundary
        visualBoundary.markers.clear();
        visualization_msgs::Marker boundary_red;
        boundary_red.header.frame_id = "/base_link";
        boundary_red.header.stamp = ros::Time();
        boundary_red.ns = "red";
        boundary_red.action = visualization_msgs::Marker::ADD;
        boundary_red.type = visualization_msgs::Marker::LINE_STRIP;
        boundary_red.scale.x = 0.8;
        boundary_red.color.r = 1.0;
        boundary_red.color.a = 1.0;
        geometry_msgs::Point p;
        for(const auto &iter: boundaryDetections.cone_red) {
            p.x = iter.position.x;
            p.y = iter.position.y;
            boundary_red.points.push_back(p);
        }

        visualization_msgs::Marker boundary_blue;
        boundary_blue.header.frame_id = "/base_link";
        boundary_blue.header.stamp = ros::Time();
        boundary_blue.ns = "blue";
        boundary_blue.action = visualization_msgs::Marker::ADD;
        boundary_blue.type = visualization_msgs::Marker::LINE_STRIP;
        boundary_blue.scale.x = 0.8;
        boundary_blue.color.b = 1.0;
        boundary_blue.color.a = 1.0;
        for(const auto &iter: boundaryDetections.cone_blue) {
            p.x = iter.position.x;
            p.y = iter.position.y;
            boundary_blue.points.push_back(p);
        }

        visualBoundary.markers.push_back(boundary_red);
        visualBoundary.markers.push_back(boundary_blue);

        // visualization BestPath
        visualPath.points.clear();
        visualPath.header.frame_id = "/base_link";
        visualPath.header.stamp = ros::Time();
        visualPath.ns = "best_path";
        visualPath.action = visualization_msgs::Marker::ADD;
        visualPath.type = visualization_msgs::Marker::LINE_STRIP;
        visualPath.scale.x = 0.8;
        visualPath.color.r = 0;
        visualPath.color.g = 0;
        visualPath.color.b = 0;
        visualPath.color.a = 1.0;
        for(const auto &iter: BestPath) {
            p.x = iter.x;
            p.y = iter.y;
            visualPath.points.push_back(p);
        }

    }
}