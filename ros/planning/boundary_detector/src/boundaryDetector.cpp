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

#include <ros/ros.h>
#include "boundaryDetector.hpp"
#include <sstream>

namespace ns_boundaryDetector {

// Constructor
BoundaryDetector::BoundaryDetector(ros::NodeHandle &nh) : nh_(nh) {
  loadParameters();
};

void BoundaryDetector::loadParameters() {
    if (!nh_.param("max_iter_num", max_iter_num_, 5)) {
		ROS_WARN_STREAM("Did not load max_iter_num. Standard value is: " << max_iter_num_);
	}
    if (!nh_.param("max_search_num", max_search_num_, 3)) {
		ROS_WARN_STREAM("Did not load max_search_num. Standard value is: " << max_search_num_);
	}
    if (!nh_.param("max_beam_cost", max_beam_cost_, 1.5)) {
		ROS_WARN_STREAM("Did not load max_beam_cost. Standard value is: " << max_beam_cost_);
	}

    beam_weight_.w_c = nh_.param("beam_weight/w_c", beam_weight_.w_c);
    beam_weight_.w_k = nh_.param("beam_weight/w_k", beam_weight_.w_k);
    beam_weight_.w_w = nh_.param("beam_weight/w_w", beam_weight_.w_w);
    beam_weight_.w_d = nh_.param("beam_weight/w_d", beam_weight_.w_d);

    path_weight_.w_k = nh_.param("path_weight/w_k", path_weight_.w_k);
    path_weight_.w_w = nh_.param("path_weight/w_w", path_weight_.w_w);
    path_weight_.w_c = nh_.param("path_weight/w_c", path_weight_.w_c);
    path_weight_.w_b = nh_.param("path_weight/w_b", path_weight_.w_b);
    path_weight_.w_r = nh_.param("path_weight/w_r", path_weight_.w_r);

}

// Getters
fsd_common_msgs::Map BoundaryDetector::getboundaryDetections() { return boundaryDetections; }
visualization_msgs::Marker BoundaryDetector::getVisualTriangles() { return visualTriangles; }
visualization_msgs::MarkerArray BoundaryDetector::getVisualBoundary() { return visualBoundary; }
visualization_msgs::MarkerArray BoundaryDetector::getVisualTree() { return visualTree; }
visualization_msgs::Marker BoundaryDetector::getVisualPath() { return visualPath; }

// Setters
void BoundaryDetector::setLocalMap(fsd_common_msgs::Map msg) {
    map_current = msg;
}

void BoundaryDetector::runAlgorithm() {

    // Init Map
    map = map_current;
    if(map.cone_red.size() == 0 || map.cone_blue.size() == 0) {return;}
    if(!filter(map)) {return;}

    // Build delaunay triangles
    cv::Rect rect(-200, -200, 400, 400);
    cv::Subdiv2D coneSet(rect);
    std::map<ConePos, char> colorMap;    // Key: Cones Position -> Map: Cones Color ( r, b, u )
    std::map<int, PathPoint> MidSet;
    initSet(map, coneSet, colorMap);
    getMidPoint(coneSet, colorMap, MidSet);
    
    // Tree-Search Best Path
    SearchTree Path;
    std::vector<PathPoint> BestPath;
    searchPath(MidSet, Path);
    selectBestPath(Path, BestPath);
    generateBoundary(BestPath, boundaryDetections);

    // Visualization
    FSD::visual(coneSet, Path, boundaryDetections, BestPath, visualTriangles, visualTree, visualBoundary, visualPath);
}

bool BoundaryDetector::filter(fsd_common_msgs::Map &init_map) {
    fsd_common_msgs::Map map;

    // filter cones that x <= 0
    for(const auto &red: init_map.cone_red) {
        if (red.position.x > 0 && std::hypot(red.position.x, red.position.y) < 15) {
            map.cone_red.push_back(red);
        }
    }
    for(const auto &blue: init_map.cone_blue) {
        if (blue.position.x > 0 && std::hypot(blue.position.x, blue.position.y) < 15) {
            map.cone_blue.push_back(blue);
        }
    }
    for(const auto &unknow: init_map.cone_unknow) {
        if (unknow.position.x > 0 && std::hypot(unknow.position.x, unknow.position.y) < 15) {
            map.cone_unknow.push_back(unknow);
        }
    }

    init_map = map;

    if(init_map.cone_red.size() >= 2 && init_map.cone_blue.size() >= 2)
        return true;
    else 
        return false;

}

void BoundaryDetector::initSet(fsd_common_msgs::Map map, cv::Subdiv2D &coneSet, std::map<ConePos,char> &colorMap) {

    // insert points to create delaunay triangles and Pos2color Map
    for (const auto &iter: map.cone_red) {
        cv::Point2f fp(iter.position.x, iter.position.y);
        ConePos pt(iter.position.x, iter.position.y);
        coneSet.insert(fp);
        colorMap[pt] = 'r';
    }
    for (const auto &iter: map.cone_blue) {
        cv::Point2f fp(iter.position.x, iter.position.y);
        ConePos pt(iter.position.x, iter.position.y);
        coneSet.insert(fp);
        colorMap[pt] = 'b';
    }
    for (const auto &iter: map.cone_unknow) {
        cv::Point2f fp(iter.position.x, iter.position.y);
        ConePos pt(iter.position.x, iter.position.y);
        coneSet.insert(fp);
        colorMap[pt] = 'u';
    }

    return;
}

void BoundaryDetector::getMidPoint(cv::Subdiv2D coneSet, std::map<ConePos,char> colorMap, std::map<int, PathPoint> &MidSet) {

    std::vector<cv::Vec4f> edges;
    coneSet.getEdgeList(edges);
    
    // the virtual triangle three edges
    cv::Point2f outer_vtx[3];
    for (int i = 0; i < 3; i++)
        outer_vtx[i] = coneSet.getVertex(i + 1);

    // add car itself to Mid Point
    {
        PathPoint car;
        car.id = 0;
        car.wrong_color_cnt = 0;
        car.x = 0;
        car.y = 0;
        car.edge_dst = 0;
        MidSet[0] = car;
    }

    // get Mid Point
    for (int i = 0, j = 0; i < edges.size(); i++) {

        PathPoint tmp;

        // abandon the points of the virtual triangle
        if (edges[i][0] == outer_vtx[0].x && edges[i][1] == outer_vtx[0].y ||
            edges[i][0] == outer_vtx[1].x && edges[i][1] == outer_vtx[1].y ||
            edges[i][0] == outer_vtx[2].x && edges[i][1] == outer_vtx[2].y ||
            edges[i][2] == outer_vtx[0].x && edges[i][3] == outer_vtx[0].y ||
            edges[i][2] == outer_vtx[1].x && edges[i][3] == outer_vtx[1].y ||
            edges[i][2] == outer_vtx[2].x && edges[i][3] == outer_vtx[2].y)
            continue;
        
        if(colorMap[ConePos(edges[i][0], edges[i][1])] == colorMap[ConePos(edges[i][2], edges[i][3])]) {
            tmp.left_cone = ConePos(edges[i][0], edges[i][1]);
            tmp.right_cone = ConePos(edges[i][2], edges[i][3]);
            tmp.wrong_color_cnt += 1;
        }
        else if(colorMap[ConePos(edges[i][0], edges[i][1])] == 'r') {
            tmp.left_cone = ConePos(edges[i][0], edges[i][1]);
            tmp.right_cone = ConePos(edges[i][2], edges[i][3]);
            if(colorMap[ConePos(edges[i][2], edges[i][3])] == 'u') 
                tmp.wrong_color_cnt += 0.5;
        }
        else if(colorMap[ConePos(edges[i][0], edges[i][1])] == 'b') {
            tmp.left_cone = ConePos(edges[i][2], edges[i][3]);
            tmp.right_cone = ConePos(edges[i][0], edges[i][1]);
            if(colorMap[ConePos(edges[i][2], edges[i][3])] == 'u')
                tmp.wrong_color_cnt += 0.5;
        }
        else if(colorMap[ConePos(edges[i][0], edges[i][1])] == 'u') {
            tmp.wrong_color_cnt += 0.5;
            if(colorMap[ConePos(edges[i][2], edges[i][3])] == 'r') {
                tmp.left_cone = ConePos(edges[i][2], edges[i][3]);
                tmp.right_cone = ConePos(edges[i][0], edges[i][1]); 
            }
            else {
                tmp.left_cone = ConePos(edges[i][0], edges[i][1]);
                tmp.right_cone = ConePos(edges[i][2], edges[i][3]);
            }
        }

        tmp.CalculateDst();
        tmp.CalculateMidPoint();
        tmp.id = 2*j+1;
        MidSet[2*j+1] = tmp;
        MidSet[2*j+2] = tmp.ReverseCone();
        j++;
    }
    
    return;
}

void BoundaryDetector::searchPath(std::map<int, PathPoint> MidSet, SearchTree &Path) {
    Path.history.push_back(Path.Node);
    
    Path.node_cost_weight = beam_weight_;
    Path.path_cost_weight = path_weight_;

    std::vector<SearchTree*> next_set;
    std::vector<SearchTree*> next_tmp;

    next_set.push_back(&Path);

    for(int iter = 0; iter < max_iter_num_; iter++) {
        next_tmp.clear();
        std::vector<Cost_index> v_c;
        for(auto &leaf: next_set) {

            // Calculate each cost which current node connects to next lead to
            v_c.clear();
            for(int i = 1; i < MidSet.size(); i++) {
                if(leaf->CheckExist(MidSet[i]))
                    continue;
                double cost_curr_tmp = leaf->CalculateCurrentCost(MidSet[i]);

                if(cost_curr_tmp > max_beam_cost_)
                    continue;
                v_c.push_back(Cost_index(i, cost_curr_tmp));
            }

            // Select max_search_num_ smallest cost as next node
            sort(v_c.begin(),v_c.end(),
                    [&](const Cost_index &a, const Cost_index &b) {
                        return a.cost < b.cost;
                    });

            leaf->next.resize(std::min(max_search_num_, int(v_c.size())));

            for(int i = 0; i < std::min(max_search_num_, int(v_c.size())); i++) {
                SearchTree branch;
                branch.Node = MidSet[v_c[i].index];
                branch.history = leaf->history; branch.history.push_back(MidSet[v_c[i].index]);
                branch.node_cost_weight = leaf->node_cost_weight;
                branch.path_cost_weight = leaf->path_cost_weight;
                leaf->next[i] = branch;
                next_tmp.push_back(&(leaf->next[i]));

                leaf->CalculateCurrentCost(MidSet[v_c[i].index]);

            }
        }
        if(v_c.size() == 0)
            break;
        next_set = next_tmp;
    }

    return;
}

void BoundaryDetector::selectBestPath(SearchTree Path, std::vector<PathPoint> &BestPath) {
    std::vector<std::vector<PathPoint>> tree_path;
    std::vector<SearchTree*> next;
    std::vector<SearchTree*> tmp;
    std::vector<Cost_index> costSet;
    next.push_back(&Path);

    int index_i = 0;

    while(next.size() != 0) {
        tmp.clear();
        for(auto &iter :next) {
            if(iter->next.size() == 0) {
                if(iter->history.size() <= 1)
                    continue;
                tree_path.push_back(iter->history);
                costSet.push_back(Cost_index(index_i, iter->CalculateAllCost()));
                index_i++;
            }
            else {
                for(auto &it:iter->next)
                    tmp.push_back(&it);
            }
        }
        next = tmp;
    }
    sort(costSet.begin(),costSet.end(),
                    [&](const Cost_index &a, const Cost_index &b) {
                        return a.cost < b.cost;
                    });
    if (tree_path.empty()) {
        ROS_ERROR("empty tree!\n");
        return;
    }
    BestPath = tree_path[costSet[0].index];

    return;
}

void BoundaryDetector::generateBoundary(std::vector<PathPoint> BestPath, fsd_common_msgs::Map &Boundary) {

    std::vector<ConePos> bound_l;
    std::vector<ConePos> bound_r;

    Boundary.cone_red.clear();
    Boundary.cone_blue.clear();

    fsd_common_msgs::Cone tmp;

    for(int i = 1; i < BestPath.size(); i++) {
        if(i == 1) {
            bound_l.push_back(BestPath[i].left_cone);
            bound_r.push_back(BestPath[i].right_cone);

            tmp.position.x = BestPath[i].left_cone.x;
            tmp.position.y = BestPath[i].left_cone.y;
            tmp.color.data = "r";
            Boundary.cone_red.push_back(tmp);

            tmp.position.x = BestPath[i].right_cone.x;
            tmp.position.y = BestPath[i].right_cone.y;
            tmp.color.data = "b";
            Boundary.cone_blue.push_back(tmp);
        }
        else {
            if(BestPath[i].left_cone != bound_l[bound_l.size()-1]) {
                bound_l.push_back(BestPath[i].left_cone);
                tmp.position.x = BestPath[i].left_cone.x;
                tmp.position.y = BestPath[i].left_cone.y;
                tmp.color.data = "r";
                Boundary.cone_red.push_back(tmp);
            }
            if(BestPath[i].right_cone != bound_r[bound_r.size()-1]) {
                bound_r.push_back(BestPath[i].right_cone);
                tmp.position.x = BestPath[i].right_cone.x;
                tmp.position.y = BestPath[i].right_cone.y;
                tmp.color.data = "b";
                Boundary.cone_blue.push_back(tmp);
            }
        }
    }
}

}
