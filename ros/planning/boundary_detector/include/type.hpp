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

#ifndef TYPE_HPP
#define TYPE_HPP

#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>


namespace FSD{

struct ConePos {

    // Cone Position
    float x,y;

    // Init
    ConePos(float x_ = 0, float y_ = 0) {
        x = x_;
        y = y_;
    }

    // operator overload
    bool operator < (const ConePos &a)const {
        if (x == a.x)   return y < a.y;
        else return x < a.x;
    }

    // operator overload
    bool operator != (const ConePos &b)const {
        if(fabs(b.x - x) > 1e-2 || fabs(b.y - y) > 1e-2) 
            return true;
        return false;
    }
};

struct PathPoint {

    // Path Point id
    int id;

    // path point position
    float x,y;

    // Each edge vertex. Generally left means red, right means blue.
    ConePos left_cone;
    ConePos right_cone;
    
    // wrong color number
    // If color is unknown, wrong_color_cnt add 0.5
    // If color is right, wrong_color_cnt add 0
    // If color is wrong, wrong_color_cnt add 1
    float wrong_color_cnt;

    // Edge distance
    double edge_dst;

    // Init
    PathPoint() {
        x = 0; y = 0;
        wrong_color_cnt = 0;
        edge_dst = 0;
    }

    // operator overload
    bool operator == (const PathPoint &b)const {
        if(fabs(b.x - x) < 1e-2 || fabs(b.y - y) < 1e-2) 
            return true;
        return false;
    }

    // Calculate edge distance
    void CalculateDst() {
        edge_dst = std::hypot(left_cone.x - right_cone.x, left_cone.y - right_cone.y);
    }

    // Calculate Mid Point
    void CalculateMidPoint() {
        x = (left_cone.x + right_cone.x)/2.0;
        y = (left_cone.y + right_cone.y)/2.0;
    }

    // Reverse Cone Color
    PathPoint ReverseCone() {
        PathPoint rev;
        rev.x = x;
        rev.y = y;
        rev.left_cone = right_cone;
        rev.right_cone = left_cone;
        rev.wrong_color_cnt = 2 - wrong_color_cnt;
        rev.edge_dst = edge_dst;
        rev.id = id + 1;
        return rev;
    }
};

// Node cost weight
struct Cost_n {
    double w_k;
    double w_w;
    double w_c;
    double w_d;
};

// Path cost weight
struct Cost_p {
    double w_k;
    double w_w;
    double w_c;
    double w_b;
    double w_r;
};

// help to find low cost index
struct Cost_index {
    int index;
    double cost;
    Cost_index(int a,double b) {
        index = a;
        cost = b;
    }
};

struct SearchTree {

    // Current Path Point of each Path
    PathPoint Node;

    // Next branch
    std::vector<SearchTree> next;

    // History trunk
    std::vector<PathPoint> history;

    // The whole cost
    double all_cost;

    // Current Node cost
    double current_cost;

    // Cost Weight
    Cost_n node_cost_weight;
    Cost_p path_cost_weight;

    // Params
    double track_width = 3;
    double track_length = 3.5;
    double sensor_range = 15;

    // Init
    SearchTree() {
        all_cost = 0;
        current_cost = 0;
    }

    double CalculateStd(std::vector<double> vec, double mean) {
        double accum  = 0.0;
        std::for_each (std::begin(vec), std::end(vec), [&](const double d) {
            accum  += (d-mean)*(d-mean);
        });

        double std = sqrt(accum/vec.size());
        return std;
    }

    // Calculate Cost, if add the next node
    double CalculateCurrentCost(PathPoint curr) {
        int N = history.size();

        //cost
        double cost;
        double cost_color;
        double cost_width;
        double cost_theta;
        double cost_distance;

        cost_color = curr.wrong_color_cnt;
        cost_width = fabs(curr.edge_dst - track_width);
        cost_distance = fabs(std::hypot(history[N-1].x - curr.x, history[N-1].y - curr.y) - track_width);

        if(N < 2)
            cost_theta = atan(fabs(curr.y)/curr.x);
        else {
            PathPoint prev = history[N-2];
            PathPoint last = history[N-1];

            double dist_a = (curr.x - last.x) * (curr.x - last.x) + (curr.y - last.y) * (curr.y - last.y);
			double dist_b = (prev.x - last.x) * (prev.x - last.x) + (prev.y - last.y) * (prev.y - last.y);
			double dist_c = (prev.x - curr.x) * (prev.x - curr.x) + (prev.y - curr.y) * (prev.y - curr.y);

			double angle_cos = (dist_a + dist_b - dist_c) / sqrt(4 * dist_a * dist_b);

            if (angle_cos > 1)
				angle_cos = 1;
			if (angle_cos < -1)
				angle_cos = -1;

			cost_theta = M_PI - acos(angle_cos);

        }

        cost = cost_color * node_cost_weight.w_c + cost_theta * node_cost_weight.w_k / M_PI * 2 + cost_width * node_cost_weight.w_w + cost_distance * node_cost_weight.w_d;

        return cost;
    }

    double CalculateAllCost() {

        double cost;                // total cost
        double cost_theta;          // max theta change
        double cost_std_width;      // standard deviation of track width
        double cost_std_bound;      // standard deviation of each boundary distance
        double cost_error_color;    // total color error
        double cost_sensor_range;   // Squared diﬀerence between path length and sensor range
        
        std::vector<double> theta;
        std::vector<double> width;
        std::vector<ConePos> bound_l;
        std::vector<ConePos> bound_r;

        std::vector<double> bound_ld;
        std::vector<double> bound_rd;

        double color = 0;
        double range = 0;

        for(int i = 1; i < history.size(); i++) {
            if(i == 1) {
                theta.push_back(atan(fabs(history[i].y/history[i].x)));
                bound_l.push_back(history[i].left_cone);
                bound_r.push_back(history[i].right_cone);
            }
            else {
                if(history[i].left_cone != bound_l[bound_l.size()-1]) {
                    bound_ld.push_back(std::hypot(bound_l[bound_l.size()-1].x - history[i].left_cone.x, bound_l[bound_l.size()-1].y - history[i].left_cone.y));
                    bound_l.push_back(history[i].left_cone);
                }
                if(history[i].right_cone != bound_r[bound_r.size()-1]) {
                    bound_rd.push_back(std::hypot(bound_r[bound_r.size()-1].x - history[i].right_cone.x, bound_r[bound_r.size()-1].y - history[i].right_cone.y));
                    bound_r.push_back(history[i].right_cone);
                }

                PathPoint prev = history[i-2];
                PathPoint last = history[i-1];
                PathPoint curr = history[i];

                double dist_a = (curr.x - last.x) * (curr.x - last.x) + (curr.y - last.y) * (curr.y - last.y);
                double dist_b = (prev.x - last.x) * (prev.x - last.x) + (prev.y - last.y) * (prev.y - last.y);
                double dist_c = (prev.x - curr.x) * (prev.x - curr.x) + (prev.y - curr.y) * (prev.y - curr.y);

                double angle_cos = (dist_a + dist_b - dist_c) / sqrt(4 * dist_a * dist_b);

                if (angle_cos > 1)
                    angle_cos = 1;
                if (angle_cos < -1)
                    angle_cos = -1;

                theta.push_back(M_PI - acos(angle_cos));
            }
            width.push_back(history[i].edge_dst);
            color += history[i].wrong_color_cnt;
            range += std::hypot(history[i].x - history[i-1].x, history[i].y - history[i-1].y);
        }

        cost_theta = *std::max_element(theta.begin(),theta.end());
        cost_std_width = CalculateStd(width, track_width);
        cost_std_bound = CalculateStd(bound_ld, track_length) + CalculateStd(bound_rd, track_length);
        cost_error_color = color;
        cost_sensor_range = fabs(range - sensor_range);

        cost = cost_theta * path_cost_weight.w_k + cost_std_width * path_cost_weight.w_w + cost_std_bound * path_cost_weight.w_b +
        cost_error_color * path_cost_weight.w_c + cost_sensor_range * path_cost_weight.w_r;

        return cost;
    }

    // Check exist
    bool CheckExist(PathPoint curr) {
        int curr_id = curr.id;
        for(const auto iter: history) {
            if(iter.id == 0)
                continue;
            
            if((iter.id-1)/2 == (curr_id-1)/2) {
                return true;
            }
        }
        return false;
    }
};



};

#endif