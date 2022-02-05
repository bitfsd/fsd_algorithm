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

#include "Solver/mpc_solver.h"
#include "ros/ros.h"
#include "Utils/param.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cmath>

namespace ns_control {

void MPC_Solver::solve() {
  using CppAD::AD;
  const size_t N = control_param_.N;
  const double dt = control_param_.dt;
  const double Lf = control_param_.car_length;
  const int px_range_begin       = 0;
  const int py_range_begin       = px_range_begin    + N;
  const int psi_range_begin      = py_range_begin    + N;
  const int v_range_begin        = psi_range_begin   + N;
  const int cte_range_begin      = v_range_begin     + N;
  const int epsi_range_begin     = cte_range_begin   + N;
  const int steering_range_begin = epsi_range_begin  + N;
  const int throttle_range_begin = steering_range_begin + (N - 1);
    ROS_INFO_STREAM("begin solve");
  if(trajectory_.empty()) {
      ROS_INFO_STREAM("trajectory empty");
      return;
  }
  double v      = state_.v;
  double delta  = state_.Delta;
  double a      = state_.a;

  double actuator_latency = 0;
  const double px = 0.0 + v * actuator_latency;
  const double py = 0.0;
  const double psi = 0.0 + v * (-delta) / Lf * actuator_latency;
  v = v + a * actuator_latency;


  int state_size = 6;
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  size_t n_vars = state_size * N + 2 * (N-1);

  size_t n_constraints = state_size * N;

  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
  {
    vars[i] = 0;
  }

  vars[px_range_begin]        = px;
  vars[py_range_begin]        = py;
  vars[psi_range_begin]       = psi;
  vars[v_range_begin]         = v;
  vars[cte_range_begin]       = cte;
  vars[epsi_range_begin]      = epsi;
  vars[steering_range_begin]  = 0.0;
  vars[throttle_range_begin]  = 0.0;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  for (int i = 0; i < steering_range_begin; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (int i = steering_range_begin; i < throttle_range_begin; i++)
  {
    vars_lowerbound[i] = -0.8;
    vars_upperbound[i] = 0.8;
  }

  for (int i = throttle_range_begin; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = +1.0;
  }

  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }

  constraints_lowerbound[px_range_begin]    = px;
  constraints_upperbound[px_range_begin]    = px;

  constraints_lowerbound[py_range_begin]    = py;
  constraints_upperbound[py_range_begin]    = py;

  constraints_lowerbound[psi_range_begin]  = psi;
  constraints_upperbound[psi_range_begin]  = psi;

  constraints_lowerbound[v_range_begin]    = v;
  constraints_upperbound[v_range_begin]    = v;

  constraints_lowerbound[cte_range_begin]  = cte;
  constraints_upperbound[cte_range_begin]  = cte;

  constraints_lowerbound[epsi_range_begin] = epsi;
  constraints_upperbound[epsi_range_begin] = epsi;

  std::string options;
  options += "Integer print_level  0\n";
  
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;

  FG_eval fg_eval(trajectory_);
  
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  double cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  for (int i = 0; i < N; i++)
  {
    geometry_msgs::Point32 p;
    p.x = solution.x[px_range_begin + i];
    p.y = solution.x[py_range_begin + i];
  }
  control_command_.steering_angle.data = solution.x[steering_range_begin];
  control_command_.throttle.data = solution.x[throttle_range_begin];
  ROS_INFO_STREAM(control_command_.steering_angle.data);
  ROS_INFO_STREAM(control_command_.throttle.data);

  predictive_path.clear();
  TrajectoryPoint p_tmp;
  for (int i = 0; i < N; i++)
  {
    geometry_msgs::Point32 p;
    p_tmp.pts.x = solution.x[px_range_begin + i];
    p_tmp.pts.y = solution.x[py_range_begin + i];
    p_tmp.velocity = solution.x[v_range_begin + i];
    predictive_path.push_back(p_tmp);
  }
}

}