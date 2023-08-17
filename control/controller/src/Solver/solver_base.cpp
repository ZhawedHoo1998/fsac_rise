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

#include "Solver/solver_base.h"
#include "ros/ros.h"


namespace ns_control {

// 获取类成员变量 predictive_path
Trajectory Solver::getTrajectory() { return predictive_path; }

// 设置类成员变量 trajectory_ 的值，该变量表示机器人的路径轨迹
void Solver::setTrajectory(const Trajectory &trajectory) {
  ROS_INFO_STREAM("set solver trajectory");
  trajectory_ = trajectory;
}

// 设置类成员变量 state_ 的值，该变量表示车辆的状态
void Solver::setState(const VehicleState &state) { 
  ROS_INFO_STREAM("set state");
  state_ = state; 
  ROS_INFO_STREAM("finish set state");
}

// 获取类成员变量 control_command_，并将其作为返回值返回
fsd_common_msgs::ControlCommand Solver::getCmd() {
  return control_command_;
}

} // namespace ns_mpc
