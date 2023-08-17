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

#include <ros/ros.h>
#include "Utils/param.h"
#include "control.hpp"
#include <sstream>



namespace ns_control {

    Param control_param_;

    // 初始化ROS节点的句柄
    Control::Control(ros::NodeHandle &nh) : nh_(nh) {

        controller_ = nh_.param<std::string>("controller", "pure_pursuit");
        control_param_.getParams(nh_, controller_);

        if (controller_ == "pure_pursuit") { solver_ = &pure_pursuit_solver_; }
        else if (controller_ == "mpc") { solver_ = &mpc_solver_; }
        else {
            ROS_ERROR("Undefined Solver name !");
        }
    }

    // 分别实现了设置车辆状态、设置轨迹、获取控制命令和获取预处理路径的功能
    void Control::setCarState(const fsd_common_msgs::CarState &msgs) { car_state_ = msgs; }

    void Control::setTrack(const Trajectory &msgs) { refline_ = msgs; }

    // 获取控制命令，以便将其发送到执行机构进行执行
    fsd_common_msgs::ControlCommand Control::getCmd() { return cmd_; }

    // 获取预处理路径，以便在可视化界面中进行显示
    visualization_msgs::MarkerArray Control::getPrePath() { return PrePath_; }
    
    // 检查参考轨迹是否为空
    bool Control::Check() {
        if (refline_.empty()) {
            ROS_DEBUG_STREAM("Successfully passing check");
            return false;
        }
        return true;
    }

    // 控制算法的运行和结果的处理
    void Control::runAlgorithm() {
        if (!Check()) {
            ROS_WARN_STREAM("Check Error");
            return;
        }

        solver_->setState(VehicleState(car_state_, cmd_));
        solver_->setTrajectory(refline_);
        solver_->solve();

        cmd_ = solver_->getCmd();

        std::vector<float> color_ref = {1, 0, 0};
        std::vector<float> color_pre = {0, 1, 0};
        std::vector<float> color_init = {0, 0, 1};

        if (controller_ == "mpc")
            visual_trajectory(solver_->getTrajectory(), PrePath_, "/base_link",
                              color_pre, car_state_.header, true);

        // 发布信息
        std::cout << "steering: " << cmd_.steering_angle.data << std::endl;
        std::cout << "throttle: " << cmd_.throttle.data << std::endl;
    }
}
