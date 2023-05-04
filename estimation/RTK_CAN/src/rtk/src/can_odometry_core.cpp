/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "can_odometry_core.h"
#include <cmath>

// Constructor
CanOdometryNode::CanOdometryNode() : private_nh_("~"), v_info_(), odom_(ros::Time::now())
{
  initForROS();
}

// Destructor
CanOdometryNode::~CanOdometryNode()
{
}

void CanOdometryNode::initForROS()
{
  // ros parameter settings  设置轮距
  // if (!nh_.hasParam("/vehicle_info/wheel_base"))
  // {
  //   v_info_.is_stored = false;
  //   ROS_INFO("vehicle_info is not set");
  // }
  // else
  // {
  //   private_nh_.getParam("/vehicle_info/wheel_base", v_info_.wheel_base);
  //   // ROS_INFO_STREAM("wheel_base : " << wheel_base);

  //   v_info_.is_stored = true;
  // }
  v_info_.is_stored = true;

  // setup subscriber
  sub1_ = nh_.subscribe("/ctrl_fb", 10, &CanOdometryNode::callbackFromVehicleStatus, this);

  // setup publisher
  pub1_ = nh_.advertise<nav_msgs::Odometry>("/vehicle/odom", 10);
}

void CanOdometryNode::run()
{
  ros::spin();
}

void CanOdometryNode::publishOdometry(const yhs_can_msgs::ctrl_fbConstPtr& msg)
{
  float vx = msg->ctrl_fb_velocity ; // 速度转换 单位
  // std::cout << "vx = " << vx << std::endl; 
  // std::cout << "angular = " << msg->ctrl_fb_steering << std::endl; 
  float vth_rad = ( msg->ctrl_fb_steering * M_PI ) / 180;
  // std::cout << "vth_rad = " << vth_rad << std::endl; 
  float vth = v_info_.convertSteeringAngleToAngularVelocity(msg->ctrl_fb_velocity, vth_rad);
  // std::cout << "vth = " << vth << std::endl; 
  odom_.updateOdometry(vx, vth, msg->header.stamp);  

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_.th);  //将此时车辆的转向角转换为 四元数 只有绕z轴的旋转角

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = odom_.x;
  // ROS_INFO("odom.x:%d",odom_.x);
  odom.pose.pose.position.y = odom_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";  //修改_1
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = vth;

  ///tf2_ros
  //   发布TF
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = msg->header.stamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id  = "base_link"; //修改_1
 
  geometry_msgs::Quaternion odom_quat_1;
  odom_quat_1 = tf::createQuaternionMsgFromYaw(odom_.th);
  odom_trans.transform.translation.x = odom_.x;
  odom_trans.transform.translation.y = odom_.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat_1;
        
  odom_broadcaster.sendTransform(odom_trans);
  //ROS_INFO("SUCCEED");

  // publish the message
  pub1_.publish(odom);

}

void CanOdometryNode::callbackFromVehicleStatus(const yhs_can_msgs::ctrl_fbConstPtr& msg)
{
  publishOdometry(msg);
}


