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

#ifndef LINE_DETECTOR_HPP
#define LINE_DETECTOR_HPP

#include "sensor_msgs/PointCloud.h"
#include "fsd_common_msgs/Map.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace ns_line_detector {

class LineDetector {

 public:
  // Constructor
  LineDetector(ros::NodeHandle& nh);

	// Getters
  geometry_msgs::Point getendPoint();

	// Setters
  void setlidarCluster(fsd_common_msgs::Map msgs);

  void runAlgorithm();

private:

	ros::NodeHandle& nh_;
	
  tf2_ros::Buffer tfBuffer;
  sensor_msgs::PointCloud cluster;
  fsd_common_msgs::Map local_map;
  geometry_msgs::Point end_point;

  //世界坐标系与相机的转换关系
  float camera_odom_x;
  float camera_odom_y;

  bool getPath = false;
  double path_length;
  double allow_angle_error;

  void createPath();
};
}

#endif //LINE_DETECTOR_HPP
