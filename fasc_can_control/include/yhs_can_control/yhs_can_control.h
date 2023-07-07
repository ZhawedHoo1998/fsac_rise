#ifndef __PIDCONTROL_NODE_H__
#define __PIDCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/odo_fb.h"


#include "yhs_can_msgs/vehicle_status.h" //111
#include "sensor_msgs/NavSatFix.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>


namespace yhs_tool {
class CanControl
{
public:
	CanControl();
	~CanControl();
	
	void run();

private:
	ros::NodeHandle nh_;

	//publishing IMU gps vehicle_status


	ros::Publisher vehicle_status_pub_;
	ros::Publisher autoware_status_pub_;
	ros::Publisher IMU_pub_;
	ros::Publisher GPS_pub_;


	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber io_cmd_sub_;
	ros::Subscriber autoware_cmd_sub_; 

	boost::mutex cmd_mutex_;

	unsigned char sendData_u_io_[8] = {0};
	unsigned char sendData_u_vel_[8] = {0};

	int dev_handler_;
	can_frame send_frames_[2];
	can_frame recv_frames_[1];


	void io_cmdCallBack(const yhs_can_msgs::io_cmd msg);
	void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg);
	void autoware_ctrl_cmdCallBack(const geometry_msgs::TwistStamped msg);

	void recvData();
	void sendData();

};

}


#endif

