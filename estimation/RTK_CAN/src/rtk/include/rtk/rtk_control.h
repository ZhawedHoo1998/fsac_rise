#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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

#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/odo_fb.h"
#include "yhs_can_msgs/bms_Infor_fb.h"
#include "yhs_can_msgs/bms_flag_Infor_fb.h"
#include "yhs_can_msgs/Drive_MCUEcoder_fb.h"
#include "yhs_can_msgs/Veh_Diag_fb.h"


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <ros/time.h>



//程序启动的GPS坐标 要求设定后不能再改动！！ 起始坐标系的绝对点  const常量  定义全局变量 const 而后通过指针赋值改动
const float origin_GPS_lat=0;
const float origin_GPS_long=0;
//------设置初始xyz  全局变量------
extern double x_vehicle;
extern double y_vehicle;
extern double th_vehicle;
extern ros::Time last_time;

class rtkControl
{
public:
	rtkControl();
	~rtkControl();
	
	void run();
	void odometry(unsigned int l_wheel, unsigned int r_wheel, int steering, const ros::Time &cur_time);
	int wheelspeed_callback(const yhs_can_msgs::lr_wheel_fbConstPtr &lrwheel_fb, const yhs_can_msgs::rr_wheel_fbConstPtr &rrwheel_fb, const yhs_can_msgs::ctrl_fbConstPtr &ctrl_fb);
	typedef message_filters::sync_policies::ApproximateTime<yhs_can_msgs::lr_wheel_fb, yhs_can_msgs::rr_wheel_fb, yhs_can_msgs::ctrl_fb> MySyncPolicy;

private:
	ros::NodeHandle nh_;

	ros::Publisher TIME_pub_;
    ros::Publisher Long_latitude_pub_;
	ros::Publisher Pose_pub_;
	ros::Publisher IMU_pub_;
	ros::Publisher odom_pub_;  //话题设置
    tf2_ros::TransformBroadcaster odom_broadcaster;

	ros::Subscriber Right_wheel_sub_;
	ros::Subscriber Left_wheel_sub_;
	ros::Subscriber Steering_sub_;
	ros::Subscriber Gear_sub_;

	message_filters::Subscriber<yhs_can_msgs::lr_wheel_fb> sub_lr_wheel_;
	message_filters::Subscriber<yhs_can_msgs::rr_wheel_fb> sub_rr_wheel_;
	message_filters::Subscriber<yhs_can_msgs::ctrl_fb> sub_ctrl_fb_;

	boost::mutex cmd_mutex_;

	unsigned char sendData_u_wheel_[8] = {0};

	int dev_handler_;
	can_frame recv_frames_[1];
	can_frame send_frames_[2];
	
	

	void recvData();


};





