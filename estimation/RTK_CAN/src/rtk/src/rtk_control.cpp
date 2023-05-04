#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>
#include <iomanip>
#include <iostream>
#include <bitset>
// https://blog.csdn.net/weixin_42268975/article/details/106021808
#include <rtk_msgs/TIME_RTK.h>
#include <rtk_msgs/long_latitude.h>
#include <rtk_msgs/pose.h>
#include <rtk_msgs/IMU.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <rtk/rtk_control.h>

#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
// #include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
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

#include <message_filters/time_synchronizer.h>
#include <ros/time.h>

#include <can_odometry_core.h>

#include <tf2_ros>


double x_vehicle = 0;
double y_vehicle = 0;
double th_vehicle = 0;
ros::Time last_time; 


rtkControl::rtkControl()
{
	ros::NodeHandle private_node("~");
	
}
rtkControl::~rtkControl()
{

}
void rtkControl::recvData()
{
	//
	while(ros::ok())
		{
			if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
			{
				for(int j=0; j<1; j++)
				{
					rtk_msgs::long_latitude lo_la_msg;
					
					
					//define odometry message




					switch (recv_frames_[0].can_id)
					{
					case 0x320:
					{
						rtk_msgs::TIME_RTK msg;
						msg.WeekTime = recv_frames_[0].data[0] << 8 | recv_frames_[0].data[1];
						// std::cout << "WeekTime: "<<msg.WeekTime <<std::endl;
						msg.GpsTime =(int)(((recv_frames_[0].data[5])<< 24) | ((recv_frames_[0].data[4])<< 16) | ((recv_frames_[0].data[3])<< 8) | (recv_frames_[0].data[2])) * 0.001;
						//std::cout <<" recv_frames_[0]数据类型为：" << typeid( recv_frames_[0] ).name() << std::endl;  //  打印: int 
						//std::cout <<" recv_frames_[0].data[3]数据为：" << std::hex << recv_frames_[0].data[3] << std::endl;  //  打印: int 
						// 0xff & recv_frames_[0].data[5] )<< 24  |  (0xff & recv_frames_[0].data[4] )<< 16 |  (0xff & recv_frames_[0].data[3] )<< 8 | 
						std::cout << "GpsweekTime: "<<msg.WeekTime <<std::endl;
						std::cout << "GpsTime: "<<msg.GpsTime <<std::endl;
						TIME_pub_.publish(msg);
						break;
					}
					case 0x321:
					{
						
						break;
					}
					case 0x32D:
					{
						lo_la_msg.PosLon = (int)((recv_frames_[0].data[7] << 56) | (recv_frames_[0].data[6] << 48) | (recv_frames_[0].data[5] << 40) | (recv_frames_[0].data[4] << 32) | (recv_frames_[0].data[3] << 24) | (recv_frames_[0].data[2] << 16) | (recv_frames_[0].data[1] << 8) | recv_frames_[0].data[0]) / 100000000 ;
						break;
					}	
					case 0x32E:
					{
						lo_la_msg.PosLat = (int)((recv_frames_[0].data[7] << 56) | (recv_frames_[0].data[6] << 48) | (recv_frames_[0].data[5] << 40) | (recv_frames_[0].data[4] << 32) | (recv_frames_[0].data[3] << 24) | (recv_frames_[0].data[2] << 16) | (recv_frames_[0].data[1] << 8) | recv_frames_[0].data[0]) / 100000000 ;
						break;
					}
					case 0x32A:
					{
						rtk_msgs::pose msg;
						msg.Heading = (int)(( ((recv_frames_[0].data[1] )<< 8)  | recv_frames_[0].data[0] )) * 0.01 ;
						msg.Pitch = ((float)((short)(((recv_frames_[0].data[3] )<< 8)  | (recv_frames_[0].data[2]) ))) *0.01 ;
						msg.Roll = (float)((short)(((recv_frames_[0].data[5] )<< 8)  | (recv_frames_[0].data[4]) )) *0.01 ;
						// std::cout << "Heading : "<<msg.Heading <<std::endl;
						// std::cout << "Pitch : "<<msg.Pitch <<std::endl;
						// std::cout << "Roll : "<<msg.Roll <<std::endl;
						Pose_pub_.publish(msg);
						break;
					}
					case 0x322:
					{
						rtk_msgs::IMU msg;
						msg.AccelRawX = (int)( ((0x0f & recv_frames_[0].data[2] )<< 16) | (( recv_frames_[0].data[1] )<< 8) | (recv_frames_[0].data[0] ) ) * 0.0001 ;
						std::cout << "X: "<<msg.AccelRawX <<std::endl;
						// msg.AccelRawY = (int)( (0x0f & recv_frames_[0].data[2] ) >>  | (0xff & recv_frames_[0].data[1] )<< 8 | ( 0xff & recv_frames_[0].data[0] ) ) / 10000 ;
						// msg.AccelRawZ = (int)( (0x0f & recv_frames_[0].data[2] )<< 16 | (0xff & recv_frames_[0].data[1] )<< 8 | ( 0xff & recv_frames_[0].data[0] ) ) / 10000 ;
						// std::cout << "AccelRawX : "<<msg.AccelRawX <<std::endl;
						// std::cout << "Pitch : "<<msg.Heading <<std::endl;
						// std::cout << "Roll : "<<msg.Heading <<std::endl;
						// IMU_pub_.publish(msg);
						break;
					}
						break;
					default:
						Long_latitude_pub_.publish(lo_la_msg);
						break;
					}
				}	
			}
		}
}
void rtkControl::run()
{

	//创建发布者对象
	TIME_pub_ = nh_.advertise<rtk_msgs::TIME_RTK>("time_rtk",5);
	Long_latitude_pub_ = nh_.advertise<rtk_msgs::long_latitude>("long_latitude_rtk",5);
	Pose_pub_ = nh_.advertise<rtk_msgs::pose>("pose_rtk",5);
	IMU_pub_ = nh_.advertise<rtk_msgs::IMU>("IMU_rtk",5);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odomstry", 10);

	//订阅轮速等多个话题信息 并送入同步的回调函数

	// 需要用message_filter容器对两个话题的数据发布进行初始化，这里不能指定回调函数
	sub_lr_wheel_.subscribe(nh_, "/lr_wheel_fb" ,50 , ros::TransportHints().tcpNoDelay());
	sub_rr_wheel_.subscribe(nh_, "/rr_wheel_fb" ,50 , ros::TransportHints().tcpNoDelay());
	sub_ctrl_fb_.subscribe(nh_, "/ctrl_fb" ,50 , ros::TransportHints().tcpNoDelay());
  		
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), sub_lr_wheel_, sub_rr_wheel_, sub_ctrl_fb_);
    sync.registerCallback(boost::bind(&rtkControl::wheelspeed_callback, this, _1, _2, _3));
	
	
	//打开设备
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dev_handler_ < 0) 
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}
    else
	{
		ROS_INFO(">>open can deivce success!");
	}

	struct ifreq ifr;
	std::string can_name("can0");
	strcpy(ifr.ifr_name,can_name.c_str());
	ioctl(dev_handler_,SIOCGIFINDEX, &ifr);

    // bind socket to network interface
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
	if (ret < 0) 
	{
		ROS_ERROR(">>bind   error!\r\n");
		return;
	}

	//创建接收发送数据线程
	boost::thread recvdata_thread(boost::bind(&rtkControl::recvData, this));
	// //	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));
	ros::spin();  //对于spin的理解
	close(dev_handler_);
}
int rtkControl::wheelspeed_callback(const yhs_can_msgs::lr_wheel_fbConstPtr &lrwheel_fb, const yhs_can_msgs::rr_wheel_fbConstPtr &rrwheel_fb, const yhs_can_msgs::ctrl_fbConstPtr &ctrl_fb)
{
	unsigned int l_wheel = lrwheel_fb->lr_wheel_fb_velocity * 3.6 * 10;
	unsigned int r_wheel = rrwheel_fb->rr_wheel_fb_velocity * 3.6 * 10; 
	int steering = ctrl_fb->ctrl_fb_steering * 10;


	//------里程计计算及tf发布----------
	odometry(l_wheel, r_wheel, steering, ctrl_fb->header.stamp);


	//---------------------向RTK发送挡位及轮速can信息-------------------------
	unsigned int gear = ctrl_fb->ctrl_fb_gear;
	// debug  std::cout << "成功接收消息" << std::endl;
	//！！档位需要修改  RTK的档位说明和底盘车说明不同
	if(gear == 0){
		return 0;}
	else if(gear == 1){
		gear = 3;}
	else if(gear == 3){
		gear = 0;}
	else if(gear == 4){
		gear = 1;}
	static unsigned char count_1 = 0;
	cmd_mutex_.lock();
	memset(sendData_u_wheel_,0,8);
	sendData_u_wheel_[0] = (sendData_u_wheel_[0] | ( 0xff & l_wheel ));  //比例系数！
	// std::cout<< "test_1:" << (std::bitset<32>)sendData_u_wheel_[1]<<std::endl;
	sendData_u_wheel_[1] = (sendData_u_wheel_[1] | ( 0xff & l_wheel >> 8 ));
	//std::cout<< "test_1:" << (std::bitset<8>)sendData_u_wheel_[0] << (std::bitset<8>)sendData_u_wheel_[1] <<std::endl;
	sendData_u_wheel_[2] = sendData_u_wheel_[2] | ( 0xff & r_wheel  );
	sendData_u_wheel_[3] = sendData_u_wheel_[3] | ( 0xff & r_wheel >> 8 );
	sendData_u_wheel_[4] = sendData_u_wheel_[4] | ( 0xff & steering  );
	sendData_u_wheel_[5] = sendData_u_wheel_[5] | ( 0xff & steering >> 8 );
	sendData_u_wheel_[6] = sendData_u_wheel_[6] | ( 0x0f & gear ); 
	//数据设置
	//设置信号crc等参数
	sendData_u_wheel_[7] = sendData_u_wheel_[0] ^ sendData_u_wheel_[1] ^ sendData_u_wheel_[2] ^ sendData_u_wheel_[3] ^ sendData_u_wheel_[4] ^ sendData_u_wheel_[5] ^ sendData_u_wheel_[6];
	send_frames_[0].can_id = 0x334;
    send_frames_[0].can_dlc = 8;
	memcpy(send_frames_[0].data, sendData_u_wheel_, 8);
	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	
	cmd_mutex_.unlock();
	//----------------------------------------------------------------------------
}
//主函数
void rtkControl::odometry(unsigned int l_wheel, unsigned int r_wheel, int steering, const ros::Time &cur_time){
	// ROS_INFO("LAST_TIME: %f",last_time.sec);
	// std::cout << "LAST_TIME: "<<last_time.sec <<std::endl;
	if ( last_time.sec == 0 && last_time.nsec == 0)
	{
		//-----当程序刚启动时，获取初始GPS数据-----

		//--------------------------------------
	}

	double dt = ( cur_time - last_time ).toSec();
	//------设置时间-------
	//设置时间戳为当前时间 并 设置为last_time 
	last_time = cur_time;
	std::cout << "LAST_TIME: "<<last_time.sec <<std::endl;

	double v = ( l_wheel + r_wheel ) / 2;

	float beta;
	if(abs(steering)>=0.01)
		beta=atan(( 645 /( 645 + 765 ))*tan(steering*(M_PI/180)));//msg.steering_angle 弧度or 角度  这里需要的弧度
	else
		beta=0.0;
	if( v >= 1e-5 )
	{
		//需要确定车辆横摆角 该横摆角指的是 车辆坐标系 在 惯性坐标系下的角度

		float dx = v*cos(odom_out.theta+beta);
		float dy = v*sin(odom_out.theta+beta);
		float d_th = ( v / 765 )*sin(beta)*angle_scale_;

		odom_out.x+=odom_out.v_x*dt;
		odom_out.y+=odom_out.v_y*dt;
		odom_out.theta+=odom_out.v_theta*dt;
	}

	//发布里程计数据 进行四元数转换
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw (odom_out.theta);
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = cur_time ;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = odom_out.x;
	odom_trans.transform.translation.y = odom_out.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	ROS_INFO("SUCCEED");


	//set the position
	odom.pose.pose.position.x = odom_out.x;
	odom.pose.pose.position.y = odom_out.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom.twist.twist.linear.x =   odom_out.v_x;
	odom.twist.twist.linear.y =   odom_out.v_y;
	odom.twist.twist.angular.z =  odom_out.v_theta;
	odom_pub_.publish(odom);


	
}
int main(int argc, char ** argv)
{
	//初始化ros节点，并命名
	ros::init(argc, argv, "rtkControl_node");

	//创建节点句柄
	rtkControl cancontrol;
	
	cancontrol.run();

	return 0;
}
