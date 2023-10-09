#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>
#include <iostream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "yhs_can_control.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/Imu.h"
#include "autoware_msgs/VehicleCmd.h"
#include <chrono>

#include <bitset> // 包含 bitset 函数
namespace yhs_tool {

//消息全局变量
yhs_can_msgs::vehicle_status vehicle_msg;
sensor_msgs::Imu imu_msgs;
sensor_msgs::NavSatFix gps_msg;
int count_imu;
int count_vehicle_status;
int count_gps;

CanControl::CanControl()
{
	ros::NodeHandle private_node("~");
	
}

CanControl::~CanControl()
{

}


void CanControl::autoware_ctrl_cmdCallBack(const autoware_msgs::VehicleCmd& msg)
{
	
	unsigned short vel = msg.ctrl_cmd.linear_velocity;
	short angular = msg.ctrl_cmd.steering_angle; //转角计算公式
	static unsigned char count = 0; //计数器

	//  档位判断
	unsigned int ctrl_cmd_gear = 4;

	// switch ( v > 0.01 )
	// {
	// case (vel < 10):
	// ctrl_cmd_gear = 1;
	// case ():
	// 	ctrl_cmd_gear = 2;
	
	// 	break;
	
	// default:
	// 	break;
	// }

	//  制动命令判断
	int ctrl_cmd_Brake = 1;



	cmd_mutex_.lock();

	memset(sendData_u_vel_,0,8);
	// ROS_INFO("vel: %d", vel);
	if( msg.ctrl_cmd.linear_velocity < 0 ) vel = 0;
	else
	{
		vel = msg.ctrl_cmd.linear_velocity * 1000  ;
		// ROS_INFO("vel: %d", vel);
		// std::count<< vel <<std::endl;
	}

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & ctrl_cmd_gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendData_u_vel_[1] = (vel >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (vel >> 12));

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & (( ctrl_cmd_Brake & 0x0f) << 4));

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	sendData_u_vel_[5] = 0;

	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  count << 4;
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send_frames_[0].can_id = 0x98C4D2D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel_, 8);



	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, autoware error code: %d",ret);
	
    }	
	
	int num = (	sendData_u_vel_[7])  ;   //1010 1010
	std::bitset<20> binary(num); // 将num转换成8位二进制
	std::cout << binary << std::endl; // 输出二进制数

	cmd_mutex_.unlock();
}

//数据接收解析线程
void CanControl::recvData()
{

	while(ros::ok())
	{

		if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
		{

			for(int j=0;j<1;j++)
			{
			
				switch (recv_frames_[0].can_id)
				{

					//速度控制反馈
					case 0x98C4D2EF:
					{
						geometry_msgs::TwistStamped autoware_status_msgs;
						autoware_status_msgs.header.stamp = ros::Time::now();
						autoware_status_msgs.header.seq++;
						autoware_status_msgs.header.frame_id = "base_link";

						vehicle_msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
						vehicle_msg.ctrl_fb_velocity = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;					
						autoware_status_msgs.twist.linear.x = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
						
						if ( vehicle_msg.ctrl_fb_gear == 2 )
						{
							vehicle_msg.ctrl_fb_velocity =vehicle_msg.ctrl_fb_velocity * -1;
						}

						vehicle_msg.ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;
						autoware_status_msgs.twist.angular.z = ((float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100) * M_PI / 180;
						vehicle_msg.ctrl_fb_Brake = (recv_frames_[0].data[4] & 0x30) >> 4;
						vehicle_msg.ctrl_fb_mode = (recv_frames_[0].data[4] & 0xc0) >> 6;
						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc != recv_frames_[0].data[7])
						{
							ROS_ERROR("vehicle_msg.ctrl_fb_velocity received error!");
						}
						else{
							count_vehicle_status++;
							autoware_status_pub_.publish(autoware_status_msgs);
							ROS_INFO("IT CAN BE CONVERTED!!!!!");
						}
						break;
					}

					
                    
                    case 0x32D: 
					{						
						gps_msg.latitude = (int)((recv_frames_[0].data[7] << 56) | (recv_frames_[0].data[6] << 48) | (recv_frames_[0].data[5] << 40) | (recv_frames_[0].data[4] << 32) | (recv_frames_[0].data[3] << 24) | (recv_frames_[0].data[2] << 16) | (recv_frames_[0].data[1] << 8) | recv_frames_[0].data[0]) / 100000000 ;
						count_gps++;
						break;
					}

					case 0x32E:
					{
						gps_msg.longitude = (int)((recv_frames_[0].data[7] << 56) | (recv_frames_[0].data[6] << 48) | (recv_frames_[0].data[5] << 40) | (recv_frames_[0].data[4] << 32) | (recv_frames_[0].data[3] << 24) | (recv_frames_[0].data[2] << 16) | (recv_frames_[0].data[1] << 8) | recv_frames_[0].data[0]) / 100000000 ;
						count_gps++;
						break;
					}


					case 0x32A:  //姿态角
					{
						// vehicle_msg.heading = (int)(( ((recv_frames_[0].data[1] )<< 8)  | recv_frames_[0].data[0] )) * 0.01 ;
						// vehicle_msg.pitch = ((float)((short)(((recv_frames_[0].data[3] )<< 8)  | (recv_frames_[0].data[2]) ))) *0.01 ;
						// vehicle_msg.roll = (float)((short)(((recv_frames_[0].data[5] )<< 8)  | (recv_frames_[0].data[4]) )) *0.01 ;
						
						uint16_t Yaw = ((int)(( ((recv_frames_[0].data[1] )<< 8)  | recv_frames_[0].data[0] ))) * 0.01 ;   //原始信号单位为度
						float Pitch = (((float)((short)(((recv_frames_[0].data[3] )<< 8)  | (recv_frames_[0].data[2]) ))) *0.01) -0.55  ;
						float Roll = ((float)((short)(((recv_frames_[0].data[5] )<< 8)  | (recv_frames_[0].data[4]) ))) *0.01 ;

						// std::cout << "Heading_angle : "<< Yaw << std::endl;
						// std::cout << "Pitch_angle : "<< Pitch << std::endl;
						// std::cout << "Roll_angle : "<< Roll << std::endl;

						Yaw = Yaw * M_PI / 180.0;   //将角度值转化为弧度 并转化为四元数
						Pitch = Pitch * M_PI / 180.0;
						Roll = Roll * M_PI / 180.0;

						// 将欧拉角转换为旋转矩阵
						tf2::Matrix3x3 mat;
						mat.setRPY(Roll, Pitch, Yaw);
						// 将旋转矩阵转换为四元数
						tf2::Quaternion quat;
						mat.getRotation(quat);

						geometry_msgs::Quaternion quat_msg;
						tf2::convert(quat, quat_msg); //
						// std::cout << "quat.x : " <<quat_msg.x <<std::endl;  //= Roll
						// std::cout << "quat.y : " <<quat_msg.y <<std::endl;  
						// std::cout << "quat.z : " <<quat_msg.z <<std::endl;  
						// std::cout << "quat.w : " <<quat_msg.w <<std::endl;  
						imu_msgs.orientation.x = quat_msg.x ;
						imu_msgs.orientation.y = quat_msg.y ;
						imu_msgs.orientation.z = quat_msg.z ;
						imu_msgs.orientation.w = quat_msg.w ;


						auto now = std::chrono::high_resolution_clock::now(); // 获取当前时间
						auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now); // 将当前时间转换为毫秒
						auto time = ms.time_since_epoch().count(); // 将时间转换为整数类型
						// std::cout << "Current time_0x32A: " << time << "ms" << std::endl; // 打印当前时间
						count_imu++;
						break;
					}
					





					case 0x32C: //三轴角速度
					{
						imu_msgs.angular_velocity.x =( ((int)(((0x0f & recv_frames_[0].data[2]) << 16) | (recv_frames_[0].data[1] << 8) | recv_frames_[0].data[0] )) * 0.01) - 0.52;
						imu_msgs.angular_velocity.y = ((int)(((recv_frames_[0].data[4] << 12)) | ((recv_frames_[0].data[3]) << 4) | ((0xf0 & recv_frames_[0].data[2]) >> 4))) * 0.01;
						imu_msgs.angular_velocity.z = (((int)((( 0x0f & recv_frames_[0].data[7]) << 16) | (recv_frames_[0].data[6] << 8) | recv_frames_[0].data[5] )) * 0.01) ;
						if (imu_msgs.angular_velocity.y > 1000.7){
							imu_msgs.angular_velocity.y =0; 
						}
						if (imu_msgs.angular_velocity.x > 1000.7){
							imu_msgs.angular_velocity.x =0; 
						}
						if (imu_msgs.angular_velocity.z > 1000.7){
							imu_msgs.angular_velocity.z =0; 
						}
						// int num = ((recv_frames_[0].data[3]) << 4)  ;   //1010 1010
						// int num_2 = recv_frames_[0].data[3]; 
						// int num_3 = ((recv_frames_[0].data[3]) << 4) | ((0xf0 & recv_frames_[0].data[2]) >> 4); 
						// std::bitset<20> binary(num); // 将num转换成8位二进制
						// std::bitset<20> binary_2(num_2); // 将num转换成8位二进制
						// std::bitset<20> binary_3(num_3); // 将num转换成8位二进制
						// // std::cout << 0xfff << std::endl; // 输出二进制数
						// std::cout << binary << std::endl; // 输出二进制数
						// std::cout << binary_2 << std::endl; // 输出二进制数
						// std::cout << binary_3 << std::endl; // 输出二进制数

						// std::cout << "imu_msgs.angular_velocity.x : "<<imu_msgs.angular_velocity.x <<std::endl;
						// std::cout << "imu_msgs.angular_velocity.y : "<<imu_msgs.angular_velocity.y <<std::endl;
						// std::cout << "imu_msgs.angular_velocity.z : "<<imu_msgs.angular_velocity.z <<std::endl;
						//
						auto now = std::chrono::high_resolution_clock::now(); // 获取当前时间
						auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now); // 将当前时间转换为毫秒
						auto time = ms.time_since_epoch().count(); // 将时间转换为整数类型
						// std::cout << "Current time_0x32C: " << time << "ms" << std::endl; // 打印当前时间
						count_imu++;
						break;
					}
					
					case 0x329://三轴加速度
					{
						imu_msgs.linear_acceleration.x = ((int)((( 0x0f & recv_frames_[0].data[2]) << 16) | (recv_frames_[0].data[1] << 8) | recv_frames_[0].data[0] )) * 0.0001;
						imu_msgs.linear_acceleration.y = ((int)(((recv_frames_[0].data[4] << 12)) | (recv_frames_[0].data[3] << 4) | ((0xf0 & recv_frames_[0].data[2]) >> 4))) * 0.0001;
						imu_msgs.linear_acceleration.z = ((int)((( 0x0f & recv_frames_[0].data[7]) << 16) | (recv_frames_[0].data[6] << 8) | recv_frames_[0].data[5] )) * 0.0001;
						// std::cout << "imu_msgs.linear_acceleration.x : "<<imu_msgs.linear_acceleration.x <<std::endl;
						// std::cout << "imu_msgs.linear_acceleration.y : "<<imu_msgs.linear_acceleration.y <<std::endl;
						// std::cout << "imu_msgs.linear_acceleration.z : "<<imu_msgs.linear_acceleration.z <<std::endl;
						count_imu++;
						break;
					}

					default:

						break;
						
				}

			}		
		}
		



		if(count_gps == 2)
		{
			gps_msg.header.stamp = ros::Time::now();
			gps_msg.header.seq++;
			gps_msg.header.frame_id = "GPS";
			gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
			GPS_pub_.publish(gps_msg);
			// std::cout << "test!!!!!!" << std::endl;	
			count_gps = 0;
		}




		if(count_imu == 3)
		{
			imu_msgs.header.stamp = ros::Time::now();
			imu_msgs.header.seq++;
			imu_msgs.header.frame_id = "imu";
			IMU_pub_.publish(imu_msgs);
			// std::cout << "test!!!!!!" << std::endl;	
			count_imu = 0;
		}
		
		if(count_vehicle_status == 4)
		{
			vehicle_msg.header.stamp = ros::Time::now();
			vehicle_msg.header.seq++;
			vehicle_msg.header.frame_id = "base_link";
			vehicle_status_pub_.publish(vehicle_msg);
			count_vehicle_status = 0;
		}

	}
}


//数据发送线程
//判断数据是否接受发送
void CanControl::sendData()
{
	ros::Rate loop(100);


	while(ros::ok())
	{

		loop.sleep();
	}

}



void CanControl::run()
{

	//角速度
	autoware_cmd_sub_ = nh_.subscribe("vehicle_cmd",10, &CanControl::autoware_ctrl_cmdCallBack, this);
	

	//创建发布者对象
	GPS_pub_ =nh_.advertise<sensor_msgs::NavSatFix>("GPS",5);

	vehicle_status_pub_=nh_.advertise<yhs_can_msgs::vehicle_status>("Vehicle_status",5);
	autoware_status_pub_=nh_.advertise<geometry_msgs::TwistStamped>("autoware_vehicle_status",5);
	IMU_pub_ = nh_.advertise<sensor_msgs::Imu>("IMU",5);  //发布IMU信息

	
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
		ROS_ERROR(">>bind dev_handler error!\r\n");
		return;
	}

	//创建接收发送数据线程
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
	ROS_INFO("receive !");
	ros::spin();
	close(dev_handler_);
}
}

//主函数
int main(int argc, char ** argv)
{
	//初始化ros节点，并命名
	ros::init(argc, argv, "yhs_can_control_node");

	//创建节点句柄
	yhs_tool::CanControl cancontrol;

	cancontrol.run();

	return 0;
}


