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

#include <chrono>

#include <bitset> // 包含 bitset 函数
namespace yhs_tool {

//消息全局变量
yhs_can_msgs::vehicle_status vehicle_msg;
sensor_msgs::Imu imu_msgs;
int count_imu;

CanControl::CanControl()
{
	ros::NodeHandle private_node("~");
	
}

CanControl::~CanControl()
{

}



//io控制回调函数
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;

	cmd_mutex_.lock();

	memset(sendData_u_io_,0,8);

	sendData_u_io_[0] = msg.io_cmd_enable;

	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	sendData_u_io_[2] = msg.io_cmd_speaker;

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0; 

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io_[6] =  count_1 << 4;

	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	send_frames_[0].can_id = 0x98C4D7D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_io_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, io error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}

//速度控制回调函数
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	unsigned short vel = 0; 
	short angular = msg.ctrl_cmd_steering * 100; //转角计算公式
	static unsigned char count = 0; //计数器

	cmd_mutex_.lock();

	memset(sendData_u_vel_,0,8);

	if(msg.ctrl_cmd_velocity < 0) vel = 0;
	else
	{
		vel = msg.ctrl_cmd_velocity * 1000;
	}

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendData_u_vel_[1] = (vel >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (vel >> 12));

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((msg.ctrl_cmd_Brake & 0x0f) << 4));

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
      ROS_ERROR("send message failed, cmd error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}


void CanControl::autoware_ctrl_cmdCallBack(const geometry_msgs::TwistStamped msg)
{
	
	
	unsigned short vel = 0; 
	short angular = -(msg.twist.angular.z * 180) / 3.141596 * 100; //转角计算公式
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
	if( msg.twist.linear.x < 0 ) vel = 0;
	else
	{
		vel = msg.twist.linear.x * 1000  ;
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


			// std::cout << "111111!!!!!!" << std::endl;

			for(int j=0;j<1;j++)
			{
			
				switch (recv_frames_[0].can_id)
				{

					//速度控制反馈
					case 0x98C4D2EF:
					{
						yhs_can_msgs::ctrl_fb msg;
						msg.header.seq = 1;
						msg.ctrl_fb_gear = vehicle_msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
						msg.ctrl_fb_velocity = vehicle_msg.ctrl_fb_velocity = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;					
						if ( msg.ctrl_fb_gear == 2 )
						{
							msg.ctrl_fb_velocity = msg.ctrl_fb_velocity * -1;
							vehicle_msg.ctrl_fb_velocity =vehicle_msg.ctrl_fb_velocity * -1;
						}
						msg.ctrl_fb_steering = vehicle_msg.ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;
						msg.ctrl_fb_Brake = vehicle_msg.ctrl_fb_Brake = (recv_frames_[0].data[4] & 0x30) >> 4;
						
						msg.ctrl_fb_mode = vehicle_msg.ctrl_fb_mode = (recv_frames_[0].data[4] & 0xc0) >> 6;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
							msg.header.frame_id = "frame";  
							msg.header.seq++;  
							msg.header.stamp = ros::Time::now();  
							ctrl_fb_pub_.publish(msg);
							// std::cout<<"success"<<std::endl;
						}

						break;
					}

					//左轮反馈
					case 0x98C4D7EF:
					{
						yhs_can_msgs::lr_wheel_fb msg;
						msg.header.seq = 1;
						msg.lr_wheel_fb_velocity = vehicle_msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

						msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							msg.header.frame_id = "frame";  
							msg.header.seq++;  
							msg.header.stamp = ros::Time::now();  
							lr_wheel_fb_pub_.publish(msg);
							
						}

						break;
					}

					//右轮反馈
					case 0x98C4D8EF:
					{
						yhs_can_msgs::rr_wheel_fb msg;
						msg.header.seq = 1;
						msg.rr_wheel_fb_velocity = vehicle_msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							msg.header.frame_id = "frame";  
							msg.header.seq++;  
							msg.header.stamp = ros::Time::now();  
							rr_wheel_fb_pub_.publish(msg);
							
						}

						break;
					}

					//io反馈
					case 0x98C4DAEF:
					{
						yhs_can_msgs::io_fb msg;

						if(0x01 & recv_frames_[0].data[0]) msg.io_fb_enable = true;	else msg.io_fb_enable = false;
	
						if(0x02 & recv_frames_[0].data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

						msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

						if(0x10 & recv_frames_[0].data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

						if(0x01 & recv_frames_[0].data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

						if(0x02 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						if(0x10 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					//里程计反馈
					case 0x98C4DEEF:
					{
						yhs_can_msgs::odo_fb msg;
						msg.odo_fb_accumulative_mileage = (float)((int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

						msg.odo_fb_accumulative_angular = (float)((int)(recv_frames_[0].data[7] << 24 | recv_frames_[0].data[6] << 16 | recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 1000;

						odo_fb_pub_.publish(msg);

						break;
					}

					//bms_Infor反馈
					case 0x98C4E1EF:
					{
						yhs_can_msgs::bms_Infor_fb msg;
						msg.bms_Infor_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.bms_Infor_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						msg.bms_Infor_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//bms_flag_Infor反馈
					case 0x98C4E2EF:
					{
						yhs_can_msgs::bms_flag_Infor_fb msg;
						msg.bms_flag_Infor_soc = recv_frames_[0].data[0];

						if(0x01 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_ov = true;	else msg.bms_flag_Infor_single_ov = false;

						if(0x02 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_uv = true;	else msg.bms_flag_Infor_single_uv = false;

						if(0x04 & recv_frames_[0].data[1]) msg.bms_flag_Infor_ov = true;	else msg.bms_flag_Infor_ov = false;

						if(0x08 & recv_frames_[0].data[1]) msg.bms_flag_Infor_uv = true;	else msg.bms_flag_Infor_uv = false;

						if(0x10 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ot = true;	else msg.bms_flag_Infor_charge_ot = false;

						if(0x20 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ut = true;	else msg.bms_flag_Infor_charge_ut = false;

						if(0x40 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ot = true;	else msg.bms_flag_Infor_discharge_ot = false;

						if(0x80 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ut = true;	else msg.bms_flag_Infor_discharge_ut = false;

						if(0x01 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_oc = true;	else msg.bms_flag_Infor_charge_oc = false;

						if(0x02 & recv_frames_[0].data[2]) msg.bms_flag_Infor_discharge_oc = true;	else msg.bms_flag_Infor_discharge_oc = false;

						if(0x04 & recv_frames_[0].data[2]) msg.bms_flag_Infor_short = true;	else msg.bms_flag_Infor_short = false;

						if(0x08 & recv_frames_[0].data[2]) msg.bms_flag_Infor_ic_error = true;	else msg.bms_flag_Infor_ic_error = false;

						if(0x10 & recv_frames_[0].data[2]) msg.bms_flag_Infor_lock_mos = true;	else msg.bms_flag_Infor_lock_mos = false;

						if(0x20 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_flag = true;	else msg.bms_flag_Infor_charge_flag = false;

						msg.bms_flag_Infor_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

						msg.bms_flag_Infor_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_flag_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//Drive_fb_MCUEcoder反馈
					case 0x98C4DCEF:
					{
						yhs_can_msgs::Drive_MCUEcoder_fb msg;
						
						msg.Drive_fb_MCUEcoder = (int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]); 

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Drive_MCUEcoder_fb_pub_.publish(msg);
						}

						break;
					}

					//Veh_fb_Diag反馈
					case 0x98C4EAEF:
					{
						yhs_can_msgs::Veh_Diag_fb msg;
						msg.Veh_fb_FaultLevel = 0x0f & recv_frames_[0].data[0];

						if(0x10 & recv_frames_[0].data[0]) msg.Veh_fb_AutoCANCtrlCmd = true;	else msg.Veh_fb_AutoCANCtrlCmd = false;

						if(0x20 & recv_frames_[0].data[0]) msg.Veh_fb_AutoIOCANCmd = true;	else msg.Veh_fb_AutoIOCANCmd = false;

						if(0x01 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisOnline = true;	else msg.Veh_fb_EPSDisOnline = false;

						if(0x02 & recv_frames_[0].data[1]) msg.Veh_fb_EPSfault = true;	else msg.Veh_fb_EPSfault = false;

						if(0x04 & recv_frames_[0].data[1]) msg.Veh_fb_EPSMosfetOT = true;	else msg.Veh_fb_EPSMosfetOT = false;

						if(0x08 & recv_frames_[0].data[1]) msg.Veh_fb_EPSWarning = true;	else msg.Veh_fb_EPSWarning = false;

						if(0x10 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisWork = true;	else msg.Veh_fb_EPSDisWork = false;

						if(0x20 & recv_frames_[0].data[1]) msg.Veh_fb_EPSOverCurrent = true;	else msg.Veh_fb_EPSOverCurrent = false;

						
						
						if(0x10 & recv_frames_[0].data[2]) msg.Veh_fb_EHBecuFault = true;	else msg.Veh_fb_EHBecuFault = false;

						if(0x20 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisOnline = true;	else msg.Veh_fb_EHBDisOnline = false;

						if(0x40 & recv_frames_[0].data[2]) msg.Veh_fb_EHBWorkModelFault = true;	else msg.Veh_fb_EHBWorkModelFault = false;

						if(0x80 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisEn = true;	else msg.Veh_fb_EHBDisEn = false;


						if(0x01 & recv_frames_[0].data[3]) msg.Veh_fb_EHBAnguleFault = true;	else msg.Veh_fb_EHBAnguleFault = false;

						if(0x02 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOT = true;	else msg.Veh_fb_EHBOT = false;

						if(0x04 & recv_frames_[0].data[3]) msg.Veh_fb_EHBPowerFault = true;	else msg.Veh_fb_EHBPowerFault = false;

						if(0x08 & recv_frames_[0].data[3]) msg.Veh_fb_EHBsensorAbnomal = true;	else msg.Veh_fb_EHBsensorAbnomal = false;

						if(0x10 & recv_frames_[0].data[3]) msg.Veh_fb_EHBMotorFault = true;	else msg.Veh_fb_EHBMotorFault = false;

						if(0x20 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilPressSensorFault = true;	else msg.Veh_fb_EHBOilPressSensorFault = false;

						if(0x40 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilFault = true;	else msg.Veh_fb_EHBOilFault = false;




						if(0x01 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUDisOnline = true;	else msg.Veh_fb_DrvMCUDisOnline = false;

						if(0x02 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUOT = true;	else msg.Veh_fb_DrvMCUOT = false;

						if(0x04 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUOV = true;	else msg.Veh_fb_DrvMCUOV = false;

						if(0x08 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUUV = true;	else msg.Veh_fb_DrvMCUUV = false;

						if(0x10 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUShort = true;	else msg.Veh_fb_DrvMCUShort = false;

						if(0x20 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUScram = true;	else msg.Veh_fb_DrvMCUScram = false;

						if(0x40 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUHall = true;	else msg.Veh_fb_DrvMCUHall = false;

						if(0x80 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUMOSFEF = true;	else msg.Veh_fb_DrvMCUMOSFEF = false;


						if(0x10 & recv_frames_[0].data[5]) msg.Veh_fb_AUXBMSDisOnline = true;	else msg.Veh_fb_AUXBMSDisOnline = false;

						if(0x20 & recv_frames_[0].data[5]) msg.Veh_fb_AuxScram = true;	else msg.Veh_fb_AuxScram = false;

						if(0x40 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteClose = true;	else msg.Veh_fb_AuxRemoteClose = false;

						if(0x80 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteDisOnline = true;	else msg.Veh_fb_AuxRemoteDisOnline = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Veh_Diag_fb_pub_.publish(msg);
						}


						break;
					}

					case 0x32D:
					{
						vehicle_msg.poslon = (int)((recv_frames_[0].data[7] << 56) | (recv_frames_[0].data[6] << 48) | (recv_frames_[0].data[5] << 40) | (recv_frames_[0].data[4] << 32) | (recv_frames_[0].data[3] << 24) | (recv_frames_[0].data[2] << 16) | (recv_frames_[0].data[1] << 8) | recv_frames_[0].data[0]) / 100000000 ;
						break;
					}

					case 0x32E:
					{
						vehicle_msg.poslat = (int)((recv_frames_[0].data[7] << 56) | (recv_frames_[0].data[6] << 48) | (recv_frames_[0].data[5] << 40) | (recv_frames_[0].data[4] << 32) | (recv_frames_[0].data[3] << 24) | (recv_frames_[0].data[2] << 16) | (recv_frames_[0].data[1] << 8) | recv_frames_[0].data[0]) / 100000000 ;
						break;
					}

					case 0x32A:  //姿态角
					{
						vehicle_msg.heading = (int)(( ((recv_frames_[0].data[1] )<< 8)  | recv_frames_[0].data[0] )) * 0.01 ;
						vehicle_msg.pitch = ((float)((short)(((recv_frames_[0].data[3] )<< 8)  | (recv_frames_[0].data[2]) ))) *0.01 ;
						vehicle_msg.roll = (float)((short)(((recv_frames_[0].data[5] )<< 8)  | (recv_frames_[0].data[4]) )) *0.01 ;
						
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
		

		// vehicle_msg.header.frame_id = "frame";  
		// vehicle_msg.header.seq++;  
		// vehicle_msg.header.stamp = ros::Time::now();
		// vehicle_status_pub_.publish(vehicle_msg);

		if(count_imu == 3)
		{
			imu_msgs.header.stamp = ros::Time::now();
			imu_msgs.header.seq++;
			imu_msgs.header.frame_id = "imu";
			IMU_pub_2.publish(imu_msgs);
			// std::cout << "test!!!!!!" << std::endl;	
			count_imu = 0;
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

	//创建订阅对象
	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);

	//角速度
	autoware_cmd_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("twist_raw",10, &CanControl::autoware_ctrl_cmdCallBack, this);
	

	//创建发布者对象
	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb",5);
	lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb",5);
	rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb",5);
	odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb",5);
	bms_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor_fb>("bms_Infor_fb",5);
	bms_flag_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor_fb>("bms_flag_Infor_fb",5);
	Drive_MCUEcoder_fb_pub_ = nh_.advertise<yhs_can_msgs::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb",5);
	Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb",5);

	vehicle_status_pub_=nh_.advertise<yhs_can_msgs::vehicle_status>("Vehicle_status",5);
	IMU_pub_2 = nh_.advertise<sensor_msgs::Imu>("IMU",5);  //发布IMU信息

	
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
//	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));
	ROS_INFO("receive !");
	ros::spin();
	ROS_INFO("receive_2 !");
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
