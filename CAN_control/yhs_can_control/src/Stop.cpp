/*
    需求: 小车1m/s直行，遇见障碍物停车并开启双闪
*/
#include"stdio.h"
#include "ros/ros.h"
#include"std_msgs/Float32.h"
#include"std_msgs/UInt8.h"
#include"std_msgs/Bool.h"
#include"std_msgs/String.h"
#include"std_msgs/Int32.h"
#include"yhs_can_msgs/ctrl_cmd.h"
#include"yhs_can_msgs/io_cmd.h"
#include"yhs_can_control.h"
#include"string.h"
#include <string>

using namespace std;
int stop_sig_bool=0;

void detection_CallBack(std_msgs::String Stop_sig){
    string a="1";
    if(a==Stop_sig.data)
    {
        stop_sig_bool=1;
    }
    else{
        stop_sig_bool=0;
    }
    std::cout<<"test_1\n"<<std::endl;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"Stop");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    // ros::Publisher pub = nh.advertise<std_msgs::Float32>("ctrl_cmd",1000);
    ros::Publisher Go_ctrl=nh.advertise<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5);
    ros::Publisher Go_io=nh.advertise<yhs_can_msgs::io_cmd>("io_cmd", 5);
    ros::Subscriber Stop_Sig =nh.subscribe<std_msgs::String>("detection",5, detection_CallBack);
    
    //4.组织被发布的消息，编写发布逻辑并发布消息
        //4.1前进信号
    yhs_can_msgs::ctrl_cmd go;
    go.ctrl_cmd_gear = 4;
    go.ctrl_cmd_velocity = 0.2;
    go.ctrl_cmd_steering = 0;
    go.ctrl_cmd_Brake = 0;
  
        //4.2停车信号
    yhs_can_msgs::ctrl_cmd stop;
    stop.ctrl_cmd_gear = 1 ;
    stop.ctrl_cmd_velocity = 0;
    stop.ctrl_cmd_steering = 0;
    stop.ctrl_cmd_Brake = 0;  
        //4.3打开双闪信号
    yhs_can_msgs::io_cmd stop_sig;
    stop_sig.io_cmd_enable =0 ;
    stop_sig.io_cmd_upper_beam_headlamp= 0;
    stop_sig.io_cmd_turn_lamp= 1;
    stop_sig.io_cmd_turn_lamp= 2;
    stop_sig. io_cmd_speaker=0 ;

    ros::Rate loop(100);

    while (ros::ok())
    {
/*
        if (Stop_Sig.Subscriber!=1)
        {
            Go.publish(go);
            ROS_INFO("档位:%g,速度%dm/s,转角0rad/s,制动%.2f", zhixing .ctrl_cmd_gear,zhixing.ctrl_cmd_velocity,zhixing.ctrl_cmd_steering,zhixing.ctrl_cmd_Brake);
        }
*/

        if (stop_sig_bool==1)
        {
            Go_ctrl.publish(stop);
            Go_io.publish(stop_sig);
            ROS_INFO("前方有障碍物，档位:%g,速度%dm/s",stop.ctrl_cmd_gear,stop.ctrl_cmd_velocity);
        }

        else{
            Go_ctrl.publish(go);
            ROS_INFO("档位:%g,速度%dm/s",go .ctrl_cmd_gear,go.ctrl_cmd_velocity);
        }
        
        loop.sleep();
        ros::spinOnce();
    }



    return 0;
}



