#include <ros/ros.h>
#include "line_detector.hpp"
#include <sstream>
#include <Eigen/Dense>

namespace ns_line_detector {
// Constructor
LineDetector::LineDetector(ros::NodeHandle &nh) : nh_(nh) {
    if (!nh.param<double>("path_length", path_length, 80)) {
        ROS_WARN_STREAM("Did not load path_length. Standard value is: " << path_length);
    }
}

// Getters
geometry_msgs::Point LineDetector::getendPoint() { return end_point; }

// Setters
void LineDetector::setlidarCluster(fsd_common_msgs::Map msgs) {
    
    //设置tf监听器
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    // 获取从 odom 坐标系到 camera 坐标系的变换关系
    //transformStamped = tfBuffer.lookupTransform("camera", "odom", ros::Time(0));
    //camera_odom_x = transformStamped.transform.translation.x;
    //camera_odom_y = transformStamped.transform.translation.y;
    local_map = msgs;
    
}

void LineDetector::runAlgorithm() {
    if(!getPath)
        createPath();
    else
        return;
}

// 路径计算函数
void LineDetector::createPath() {
    if(local_map.cone_red.size() == 0 || local_map.cone_blue.size() == 0 )
        std::cout<<"test_1"<<std::endl;
        return;     //这里的判断应该用或吧 ??,黄色锥桶在这里不用做判断吧？？
    
    //对红色锥桶进行直线拟合
    int n_red = local_map.cone_red.size();  
    Eigen::MatrixXf X_RED(n_red, 2);
    Eigen::VectorXf Y_RED(n_red);
    for(int i=0; i < n_red; i++){

        X_RED(i, 0) = local_map.cone_red[i].position.x;
        X_RED(i, 0) = 1.0;
        Y_RED(i) = local_map.cone_red[i].position.y;
    }
    Eigen::Vector2f params_red = (X_RED.transpose() * X_RED).inverse() * X_RED.transpose() * Y_RED;
    float k_red = params_red(0);
    float b_red = params_red(1);

    //对蓝色锥桶进行直线拟合
    int n_blue = local_map.cone_blue.size();    
    Eigen::MatrixXf X_BLUE(n_blue, 2);
    Eigen::VectorXf Y_BLUE(n_blue);
    for(int i=0; i < n_blue; i++){

        X_BLUE(i, 0) = local_map.cone_blue[i].position.x;
        X_BLUE(i, 0) = 1.0;
        Y_BLUE(i) = local_map.cone_blue[i].position.y;
    }
    Eigen::Vector2f params_blue = (X_BLUE.transpose() * X_BLUE).inverse() * X_BLUE.transpose() * Y_BLUE;
    float k_blue = params_blue(0);
    float b_blue = params_blue(1);

    //求两直线的交点的x坐标
    float intersection_x = (b_red - b_blue)/(k_blue - k_red);

    //进行判断，两种情况：1.两直线斜率相等，2.两直线斜率不相等
    if (k_blue = k_red)
    {
        if  (abs(b_blue) < 2.8 && abs(b_red) < 2.8)        
        {
            getPath=true;
            std::cout<<"find ideal path"<<std::endl;
        }
    }
    else
    {
        //保证直线尽可能是垂直于y轴的
        if ((intersection_x < -180 || intersection_x > 180) && (abs(b_blue) < 2.8 && abs(b_red) < 2.8))  
        {
            getPath=true;
            std::cout<<"find ideal path"<<std::endl;
        }
        else
        {
            getPath=false;
            return;
        }
    }
    
    std::cout<<"test_2"<<std::endl;


    float y_red = k_red * path_length + b_red;   // 计算红色锥桶一边终点处的横坐标
    float y_blue  = k_blue * path_length + b_blue;  // 计算蓝色锥桶一边终点处的横坐标

    end_point.x = path_length;
    end_point.y = (y_red + y_blue)/2;       //终点的横纵坐标

}

}