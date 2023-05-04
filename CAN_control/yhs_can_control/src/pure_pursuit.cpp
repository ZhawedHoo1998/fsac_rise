//纯跟踪算法

#include"ros/ros.h"
#include"yhs_can_control.h"
#include"ctrl_cmd.h"
#include"std_msgs/String.h"
#include <vector>

using namespace std;

//定义车辆参数
#define  K          (1.5)       // 前视距离系数
#define  L_VEHICLE  (2.9)       // 车的轴距
int L0 = 2;    //最小前视距离，根据道路曲率变化   

//定义车辆状态，组合惯导x y yaw，车辆速度speed
struct State{
  double x = 0;          // m
  double y = 0;          // m
  double yaw = 0;        // degree
  double speed = 0;      // m/s
            Go_ctrl.publish(go);
            ROS_INFO("档位:%g,速度%dm/s",go .ctrl_cmd_gear,go.ctrl_cmd_velocity);
        }
        
        loop.sleep();
};
State vehicleState;

//车辆状态回调函数
void vehicleState_CallBack(std_msgs::String Position_x, std_msgs::String Position_y,std_msgs::String Position_yaw,yhs_can_msgs::ctrl_fb vehicle_speed){
 vehicleState.x = Position_x;
 vehicleState.y = Position_y;
 vehicleState.yaw = Position_yaw;
 vehicleState.speed = vehicle_speed;

  //用vehLocUpdatRdy表示位姿是否更新，默认为false
  bool vehLocUpdatRdy = true;
}

//接收路径规划点并处理
int get_goal_index(std_msgs::String index_x,std_msgs::String index_y){
    
    index.x = index_x;
    index.y = index_y;
    



    
    vector<double> d;

    int index = 0;
    double d_min = d[0];
    int dVecLen = d.size();

    /*
    for (int i = 0; i < path.size(); i++)
            d.push_back(pow((s.x - path[i].x), 2) + pow((s.y - path[i].y), 2));//距离计算
    */
 
    //找到距离车辆最近的路径点
    for (int i = 0; i < dVecLen; i++) {
        if (d_min > d[i]) {
            d_min = d[i];
            index = i;
        }
    }
 
    double l = 0;
    double lf = K * s.speed + L0;
    double dx_, dy_;
 
    //积分法计算路径长度
    while (l < lf && index < path.size()) {
        dx_ = path[index + 1].x - path[index].x;
        dy_ = path[index + 1].y - path[index].y;
        l += sqrt(dx_ * dx_ + dy_ * dy_);
        index++;
    }
 
    return index;

}

//根据目标进行转角控制main
double pure_pursuit_control(const State &s, const vector <Point> &path, int *lastIndex) {
        int index = get_goal_index(x, y); // 搜索目标点，返回目标点的标签
 
        // 用上一个循环的目标点判断是否是在向前走
        if (index <= *lastIndex) {
            index = *lastIndex;
        }
 
        Point goal;
 
        //防止index溢出
        if (index < path.size()) {
            goal = path[index]; 
        } else {
            index = path.size() - 1;
            goal = path[index];
        }
 
        // 车身坐标系的x轴和目标点与车身坐标系原点连线的夹角
        double alpha = atan2(goal.y - s.y, goal.x - s.x) - s.yaw;
 
        if (s.speed < 0)
            alpha = M_PI - alpha;
 
        double lf = K * s.speed + L0; // 根据车速和道路曲率设置前视距离
        // delta 即为纯跟踪算法的最终输出
        double delta = atan2((2.0 * L_VEHICLE * sin(alpha)) / lf, 1.0);
 
        *lastIndex = index;      // 为下一个循环更新上一个目标点
        return delta;
}

//判断是否已经走完并发布消息
bool pure_pursuit::track_follow_process(const vector <Point> &pathVet) {
    bool isjobDone = false;
    
    //判断路径是否走完
    if (goalIndex < rearIndex) {

        //获取纯跟踪输出的前轮转角
        delta = pure_pursuit_control(vehicleState, pathVet, &goalIndex);

        wheelAngle = delta * 180 / M_PI;
        // wheelAngle = -1 * wheelAngle;  //前轮转角为弧度制，逆时针为正，AionLX车的顺时针转向为正，逆时针为负，刚好相反
 
        //AionLX前轮转角最大为40°
        if (wheelAngle > 40.0) wheelAngle = 40.0;
        if (wheelAngle < -40.0) wheelAngle = -40.0;

    }
    
    //如果走完，则急停
    if (goalIndex >= rearIndex)
        isjobDone = true;
    
    //发布前轮转角
    std_msgs::Float32 tempAngleSet;
    tempAngleSet.data = wheelAngle;
    strAngle_pub.publish(tempAngleSet);

    return isjobDone;
}


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"pure_pursuit");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //发布话题，车辆速度和转角，
    ros::Publisher Go_ctrl=nh.advertise<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5);

    //订阅车辆位置信息节点
    ros::Subscriber Position_x =nh.subscribe<std_msgs::String>("Position_x",5, vehicleState_CallBack);
    ros::Subscriber Position_y =nh.subscribe<std_msgs::String>("Position_y",5, vehicleState_CallBack);
    ros::Subscriber Position_yaw =nh.subscribe<std_msgs::String>("Position_yaw",5, vehicleState_CallBack);

    //订阅前方路径点节点
    ros::Subscriber index_x =nh.subscribe<std_msgs::String>("index_x",5, get_goal_index);
    ros::Subscriber index_y =nh.subscribe<std_msgs::String>("index_y",5, get_goal_index);

    //订阅车辆速度信息节点
    ros::Subscriber vehicle_speed = nh.subscribe<yhs_can_msgs::ctrl_fb>("ctrl_fb", 5,vehicleState_CallBack);

    
    //组织被发布的消息，编写发布逻辑并发布消息
    yhs_can_msgs::ctrl_cmd go;
    go.ctrl_cmd_gear = 4;
    go.ctrl_cmd_velocity = 0.2;
    go.ctrl_cmd_steering = wheelAngle;
    go.ctrl_cmd_Brake = 0;
  
    ros::Rate loop(100);

    while (ros::ok())
    {

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