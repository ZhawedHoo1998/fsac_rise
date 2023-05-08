本功能包主要定义了底盘车各传感器的can通信的信号解析功能，主要分为：
1. RTK接入的can信号解析;
2. 底盘车can信号解析;
3. yhs_can_msgs文件夹内定义了vehicle_status等自定义消息数据；
4. config文件夹中 vehicle_basis_info.yaml 定义了车辆基本参数，如：轮距、轴距等参数，launch文件将该参数文件上传至ros公共参数库，日常代码可通过设置句柄直接获取车辆基本参数；
5. 读取或接受控制信号

话题发布：
    a. vehicle_status 该话题主要发布了车辆车速、bms基本电量信息、经纬度位置信息等信息，可通过订阅它获取车辆基本信息；
    b. IMU 
    C. GPS
    D. bms_flag_Infor_fb   bms详细信息
    e. Drive_MCUEcoder_fb  车辆故障码等基本信息
    f. Veh_Diag_fb

订阅：
    a. ctrl_cmd  常见控制信号
    b. io_cmd 
    c. twist_raw  用于autoware控制信号接受苏