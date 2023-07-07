#
# @Author: Qihao Yao
# @Date: Fri Apr 08 2022
# @Last Modified by: DaiKai 
# @Last Modified time: Fri Apr 08 2022 11:01:46 AM
# @Comment: 启动各个子模块，间隔启动
#
#!/bin/bash
# sleep 10s




#运行雷达
source /home/rise/shared_dir/zitox_autoware/drivers/devel/setup.bash
{
gnome-terminal -t "rslidar" -x bash -c "roslaunch rslidar_sdk start.launch;exec bash"
}&

sleep 5s

# 控制节点
source /home/rise/fsac/devel/setup.bash
{
gnome-terminal -t "control" -x bash -c "roslaunch fsac_can_control fsac_can_control.launch;exec bash"
}&
 
sleep 5s

# 左相机
{
gnome-terminal -t "camera_left" -x bash -c "python /home/rise/fsac/src/fsac_rise/perception/camera/script/left_camera.py;exec bash"
}&
 
sleep 1s

# 右相机
{
gnome-terminal -t "camera_right" -x bash -c "python /home/rise/fsac/src/fsac_rise/perception/camera/script/right_camera.py;exec bash"
}&
 
sleep 1s

# 锥桐检测


# 里程计


# 全局规划

# 局部规划

# 跟踪


# rqt
{
gnome-terminal -t "rqt" -x bash -c "rqt;exec bash"
}&
 
sleep 5s

# rviz
{
gnome-terminal -t "rviz" -x bash -c "rviz;exec bash"
}&
 
sleep 5s

rosclean purge -y