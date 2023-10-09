#
# @Author: Qihao Yao
# @Date: Fri Apr 08 2022
# @Last Modified by: DaiKai 
# @Last Modified time: Fri Apr 08 2022 11:01:46 AM
# @Comment: 启动各个子模块，间隔启动
#
#!/bin/bash
# sleep 10s
source /home/autoware/zitox_ws/devel/setup.bash
echo '123456'| sudo chmod 777 /dev/ttyS0


# 控制节点
source /home/autoware/zitox_ws/devel/setup.bash
{
gnome-terminal -t "control" -x bash -c "roslaunch fsac_can_control fsac_can_control.launch;exec bash"
}&
 
sleep 5s


#运行雷达
source /home/autoware/zitox_ws/devel/setup.bash
{
gnome-terminal -t "rslidar" -x bash -c "roslaunch rslidar_sdk start.launch;exec bash"
}&

# RTK
source /home/autoware/zitox_ws/devel/setup.bash
{
gnome-terminal -t "point_transformer" -x bash -c "python /home/autoware/zitox_ws/src/script/topic_transmit.py;exec bash"
}&

sleep 5s

# RTK
source /home/autoware/zitox_ws/devel/setup.bash
{
gnome-terminal -t "gnss" -x bash -c "python /home/autoware/zitox_ws/src/script/GPS_serial.py;exec bash"
}&

sleep 5s

#经纬度转化
source /home/autoware/zitox_ws/devel/setup.bash
{
gnome-terminal -t "wgs_conversions" -x bash -c "roslaunch wgs_conversions convert.launch;exec bash"
}&
sleep 5s

# autoware 
source /home/autoware/Autoware/install/setup.bash
{
gnome-terminal -t "control" -x bash -c "roslaunch runtime_manager runtime_manager.launch;exec bash"
}&
 

sleep 5s
rosclean purge -y