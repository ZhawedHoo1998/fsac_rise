#
# @Author: Qihao Yao
# @Date: Fri Apr 08 2022
# @Last Modified by: DaiKai 
# @Last Modified time: Fri Apr 08 2022 11:01:46 AM
# @Comment: 启动各个子模块，间隔启动
#
#!/bin/bash
# sleep 10s


# # rqt
# {
# gnome-terminal -t "rqt" -x bash -c "rqt;exec bash"
# }&
 
# sleep 5s

# # rviz
# {
# gnome-terminal -t "rviz" -x bash -c "rviz;exec bash"
# }&
 

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
rosclean purge -y
