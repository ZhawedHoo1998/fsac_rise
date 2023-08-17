#!/bin/bash

# Sleep time between launching processes
SLEEP_INTERVAL=5

# Run rslidar
gnome-terminal --title "rslidar" -- bash -c "\
source /home/rise/shared_dir/zitox_autoware/drivers/devel/setup.bash; \
roslaunch rslidar_sdk start.launch; \
exec bash" &
sleep $SLEEP_INTERVAL

# Control node
source /home/rise/fsac/devel/setup.bash
gnome-terminal --title "control" -- bash -c "\
roslaunch fsac_can_control fsac_can_control.launch; \
exec bash" &
sleep $SLEEP_INTERVAL

# Left camera
gnome-terminal --title "camera_left" -- bash -c "\
python /home/rise/fsac/src/fsac_rise/perception/camera/script/left_camera.py; \
exec bash" &
sleep 1

# Right camera
gnome-terminal --title "camera_right" -- bash -c "\
python /home/rise/fsac/src/fsac_rise/perception/camera/script/right_camera.py; \
exec bash" &
sleep 1

# Cone detection
gnome-terminal --title "camera_arg_launch" -- bash -c "\
source /home/rise/fsac/devel/setup.bash; \
roslaunch camera perception.launch; \
exec bash" &
sleep 2

# YOLOv4 tiny
gnome-terminal --title "yolov4_tiny" -- bash -c "\
conda activate yolov4_tiny; \
source /home/rise/fsac/devel/setup.bash; \
python /home/rise/fsac/src/fsac_rise/perception/camera/script/multi_camera_detection.py; \
exec bash" &
sleep 2

# Odometry
gnome-terminal --title "odometry" -- bash -c "\
source /home/rise/fsac/devel/setup.bash; \
roslaunch rtk rtk_start.launch; \
exec bash" &
sleep 2

# Global planning

# Local planning

# Tracking

# rqt
gnome-terminal --title "rqt" -- bash -c "\
rqt; \
exec bash" &
sleep $SLEEP_INTERVAL

# rviz
gnome-terminal --title "rviz" -- bash -c "\
rviz; \
exec bash" &
sleep $SLEEP_INTERVAL

# Clean up
rosclean purge -y