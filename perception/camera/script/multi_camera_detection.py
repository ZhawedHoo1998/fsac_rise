#! /home/advan/.pyenv/versions/3.7.16/bin/python 
# coding=UTF-8
# This Python file uses the following encoding: utf-8

import rospy

import threading
import queue

import cv2
import time

import numpy as np
from PIL import Image as imim
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from message_filters import TimeSynchronizer, Subscriber

import os
import sys
# 获取当前脚本所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 将相对路径添加到当前目录中
relative_path = '../src/yolov4_tiny_track'
path = os.path.join(current_dir, relative_path)
sys.path.append(path)


from yolo import YOLO_ONNX
from sort import *

##自定义消息类型导入
from fsd_common_msgs.msg import Cone
from fsd_common_msgs.msg import Map
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Point 
##tf收听器

# 创建全局变量，用于存储检测结果
maps_msg = Map()
global_marker_array = Marker()

def callback(left_image_msg, right_image_msg):
    # 在回调函数中获取全局变量，并将同步信息放入队列
    global global_object_list, global_marker_array
    image_queue_left.put(left_image_msg)
    image_queue_right.put(right_image_msg)
    
def detection_worker(queue, maps_msg, global_marker_array):
	while True:
		# 获取列表中的信息
		data = queue.get()
		global bridge,yolo,mot_tracker
		cv_img = bridge.imgmsg_to_cv2(data,'bgr8')
		frame = cv2.resize(cv_img,(960,540),interpolation=cv2.INTER_LINEAR)	
		fps = 0.0
		t1 = time.time()
		frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
		frame = imim.fromarray(np.uint8(frame))
		frame,bbox,color = yolo.detect_image(frame)
		tracking_result = mot_tracker.update(np.array(bbox))
		res = tracker_Res(tracking_result)
		print(res,color) # [top,left,bottom.right,score,distance]

		frame = np.array(frame)
		frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
		fps  = ( fps + (1./(time.time()-t1)) ) / 2
		#print("fps= %.2f"%(fps))
		# frame = cv2.putText(frame, "fps= %.2f"%(fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) #在图像绘制 fps
		#发布检测数据
		maps_msg.header.stamp = rospy.Time.now()
		maps_msg.header.seq+=1
		num_all = len(res) #获取锥桶总数
		for i in range(num_all):
			# 进行颜色判断 放入Map不同消息中
			if color[i] == "r":
				cone_msg = Cone()
				cone_msg.position.x = float(res[i][0])
				cone_msg.position.y = float(res[i][1])
				maps_msg.cone_red.append( cone_msg )
			elif color[i] == "b":
				cone_msg = Cone()
				cone_msg.position.x = res[i][0]
				cone_msg.position.y = res[i][1]
				maps_msg.cone_blue.append( cone_msg )
			elif color[i] == "y":
				cone_msg = Cone()
				cone_msg.position.x = res[i][0]
				cone_msg.position.y = res[i][1]
				maps_msg.cone_yellow.append( cone_msg )



def main():
	global yolo,mot_tracker,bridge
	bridge = CvBridge()
	# sort跟踪器
	mot_tracker = Sort()
	# 检测器
	yolo = YOLO_ONNX()
	# 新建节点
	rospy.init_node('image_processing',anonymous=True)
	# 读取参数库中的参数
	camera_left_topic = rospy.get_param("camera_left_topic")
	camera_right_topic = rospy.get_param("camera_right_topic")

	frame_id = rospy.get_param("frame_id")

	maps_msg.header.frame_id = frame_id
	# 创建队列和线程
	image_queue_left = queue.Queue()
	image_queue_right = queue.Queue()

	#发布器
	image_subscriber_left = Subscriber(camera_left_topic,Image)
	image_subscriber_right = Subscriber(camera_right_topic,Image)
	ts = TimeSynchronizer([image_subscriber_left, image_subscriber_right], 10)
	ts.registerCallback(callback)
    
	# 创建发布者，用于发布检测地图及可视化结果
	result_pub_ = rospy.Publisher('/local_map', Map, queue_size=10)
	marker_publisher = rospy.Publisher('/visualization_marker_array', Marker, queue_size=10)

	# 创建队列和线程
    # 启动两个线程，分别处理左右相机的检测结果
	thread_left = threading.Thread(target=detection_worker, args=(image_queue_left, maps_msg, global_marker_array))
	thread_right = threading.Thread(target=detection_worker, args=(image_queue_right, maps_msg, global_marker_array))
	thread_left.start()
	thread_right.start()


     # 循环发布检测结果和可视化信息
	rate = rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
        # 发布全局变量中的检测结果
		maps_msg.header.stamp = rospy.Time.now()
		result_pub_.publish(maps_msg)

        # 发布全局变量中的可视化信息
		global_marker_array.header.stamp = rospy.Time.now()
		marker_publisher.publish(global_marker_array)

        # 清空全局变量中的检测结果和可视化信息，以便下一次循环使用
		maps_msg.cone_red = []
		maps_msg.cone_blue = []
		maps_msg.cone_yellow = []
		global_marker_array.points = []

        # 休眠一段时间，以控制发布频率
		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt detected. Shutting down...")
        rospy.signal_shutdown("Keyboard interrupt")