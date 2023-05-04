# coding=UTF-8
# This Python file uses the following encoding: utf-8
import sys
import rospy

import cv2
import time

import numpy as np
from PIL import Image as imim
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

sys.path.append('/home/advan/yhs_ws/perception/src/yolov4_tiny_track')

from yolo import YOLO
from sort import *

##自定义消息类型导入
from fsd_common_msgs.msg import Cone
from fsd_common_msgs.msg import Map
##tf收听器



def callback(data):
	scaling_factor = 0.5	
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

	# num_all = len(res) # 获取数据长度
	# for i in num_all:
		
	frame = np.array(frame)
	frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
	fps  = ( fps + (1./(time.time()-t1)) ) / 2
	print("fps= %.2f"%(fps))
	frame = cv2.putText(frame, "fps= %.2f"%(fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
	#发布检测数据
	##时间数据
	image_header_ = data.header
	result_publish_(res, image_header_, color)
	# cv2.imshow("video",frame)
	# cv2.waitKey(3)


	

def result_publish_(tracking_result, image_header_, color):

	maps_msg = Map()
	maps_msg.header.stamp = rospy.Time.now()
	maps_msg.header.seq+=1
	maps_msg.header.frame_id = "camera" 
	#maps_msg.image_header = image_header_
	num_all = len(tracking_result) #获取锥桶总数

	for i in range(num_all):
		# 进行颜色判断 放入Map不同消息中
		if color[i] == "r":
			cone_msg = Cone()
			cone_msg.position.x = float(tracking_result[i][0])
			cone_msg.position.y = float(tracking_result[i][1])
			maps_msg.cone_red.append( cone_msg )
		elif color[i] == "b":
			cone_msg = Cone()
			cone_msg.position.x = tracking_result[i][0]
			cone_msg.position.y = tracking_result[i][1]
			maps_msg.cone_blue.append( cone_msg )
		elif color[i] == "y":
			cone_msg = Cone()
			cone_msg.position.x = tracking_result[i][0]
			cone_msg.position.y = tracking_result[i][1]
			maps_msg.cone_yellow.append( cone_msg )
	#将检测数据转换为信息发布
	# print(maps_msg)
	result_pub_.publish(maps_msg)




if __name__ == '__main__':

	global yolo,mot_tracker,bridge
	bridge = CvBridge()
	#sort跟踪器
	mot_tracker = Sort()
	#检测器
	yolo = YOLO()
	#新建节点
	rospy.init_node('camera_display',anonymous=True)
	rate = rospy.Rate(10)
	#发布器
	image_subscriber = rospy.Subscriber('/camera/image_color',Image,callback)
	result_pub_ = rospy.Publisher('/local_map', Map, queue_size=10)
	#检测可视化
	rospy.spin()





	
	
