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
from darknet_ros_msgs.msg import map
from darknet_ros_msgs.msg import maps
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
	print(res,color) # [x,y,id,classes]
	frame = np.array(frame)
	frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
	fps  = ( fps + (1./(time.time()-t1)) ) / 2
	print("fps= %.2f"%(fps))
	frame = cv2.putText(frame, "fps= %.2f"%(fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
	#发布检测数据
	##时间数据
	image_header_ = data.header
	result_publish_(res, image_header_)
	cv2.imshow("video",frame)
	cv2.waitKey(3)


	

def result_publish_(tracking_result, image_header_):
	#监听信息应该有RT矩阵
	maps_msg = maps()
	maps_msg.header.stamp = rospy.Time.now()
	maps_msg.image_header = image_header_
	for i in tracking_result:
		map_msg = map()
		#新建消息 
		map_msg.x = float(i[0])
		map_msg.y = float(i[1])
		map_msg.id = float(i[-1])
		maps_msg.maps.append( map_msg )
		#将检测数据转换为
	#信息发布
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
	result_pub_ = rospy.Publisher('/tracking_result', maps, queue_size=10)
	#检测可视化
	rospy.spin()





	
	
