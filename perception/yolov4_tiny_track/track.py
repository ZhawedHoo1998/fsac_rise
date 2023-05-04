import cv2
import time
import rospy
import numpy as np
from PIL import Image as imim
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from yolo import YOLO
from sort import *
##自定义消息类型导入
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes

##tf收听器
from tf2_ros import transform_listener
from geometry_msgs.msg import TransformStamped

def callback(data):
	scaling_factor = 0.5	
	global bridge,yolo,mot_tracker
	cv_img = bridge.imgmsg_to_cv2(data,'bgr8')
	frame = cv2.resize(cv_img,(960,540),interpolation=cv2.INTER_LINEAR)	
	fps = 0.0
	t1 = time.time()
	frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
	frame = imim.fromarray(np.uint8(frame))
	frame,bbox = yolo.detect_image(frame)
	tracking_result = mot_tracker.update(np.array(bbox))
	res = tracker_Res(tracking_result)
	print(res) # [top,left,bottom.right,score,distance]
	frame = np.array(frame)
	frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
	fps  = ( fps + (1./(time.time()-t1)) ) / 2
	print("fps= %.2f"%(fps))
	frame = cv2.putText(frame, "fps= %.2f"%(fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

	#发布检测数据
	##时间数据
	image_header_ = data.header
	result_publish_(tracking_result, image_header_)

	cv2.imshow("video",frame)
	cv2.waitKey(3)

def display():
	rospy.init_node('camera_display',anonymous=True)
	global bridge
	bridge = CvBridge()
	rospy.Subscriber('/image_view/image_raw',Image,callback)
	rospy.spin()

def result_publish_(tracking_result, image_header_):
	bounding_msgs = BoundingBoxes()
	bounding_msgs.header.stamp = ros.Time.now()
	bounding_boxes.image_header = image_header
	for i in tracking_result:
		msg = BoundingBox()
		msg.probability = 1.0
		msg.xmin = i[0]
		msg.ymin = i[1]
		msg.xmax = i[2]
		msg.ymax = i[3]
		msg.id = i[-1]
		bounding_msgs.bounding_boxes.append(msg)
	#信息发布
	rresult_pub_.publish(bounding_msgs)


if __name__ == '__main__':
	try:
		while True:
			global yolo,mot_tracker

			#发布器
			rresult_pub_ = rospy.Publisher('/tracking_result', BoundingBoxes, queue_size=10)

			mot_tracker = Sort()
			yolo = YOLO()
			display()
	except rospy.ROSInterruptException:
		pass



