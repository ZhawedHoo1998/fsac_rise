import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
import time

if __name__=="__main__":

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2448)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2048)
    cap.set(cv2.CAP_PROP_FPS, 60)

    bridge=CvBridge()
    rospy.init_node('camera_node', anonymous=True)
    image_pub=rospy.Publisher('/image_view/image_raw', Image, queue_size = 1)
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        #frame_1 = cv2.resize(frame, (640, 512))
        if ret: 
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            rate = rospy.Rate(25) # 10hz 

    cap.release()
    cv2.destroyAllWindows()
