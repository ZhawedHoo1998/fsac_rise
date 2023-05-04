# -*- coding: utf-8 -*
#!/usr/bin/env python
# license removed for brevity
# import numpy as np
import cv2
import random
import rospy
import torch
import time
import datetime

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from sort import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
# sys.path()
# 增加yolov5的功能包路径


#tf
# import tf

#自定义消息
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes



# 设置路径！！！
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
#torch.hub.load(repo_or_dir, model, *args, source='github', force_reload=False, verbose=True, skip_validation=False, **kwargs)
path= "/home/advan/yhs_ws/perception/src/yolov5_tracking/yolov5" 
model = torch.hub.load(path,'custom', '/home/advan/yhs_ws/perception/src/yolo_d435i/src/yolov5_D435i/cone.pt', source='local')  #修改为加载本地权重 ：第一个参数应该为本地权重路径
model.conf = 0.5

# device = select_device(device)
# model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)


def get_mid_pos_2(frame,box,depth_data,randnum):
    distance_list = []
    mid_pos = [(box[0] + box[2])//2, (box[1] + box[3])//2] #确定索引深度的中心像素位置
    min_val = min(abs(box[2] - box[0]), abs(box[3] - box[1])) #确定深度搜索范围
    #print(box,)
    for i in range(randnum):
        bias = random.randint(-min_val//4, min_val//4)
        dist = depth_data[int(mid_pos[1] + bias), int(mid_pos[0] + bias)]
        cv2.circle(frame, (int(mid_pos[0] + bias), int(mid_pos[1] + bias)), 4, (255,0,0), -1)
        #print(int(mid_pos[1] + bias), int(mid_pos[0] + bias))
        if dist:
            distance_list.append(dist)
    distance_list = np.array(distance_list)
    distance_list = np.sort(distance_list)[randnum//2-randnum//4:randnum//2+randnum//4] #冒泡排序+中值滤波
    #print(distance_list, np.mean(distance_list))
    return np.mean(distance_list)


def dectshow(org_img, boxs_track, data):  #改为指针发布
    img = org_img.copy()

    boundingboxes = BoundingBoxes()
    time_now = rospy.Time.now()
    boundingboxes.header.stamp = time_now 
    boundingboxes.image_header = data.header
    
    # (trans, rot) = tf_listener.LookupTransform("odometry", "camera", rospu.Time(0))

    for box in boxs_track:
        cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)  #cv2.rectangle需要长方形左上角和右下角坐标  
        #-------发布信息-----------------
        boundingbox = BoundingBox()
        boundingbox.probability = 1.0
        # top_left  bottom_right
        boundingbox.xmin = int(box[0])
        boundingbox.ymin = int(box[1])
        boundingbox.xmax = int(box[2])
        boundingbox.ymax = int(box[3])
        boundingbox.id = int(box[-1])
        # boundingbox.Class = str(box[4])
        #获取中心点的位置信息
        boundingboxes.bounding_boxes.append(boundingbox)
    detection_pub.publish(boundingboxes)
    # cv2.imshow('dect_img', img)


def imageCallback(data):
    global count,bridge
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    results = model( cv_image )
    boxs= results.pandas().xyxy[0].values   #将tensor转化为numpy的格式！！！
    boxs_track=boxs[:, 0:5]
    # print("this is the box:\n",boxs_track)
    #将变量清空
    results_track = mot_tracker.update(boxs_track)
    dectshow(cv_image ,results_track, data) #可视化跟踪结果并发布消息



if __name__ == "__main__":
    # 创建节点及发布订阅器 跟踪器
    mot_tracker = Sort()    #设置max_age min_hits等参数


    # while True:
    # ----------------init rosnode--------------  此处发布及读取最好改为读取参数
    rospy.init_node('detection', anonymous=True)
    image_subscriber = rospy.Subscriber('/image_view/image_raw', Image, imageCallback)
    tracking_pub = rospy.Publisher('/detection/tracking', BoundingBoxes, queue_size=10)
    detection_pub = rospy.Publisher('/detection/detection', BoundingBoxes, queue_size=10)
    
    # tf监听器 获取静态tf和动态里程计tf变换
    # tf_listener = tf2_ros.TransformLister() 

    # 加载模型  读取参数
    # device = select_device(device)
    # model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    # stride, names, pt = model.stride, model.names, model.pt
    # imgsz = check_img_size(imgsz, s=stride)  # check image size


    global count, bridge
    count = 0
    bridge = CvBridge()
    rate = rospy.Rate(10) # 10hzpass
    rospy.spin()
   