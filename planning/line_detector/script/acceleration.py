#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from fsd_common_msgs.msg import Map
from fsd_common_msgs.msg import Cone

def main():
    # 初始化节点
    rospy.init_node('map_publisher')

    # 创建一个发布者，发布 fsd_common_msgs/Map 类型消息，话题名称为 /map
    pub = rospy.Publisher('/local_map', Map, queue_size=10)

    # 创建一个 fsd_common_msgs/Map 类型消息对象
    map_msg = Map()
    
    count = 0
    
    for i in range(10):
        cone_red_count = Cone()
        cone_red_count.position.x = float(count)
        cone_red_count.position.y = 0.0
        #cone_red_count.color = "r"

        cone_blue_count = Cone()
        cone_blue_count.position.x = float(count)
        cone_blue_count.position.y = 4.0
        #cone_blue_count.color = "b"
        
        map_msg.cone_red.append(cone_red_count)
        map_msg.cone_blue.append(cone_blue_count)

        count+=1


    # 设置消息内容


    #map_msg.header = rospy.Time()
    #map_msg.image_header = rospy.Time()

    # 循环发布消息
    rate = rospy.Rate(10) # 设置发布频率为 1Hz
    print(map_msg)
    print(len(map_msg.cone_red))
    while not rospy.is_shutdown():
        pub.publish(map_msg) # 发布消息
        print(map_msg)
        print(len(map_msg.cone_red))
        rate.sleep() # 等待下一个周期

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass