#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point

def main():
    # 初始化ROS节点
    rospy.init_node('end_point_publisher', anonymous=True)

    # 创建一个发布者，发布类型为geometry_msgs/Point，话题名为/planning/end_point
    pub = rospy.Publisher('/planning/end_point', Point, queue_size=10)

    # 创建一个Point消息，设置其坐标值
    point = Point()
    point.x = 82
    point.y = 2

    # 设置发布频率为1Hz
    rate = rospy.Rate(1)

    # 循环发布消息
    while not rospy.is_shutdown():
        pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass