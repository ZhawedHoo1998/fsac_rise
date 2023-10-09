import rospy
from nav_msgs.msg import Odometry
import time

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    current_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
    with open('odom_data.yaml', 'a') as file:
        file.write('Time: {}, x: {}, y: {}\n'.format(current_time, x, y))
    print('Write done!')

def subscribe_odom_topic():
    rospy.init_node('odom_subscriber', anonymous=True)
    rospy.Subscriber('/vehicle/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    subscribe_odom_topic()