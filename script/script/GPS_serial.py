# -*- coding: utf-8 -*-

from sensor_msgs.msg import Imu, NavSatFix
import tf
import rospy
import serial
from enum import Enum

# 系统状态
class System_status(Enum):
    INITIAL = 0
    INERTIAL_NAVIGATION_MODE = 1
    STRAPDOWN_INERTIAL_NAVIGATION_MODE = 2
    GIMBALED_INERTIAL_NAVIGATION_MODE = 3
# 卫星状态
class SateliteMode(Enum):
    UNDIRECTED_UNPOSITION = 0
    SINGLE_POINT_DIRECT = 1
    PSEUDORANGE_DIFFERENTIAL_DIRECT = 2
    COMBINED_NAVIGATION = 3
    RTK_FIXED_DIRECT = 4
    RTK_FLOAT_DIRECT = 5
    SINGLE_POINT_UNDIRECTED = 6
    PSEUDORANGE_DIFFERENTIAL_UNDIRECTED = 7
    RTK_FIXED_UNDIRECTED = 8
    RTK_FLOAT_UNDIRECTED = 9
    
ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

imu_publisher = rospy.Publisher('imu_data', Imu, queue_size=10)
gps_publisher = rospy.Publisher('gps', NavSatFix, queue_size=10)
fix_publisher = rospy.Publisher('fix', NavSatFix, queue_size=10)

rospy.init_node('RTK_node', anonymous=True)

while not rospy.is_shutdown():
    data = ser.readline().strip().decode('utf-8')
    if not data:
        continue

    if data.startswith('$GPCHC'):
        fields = data[:-2].split(',')

        # 解析GPCHC数据
        msg_id = fields[0]
        GPSWeek = int(fields[1])
        GPSTime = float(fields[2])
        Heading = float(fields[3])
        Pitch = float(fields[4])
        Roll = float(fields[5])
        gyro_x = float(fields[6])
        gyro_y = float(fields[7])
        gyro_z = float(fields[8])
        acc_x = float(fields[9])
        acc_y = float(fields[10])
        acc_z = float(fields[11])
        Lattitude = float(fields[12])
        Longitude = float(fields[13])
        Altitude = float(fields[14])
        Ve = float(fields[15])
        Vn = float(fields[16])
        Vu = float(fields[17])
        Baseline = float(fields[18])
        NSV1 = int(fields[19])
        NSV2 = int(fields[20])
        Status = fields[21]
        Age = float(fields[22])
        # Warming_Cs = fields[23]
        
        system_mode = System_status(int(Status[1]))
        sta_mode = SateliteMode(int(Status[0]))
        
        # 发布IMU数据到ROS话题
        imu_data = Imu(header=rospy.Header(stamp=rospy.Time.now(), frame_id='imu_link'))
        
        imu_data.angular_velocity.x = gyro_x
        imu_data.angular_velocity.y = gyro_y
        imu_data.angular_velocity.z = gyro_z

        imu_data.linear_acceleration.x = acc_x
        imu_data.linear_acceleration.y = acc_y
        imu_data.linear_acceleration.z = acc_z

        # 将RPY角转换为四元数
        quaternion = tf.transformations.quaternion_from_euler(float(Roll), float(Pitch), float(Heading))
        imu_data.orientation = quaternion

        imu_publisher.publish(imu_data)

        # 发布GPS数据到ROS话题
        gps_data = NavSatFix(header=rospy.Header(stamp=rospy.Time.now(), frame_id='gps'))
        gps_data.latitude= Lattitude
        gps_data.longitude=Longitude
        gps_data.altitude=Altitude
        
        if sta_mode != SateliteMode.UNDIRECTED_UNPOSITION :
            fix_publisher.publish(gps_data)
            gps_publisher.publish(gps_data)
            print("SateliteMode: " + sta_mode.name + "; System_status: " + system_mode.name)
        else:
            gps_publisher.publish(gps_data)
            print("SateliteMode: " + sta_mode.name + "; System_status: " + system_mode.name)

        print("Week: %d, Time: %.3f, Lon: %.7f, Lat: %.7f, Pitch: %.3f" % (GPSWeek, GPSTime, Longitude, Lattitude, Pitch))
        
