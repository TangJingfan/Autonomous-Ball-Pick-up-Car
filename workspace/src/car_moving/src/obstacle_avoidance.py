#!/usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import LaserScan

# init serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

def callback(scan):
    min_distance = min(scan.ranges)
    command = "STOP"  

    if min_distance < 0.3:  # 障碍物距离小于30cm
        command = "TURN"  # 转弯指令
    else:
        command = "FORWARD"  # 前进指令

    # 发送命令到 Arduino
    ser.write((command + '\n').encode())
    rospy.loginfo(f"Sent command: {command}")

def listener():
    rospy.init_node('lidar_obstacle_avoidance', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()
