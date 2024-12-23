#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import serial

# 初始化串口通信
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # 根据实际情况修改串口路径

def cmd_vel_callback(msg):
    # 从消息中提取线速度和角速度
    linear = msg.linear.x
    angular = msg.angular.z

    # 构造指令字符串
    command = f"linear:{linear} angular:{angular}\n"

    # 将指令发送到 Arduino
    ser.write(command.encode('utf-8'))

def main():
    rospy.init_node('cmd_vel_to_arduino')  # 初始化节点
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)  # 订阅 /cmd_vel 话题
    rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()  # 关闭串口
