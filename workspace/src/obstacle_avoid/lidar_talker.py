#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def obstacle_avoidance(data):
    ranges = data.ranges
    min_distance = min(ranges)  
    cmd_vel = Twist()

    if min_distance < 0.1:  
        cmd_vel.linear.x = 0.0  
        cmd_vel.angular.z = 0.5  
    else:
        cmd_vel.linear.x = 0.2 
        cmd_vel.angular.z = 0.0

    vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, obstacle_avoidance)
    rospy.spin()

    