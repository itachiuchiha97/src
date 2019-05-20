#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

rospy.init_node('vision_drive')
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
twist = Twist()
Kp = 1

def callback(data):
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = Kp*(data.data)
    pub.publish(twist)
    
def listener():    
    rospy.Subscriber("error", Float32, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()