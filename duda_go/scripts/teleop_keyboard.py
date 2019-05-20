#!/usr/bin/env python

import keyboard  
import rospy
import roslib
from geometry_msgs.msg import Twist


if __name__=="__main__":
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_keyboard')
    twist = Twist()
    linSpeed = 0
    angSpeed = 0

    while True:  # making a loop
        #print("In Loop")
        
        if keyboard.is_pressed('0'):  # if key 'q' is pressed 
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angSpeed
            pub.publish(twist)

            linSpeed = float(input("Enter Linear Speed-"))
            angSpeed = float(input("Enter Angular Speed-"))
            
            twist.linear.x = linSpeed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angSpeed
            pub.publish(twist)
            pass  # finishing the loop
        else:
            #print("In Else")
            pub.publish(twist)

            pass
    

