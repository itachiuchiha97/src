#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Int16
import serial
import struct

# ard = serial.Serial('COM',9600)
port = rospy.get_param('~port','/dev/ttyUSB1')
ard = serial.Serial(port,9600)


def lcallback(msg):
        x  = int(msg.data)        
        #ans = chr(x)
        #print(ans)
        #my_ans = ans.encode("latin")
        my_ans = struct.pack("B",x)
        ard.write(my_ans)


def rcallback(msg):  
        y  = int(msg.data)
        y = y + 128               
        # ans = chr(y)
        # #print(ans)
        # my_ans = ans.encode("latin")
        my_ans = struct.pack("B",y)
        ard.write(my_ans)


def runMotor():   
        rospy.Subscriber("rmotor_cmd", Int16, rcallback)
        rospy.Subscriber("lmotor_cmd", Int16, lcallback)
        rospy.spin()

if __name__ == "__main__":
        try:
                rospy.init_node('motor_run')
                nodename = rospy.get_name()
                rospy.loginfo("%s started" % nodename)
                runMotor()
        except rospy.ROSInterruptException:
                pass
