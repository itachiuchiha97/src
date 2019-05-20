#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Float32
import serial
import struct

# ard = serial.Serial('COM',9600)
port = rospy.get_param('~port','/dev/ttyUSB1')
ard = serial.Serial(port,9600)


def rcallback(msg):  
        y  = int(msg.data)        
        print(y)        
        ans = chr(y)
        #print(ans)
        my_ans = ans.encode("latin")
        ard.write(my_ans)


def runMotor():   
        rospy.init_node('rmotor_run')
        #rate = rospy.Rate(10)
        rospy.Subscriber("rmotor_cmd", Float32, rcallback)
        rospy.spin()

if __name__ == "__main__":
        try:
                print("ok")
                runMotor()
        except rospy.ROSInterruptException:
                print("tatti")
                pass