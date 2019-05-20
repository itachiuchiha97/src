#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

import serial, string, math, time, calendar

import tf
from tf.transformations import euler_from_quaternion , quaternion_from_euler
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


if __name__ == '__main__':
    global D_Compass
    global myStr1
    
    #Init D_Compass port
    D_Compassport = rospy.get_param('~port','/dev/ttyUSB0')
    D_Compassrate = rospy.get_param('~baud',115200)
    #<!--printmodulus 60:10Hz 40:15~17  35:17~18Hz 30:21Hz 25:23~27Hz ,20: 30~35Hz,15:35~55Hz 10: 55~76 Hz,  5: 70~100 Hz, 1:70~100 Hz -->

   
    #Setup Compass serial port
    D_Compass = serial.Serial(port=D_Compassport, baudrate=D_Compassrate, timeout=.5)
    # Stop continus mode
    D_Compass.flush() # flush data out
    time.sleep(0.5)

    while True:
                    
        data = D_Compass.readline()
        #rospy.loginfo("Received a sentence: %s" % data)

        #if not check_checksum(data):
        #    rospy.logerr("Received a sentence with an invalid checksum. Sentence was: %s" % data)
        #    continue

        #DatatimeNow = rospy.get_rostime()
        fields = data.split(',')
        #print fields[0]+fields[2]+fields[6]+fields[10] #P:apgpq

    
        if len(fields)>14:
            if 'P:apgpq' == (fields[0]+fields[2]+fields[6]+fields[10]):

                #      0  1 mSec 2  3Ax  4Ay     5Az     5  7Gx  8Gy  9G    10 11YawT 1213w  14x   15y  16z
                #data='P:,878979,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,yt,342.53,q,0.98,-0.01,0.01,-0.15'
                #data='P:,%i,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,q,0.98,-0.01,0.01,-0.15\n'
                #      0  1  2  3     4      5       6  7    8    9    10 11   12    13   14  

                Ax=float(fields[3])/1000.*9.81 # convert to m/s^2 from mg/s
                Ay=float(fields[4])/1000.*9.81
                Az=float(fields[5])/1000.*9.81
                Gx=float(fields[7]) * (math.pi/180.0) # convert to radians from degrees
                Gy=float(fields[8]) * (math.pi/180.0)
                Gz=float(fields[9]) * (math.pi/180.0)
                w =float(fields[11])
                x =float(fields[12])
                y =float(fields[13])
                z =float(fields[14])
                print(x,y,z)
                        
    #D_Compass.write(myStr1) # stop data stream before close port
        #D_Compass.flush() # flush data out