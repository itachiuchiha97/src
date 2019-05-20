#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from math import cos
 
def callback(msg):
    scanner.header = msg.header
    scanner.angle_min = msg.angle_min
    scanner.angle_max = msg.angle_max
    scanner.angle_increment = msg.angle_increment
    scanner.time_increment = msg.time_increment
    scanner.scan_time = msg.scan_time
    scanner.range_min = msg.range_min
    scanner.range_max = msg.range_max
    num = list(msg.ranges)
    start = 1.2
    L = 1.219

    #print(len(msg.ranges))

    for x in range(0,541):
        c = str(num[x])
        num[x] = abs(L/cos(0.5236+(x)*(2.0944/541)))
    
        
    scanner.ranges = num
    scanner.intensities = msg.intensities
    pub.publish(scanner)
    start = 1.2
    
 
rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('newscan',LaserScan,queue_size=10)
scanner = LaserScan()
rospy.spin()