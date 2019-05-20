#!/usr/bin/env python
import serial
import roslib
import rospy
import time as T
from sensor_msgs.msg import NavSatFix

ser=serial.Serial('/dev/ttyUSB0',19200)

def dcm_to_deg(lat,lon):
    lat_deg = float(lat[0:2])
    lat_min = float(lat[2: 5])
    lat_sec = float(lat[5:9])/100
    lon_deg = float(lon[0:3])
    lon_min = float(lon[3:6])
    lon_sec = float(lon[6:10])/100

    lat_degC = (lat_sec/3600) + (lat_min/60) + lat_deg
    lon_degC = (lon_sec/3600) + (lon_min/60) + lon_deg
    
    return lat_degC,lon_degC
    # print(lat_deg,lat_min,lat_sec)
    # print(lon_deg,lon_min,lon_sec)
    # print(lat_degC)
    # print(lon_degC)


def gps_publish():    
    
    rospy.init_node('gps', anonymous=True)
    gpsPub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        while ser.inWaiting()==0:
            pass
        arr = ser.readline()
        arr3 = arr.decode('utf-8','ignore')
        
        if arr3[1:6] == "GPGGA" and len(arr) == 92:
            #print(arr3)
            arr2 = arr3.split(',')
            #print(arr2)
            time = arr2[1]
            lat = arr2[2] 
            latdir = arr2[3]
            if latdir != 'N':
                lat = -lat
            lon = arr2[4]
            londir = arr2[5]
            if londir != 'E':
                lon = -lon
            #print(lat,lon)
            lat_degC , lon_degC = dcm_to_deg(lat,lon)
            sats = arr2[7]
            alt = arr2[9]

            gps = NavSatFix()
            gps.header.stamp = rospy.Time.now()
            gps.header.frame_id = 'gps_link'
            gps.latitude = lat_degC
            gps.longitude = lon_degC
            gps.altitude = float(alt)
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            gpsPub.publish(gps)
            rate.sleep()
           


if __name__ == '__main__':
    try:
        gps_publish()
    except rospy.ROSInterruptException:
        pass