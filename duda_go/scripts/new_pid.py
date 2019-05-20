#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from numpy import array
import time
import serial
import struct



rospy.init_node("calc_velocity")


flag = 0
prev_encoder = 0
prev_time = 0
curr_encoder = 0
degree = 30
curr_time = 0
one_radian = 0.0175
wheel_radius = 0.127 # in meter
previous_error = 0
integral = 0.0
error = 0.0
derivative = 0.0
prev_pid_time = 0
target = 0
vel = 0


# ROS Parameters
Kp = rospy.get_param('~Kp',15)
Ki = rospy.get_param('~Ki', 0)
Kd = rospy.get_param('~Kd',20)
out_min = rospy.get_param('~out_min', 0)
out_max = rospy.get_param('~out_max',63)
timeout_const = rospy.get_param('~timeout_const',1.2)
sample_size = rospy.get_param('~sample_size',3)
rate = rospy.get_param('~rate',100)

prev_vel = [0.0]*sample_size


def wheelCallback(msg):
    #####################################################
        global flag
        global prev_encoder
        global curr_encoder
        global prev_time
        global curr_time
        
        if flag == 0:
            prev_encoder = msg.data
            prev_pid_time = prev_time = time.time()
            flag = 1
            print("Initialized")
        else:
            curr_encoder = msg.data

            

def appendVel(val):
    #####################################################
        global prev_vel
        
        prev_vel.append(val)
        del prev_vel[0]

def calcRollingVel():
    #####################################################
        global prev_vel, vel
        p = array(prev_vel)
        vel = p.mean()
        pub_vel.publish(vel)
        #print(vel)

def doPid():
    #####################################################
        # global target
        # print(target)
        global error,integral,previous_error,derivative,out_max,out_min, vel, prev_pid_time

        pid_dt = time.time() - prev_pid_time        
        prev_pid_time = time.time()
        
        error = target - vel
        integral = integral + (error * pid_dt)

        derivative = (error - previous_error) / pid_dt
        previous_error = error
    
        motor = (Kp * error) + (Ki * integral) + (Kd * derivative)       
        

        if motor > out_max:
            motor = out_max
            integral = integral - (error * pid_dt)
        if motor < out_min:
            motor = out_min
            integral = integral - (error * pid_dt)
      
        if (target == 0):
            motor = 0
        
        motor = int(motor)
        pub_motor.publish(motor)
        


def targetCallback(msg):
    ######################################################
        global target
        target = msg.data
        # self.ticks_since_target = 0
        # rospy.logdebug("-D- %s targetCallback " % (self.nodename))
    
if __name__ == "__main__":
   
    nodename = rospy.get_name()
    rospy.loginfo("%s started" % nodename)
    prev_time = time.time()
    prev = time.time()
    #### subscribers/publishers
    rospy.Subscriber("wheel", Int16, wheelCallback)
    rospy.Subscriber("wheel_vtarget", Float32, targetCallback)
    pub_motor = rospy.Publisher('motor_cmd',Int16, queue_size=10) 
    pub_vel = rospy.Publisher('wheel_vel', Float32, queue_size=10)

    r = rospy.Rate(rate)

    while not rospy.is_shutdown():

        if(curr_encoder > prev_encoder): 
                curr_time = time.time()
                dt = curr_time - prev_time
                prev_time = curr_time
                omega = (degree/dt)*one_radian
                speed = omega*wheel_radius
                appendVel(speed)
                calcRollingVel()
                prev_encoder = curr_encoder
                prev = time.time()
                
                
        elif (curr_encoder == prev_encoder) and (time.time()-prev > 1.2):
                #print(time.time()-prev)
                appendVel(0)
                calcRollingVel()

        doPid()
        r.sleep()


    rospy.spin()
