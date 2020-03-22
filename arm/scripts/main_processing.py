#!/usr/bin/env python

"""
this is main processing file , this includes'
subscribers to all topic
publishing of master sting
importing of sub files to use their functions
saving all the values globally (v imp!)
"""

#importing required libraries
import rospy
import math
import tf
import numpy as np
import time

#importing assistant files(larger the number , more the priority)
import gps_and_yaw_processing_0
#import camera_processing_1
#import pitch_roll_processing_2
#import lidar_processing_3

#importing messages
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#master string publisher
pub = rospy.Publisher('master_string_topic', String, queue_size=15)

#setting up fixed variables used in tuning
goal_lat  = 28.752133
goal_lon  = 77.119220
min_dist_to_stop=10.0 #rover will stop within 10m radius of the goal coordinates
straight_allowance=4.0 #rover motion will be considered straight within -4 to -4 degrees
rotation_allowance=20.0 #rover will start turning if required rotation is not in -20 to +20 degree

#setting up global variables
#1) for gps
rover_lat = 0.0
rover_lon = 0.0

#2) for imu
yaw   = 0.0
pitch = 0.0
roll  = 0.0

#3) for camera
#

#4) for lidar
#lidar_ranges = 

#others
trigger=0
intr=0
master_string=String()
master_string="00000"
master_string_old="00000"
# index 0 : motion type
# index 1 : pwm
# index 2 : pwm
# index 3 : pwm
# index 4 : interrupt

minimum_distance=0.0
rover_rotating_angle=0.0


def calculation_and_display_function():
    #globalizing every value found
    global master_string, master_string_old , minimum_distance , rover_rotating_angle , trigger, rover_lat,rover_lon,goal_lat,goal_lon
    global intr
    master_string_old=master_string
    minimum_distance=gps_and_yaw_processing_0.calc_min_distance(rover_lat, rover_lon, goal_lat, goal_lon)
    rover_rotating_angle=gps_and_yaw_processing_0.calc_rover_rotating_angle(rover_lat, rover_lon, goal_lat, goal_lon,yaw)
    
    if(False):#lidar obstacle sensed
        intr=3
        master_string=master_string[0:4]+str(intr)
        print("lidar sensed obstacle !!!!")
    elif(False):#wrong pitch+roll values sensed
        intr=2
        master_string=master_string[0:4]+str(intr)
        print("imu sensed extreme roll and pitch values !!!!")#interrupt of string becomes 2
    elif(False):#camera detects ball or arrow
        intr=1
        master_string=master_string[0:4]+str(intr)
        print("camera detected something !!!!")#interrupt of string becomes 1
    else:
        intr=0
        master_string=master_string[0:4]+str(intr)
        print("gps and yaw based travel active .......")
    
    """if(intr==3):#related to lidar_processing_3
        master_string=lidar_processing_3.master_string_generator()
    if(intr==2):#related to pitch_roll_processing_2
        master_string=pitch_roll_processing_2.master_string_generator()
    if(intr==1):#related to camera_processing_1
        master_string=camera_processing_1.master_string_generator()"""
    if(intr==0):#related to gps_and_yaw_processing_0.py
        master_string,trigger=gps_and_yaw_processing_0.master_string_generator(minimum_distance, rover_rotating_angle,trigger,min_dist_to_stop,straight_allowance,rotation_allowance)
    
    
    
    #move below lines in listener later.
    print("yaw from north          : "+str(yaw)+" degrees")
    print("goal  (lat , lon)       : ( "+str(goal_lat)+" , "+str(goal_lon)+" )")
    print("rover (lat , lon)       : ( "+str(rover_lat)+" , "+str(rover_lon)+" )")
    print("rover rotation required : "+str(rover_rotating_angle)+" degrees")
    print("Distance to goal        : "+str(minimum_distance)+"  meters")
    print("master string           : "+str(master_string))
    
    if(master_string_old!=master_string):
        pub.publish("00500")
        print("!!Applying safety delay!!")
        print("")
        time.sleep(2)#comment this for manual travel mode
    else:
        pub.publish(master_string)
        print("")


#here callbacks are defined just to assign values in global variables
#all processing will be done in separate python file and then imported
def callback_gps(gps_data):
    global rover_lat,rover_lon,count
    rover_lat = gps_data.latitude  #present latitude
    rover_lon = gps_data.longitude #present longitude

def callback_lidar(lidar_data):
    print("lidar subscriber working")

def callback_camera(camera_data):
    print("camera frame subscriber working")

def callback_imu(imu_data):
    global yaw,pitch,roll
    #quaternion=[imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
    #yaw,pitch,roll = gps_and_yaw_processing_0.quaternion_to_ypr(quaternion) #it was earlier
    yaw=imu_data.x #it is new
    if(yaw>180):
        yaw=yaw-360
    calculation_and_display_function()
    
def listener():
    #defining main processing node
    rospy.init_node('main_processing_node', anonymous=False)

    #subscribing all 4 types of sensors used in autonomous
    rospy.Subscriber("fix", NavSatFix , callback_gps)
    #rospy.Subscriber("laser_scan_topic", LaserScan , callback_lidar)
    #rospy.subscriber("camera_topic", ........ , callback_camera)
    rospy.Subscriber("imu_data", Point , callback_imu)#it is new

    #code at this line ------------ willwork only once ! wtf!!
    rospy.spin()

if __name__ == '__main__':
    print("Activating Autopilot Mode")
    listener()
