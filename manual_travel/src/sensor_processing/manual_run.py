#!/usr/bin/env python
#importing required libraries
import rospy
import tf
import time
#importing assistant files(larger the number , more the priority)
import gps_and_yaw_processing_0

#importing messages
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

#tuning parameters
goal_lat  = 28.751314
goal_lon  = 77.119187


#setting up global variables
#1) for gps
rover_lat = 0.0
rover_lon = 0.0

#2) for imu
yaw     = 0.0
pitch   = 0.0
roll    = 0.0
Bearing=0.0

goal_marker_distance=0.0
rover_rotating_angle=0.0

initial_time=0.0


def calculation_and_display_function():
    #globalizing every value found
    global goal_marker_distance , rover_rotating_angle , rover_lat,rover_lon,goal_lat,goal_lon, Bearing, initial_time
   
    goal_marker_distance=gps_and_yaw_processing_0.calc_min_distance(rover_lat, rover_lon, goal_lat, goal_lon)
    rover_rotating_angle, Bearing =gps_and_yaw_processing_0.calc_rover_rotating_angle(rover_lat, rover_lon, goal_lat, goal_lon,yaw)
    print("elapsed time              : "+str(format(((time.time()-initial_time)/60),'.2f'))+" minutes")
    print("gps bearing               : "+str(Bearing))
    print("yaw (-180 to +180) format : "+str(yaw)+" degrees")
    print("goal  (lat , lon)         : ( "+str(goal_lat)+" , "+str(goal_lon)+" )")
    print("rover (lat , lon)         : ( "+str(rover_lat)+" , "+str(rover_lon)+" )")
    print("rover rotation required   : "+str(rover_rotating_angle)+" degrees")
    print("Distance to goal          : "+str(goal_marker_distance)+"  meters")
    print("..........................................................")
    

#here callbacks are defined just to assign values in global variables
#all processing will be done in separate python file and then imported
def callback_gps(gps_data):
    global rover_lat,rover_lon
    rover_lat = gps_data.latitude  #present latitude
    rover_lon = gps_data.longitude #present longitude

def callback_imu(imu_data):
    global yaw,pitch,roll
    quaternion=[imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
    yaw,pitch,roll = gps_and_yaw_processing_0.quaternion_to_ypr(quaternion)
    
    #samsung phone tested
    yaw=yaw*-1#samsung gives opposite sign values
    if(yaw>180):
        yaw=yaw-360

    calculation_and_display_function()
    
def listener():
    rospy.init_node('main_processing_node', anonymous=False)
    
    rospy.Subscriber("android/fix", NavSatFix , callback_gps)
    rospy.Subscriber("android/imu", Imu , callback_imu)#it is new
    rospy.spin()

if __name__ == '__main__':
    global initial_time
    initial_time=time.time()
    listener()
