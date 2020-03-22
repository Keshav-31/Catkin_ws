import math
from math import *
import time
import tf

def calc_min_distance(rover_lat, rover_lon, goal_lat, goal_lon):
    R = 6371e3
    rover_lon, rover_lat, goal_lon, goal_lat = map(radians, [rover_lon, rover_lat, goal_lon, goal_lat])
    del_lat = goal_lat-rover_lat
    del_lon = goal_lon-rover_lon
    a = sin(del_lat/2) * sin(del_lat/2) + cos(rover_lat) * math.cos(goal_lat) * sin(del_lon/2) * sin(del_lon/2)
    c = 2 * math.atan2((a)**0.5, (1-a)**0.5)
    d = R * c
    return d

def calc_rover_rotating_angle(rover_lat, rover_lon, goal_lat, goal_lon,yaw):
    dLon = goal_lon - rover_lon
    y = sin(dLon) * cos(goal_lat)
    x = cos(rover_lat) * sin(goal_lat) \
	    - sin(rover_lat) * cos(goal_lat) * cos(dLon)
    Bearing = degrees(atan2(y, x))
    Bearing = -1*Bearing

    rover_rotating_angle=Bearing-yaw
    if(rover_rotating_angle>180):
        rover_rotating_angle=rover_rotating_angle-360
    
    if(rover_rotating_angle<-180):
        rover_rotating_angle=360+rover_rotating_angle
    
    return rover_rotating_angle,Bearing

def quaternion_to_ypr(quaternion):
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    yaw2=math.degrees(rpy[2])
    pitch2=math.degrees(rpy[1])
    roll2=math.degrees(rpy[0])
    return yaw2, pitch2, roll2

