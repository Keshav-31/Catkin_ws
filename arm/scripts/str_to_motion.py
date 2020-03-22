#!/usr/bin/env python
import rospy
from drive.msg import drive_msg
from std_msgs.msg import String
drivex=drive_msg()

def string_to_motor_command(data):
    print("coverting master_string to motor driver commands")
    movement = int(data.data[0])
    pwm = int(data.data[1:4])
    intr = int(data.data[4])
    drivex.ldir = bool(0)
    drivex.rdir = bool(0)
    if(movement != 0):
        drivex.lpwm = pwm 
        drivex.rpwm = pwm
        if(movement > 1):
            if(movement < 4):
                drivex.rdir = 1
        if(movement == 2):
            drivex.ldir = 1
        if(movement == 4):
            drivex.ldir = 1

        
    else:
        drivex.lpwm = 0 
        drivex.rpwm = 0
        drivex.ldir = bool(0)
        drivex.rdir = bool(0)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('rover_drive', drive_msg ,queue_size=10)
    pub.publish(drivex)
    #rate.sleep()

def listener():
    rospy.init_node('sub_master_str_pub_rover_drive', anonymous=True)
    rospy.Subscriber("master_string_topic", String, string_to_motor_command)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
