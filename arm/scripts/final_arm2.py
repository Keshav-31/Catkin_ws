#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from pcl_msgs.msg import Vertices
from numpy import interp
arm = Vertices()
arm.vertices = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
actuator_x=0
actuator_y=0
base=0
wrist=0
gripper_rotation=0
gripper_jaw=0

def move_base(data):
        
        global base
        base=0
        base_pwm=0
        '''bl= data.buttons[4]
        br=data.buttons[5]
        if((br == 0 and bl == 0) or (br==1 and bl ==1)) :
                base = 0
        if(br == 1 and bl == 0) :
                base = 1
        if(br == 0 and bl == 1) :
                base = 2'''
        
        if(data.buttons[4]==1):
            base_axis=data.axes[3]
            base_pwm= interp(base_axis,[-1,1],[-180,180])
            if(base_pwm>0) :
                base=1
            if(base_pwm<0):
                base=2
                base_pwm=base_pwm*-1
            if(base_pwm==0):
                base=0

        arm.vertices[2] = base
        arm.vertices[7] = base_pwm



def move_gripper(data):

        global wrist
        global gripper_jaw
        global gripper_rotation

       #wu = data.buttons[0]
        #wd = data.buttons[2]
        wrist_axes= data.axes[4]
        wrist_pwm = interp(wrist_axes,[-1,1],[-255,255])
        

        wr = data.buttons[1]
        wl = data.buttons[3]

        jclose = data.buttons[7]
        jopen = data.buttons[6]

        '''if((wu == 0 and wd == 0) or (wu==1 and wd ==1)) :
                wrist = 0
        if(wu == 1 and wd == 0) :
                wrist = 1
        if(wu == 0 and wd == 1) :
                wrist = 2'''
        
        if(wrist_pwm>8) :
            wrist=1
        if(wrist_pwm<0):
            wrist=2
            wrist_pwm=wrist_pwm*-1
	    if(wrist_pwm<9):
		wrist_pwm=0
			
        if(wrist_pwm==0):
            wrist=0

        if((wl == 0 and wr == 0) or (wl==1 and wr ==1)) :
                gripper_rotation = 0
        if(wl == 1 and wr == 0) :
                gripper_rotation = 1
        if(wl == 0 and wr == 1) :
                gripper_rotation = 2

        if((jclose == 0 and jopen == 0) or (jclose==1 and jopen ==1)) :
                gripper_jaw = 0
        if(jclose == 1 and jopen == 0) :
                gripper_jaw = 1
        if(jclose == 0 and jopen == 1) :
                gripper_jaw = 2

        arm.vertices[3] = wrist
        arm.vertices[4] = gripper_rotation
        arm.vertices[5] = gripper_jaw
        arm.vertices[8] = wrist_pwm
        
        
def move_actuators(data):

        global actuator_x
        global actuator_y
        x = data.axes[0]
        y = data.axes[1]

        act_pwm = interp(x,[-1,1],[-255,255])
        act_pwm1 = interp(y,[-1,1],[-255,255])

        if(data.buttons[13]==0 and data.buttons[14]==0 and data.buttons[15]==0 and data.buttons[16]==0):
            if(y>0) :
                actuator_x=2
                actuator_y=2
                arm.vertices[9] = act_pwm1
            if(y<0):
                actuator_x=1
                actuator_y=1
                arm.vertices[9] = act_pwm1*-1
            if(x>0):
                actuator_x=1
                actuator_y=2
                arm.vertices[9] = act_pwm
            if(x<0):
                actuator_x=2
                actuator_y=1
                arm.vertices[9] = act_pwm * -1
            if(x==0 and y==0):
                actuator_x=0
                actuator_y=0
                arm.vertices[9] = 0

        
        
            arm.vertices[0] = actuator_x
            arm.vertices[1] = actuator_y

def check_typing(msg):
        a=msg.buttons[8]
        if(a==1) :
                t=4
        else :
                t=0
        arm.vertices[6]=t

def move_single_actuators(data):
        act1up = data.buttons[13]
        act1down = data.buttons[14]

        act2up = data.buttons[15]
        act2down = data.buttons[16]
        if(data.axes[0] ==0 and data.axes[1]==0):
            if((act1up == 0 and act1down == 0) or (act1up==1 and act1down ==1)) :
                act1 = 0
            if(act1up == 1 and act1down == 0) :
                act1 = 1
            if(act1up == 0 and act1down == 1) :
                act1 = 2

            if((act2up == 0 and act2down == 0) or (act2up==1 and act2down ==1)) :
                act2 = 0
            if(act2up == 1 and act2down == 0) :
                act2 = 1
            if(act2up == 0 and act2down == 1) :
                act2 = 2
        
            arm.vertices[0] = act1
            arm.vertices[1] = act2



        
def move_arm(data):
    rate = rospy.Rate(10)
    check_typing(data)
    move_actuators(data)
    move_single_actuators(data)
    move_gripper(data)
    move_base(data)
    pub = rospy.Publisher('move_arm', Vertices ,queue_size=1)  #sends the Vertices message to "robotic_arm" topic
    pub.publish(arm)
    rate.sleep()

def joy_to_arm(): #function to recieve values from joystick through joy_node of joy package
    rospy.init_node('joy_to_arm', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("joy_for_arm", Joy, move_arm, queue_size=1)
    rate.sleep()
    rospy.spin()
    
if __name__ == '__main__':
    joy_to_arm()

