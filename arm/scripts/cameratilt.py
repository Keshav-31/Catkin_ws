#!/usr/bin/env python
import rospy
from pcl_msgs.msg import Vertices
import time

import getkey as gk
pantilt = Vertices()
pan=0
tilt=0
antena=0
pantilt.vertices=[0, 0, 0]

def talker():
    global pan
    global tilt
    global antena
    pub = rospy.Publisher('servo_pantilt', Vertices , queue_size=10)
    rospy.init_node('pan_tilt', anonymous=True)
    rate = rospy.Rate(3) # 3hz3
    while(True) :
        key = gk.getkey()
        if(key==gk.keys.UP) :
            while(key==gk.keys.UP) :
                pan=pan+5
                if(pan>180):
                    pan=180
                if(pan<0):
                    pan=0
                pantilt.vertices[0] = pan
                pub.publish(pantilt)
                rate.sleep()
                key=gk.getkey()
        if(key==gk.keys.DOWN) :
            while(key==gk.keys.DOWN) :
                pan=pan-5
                if(pan>180):
                    pan=180
                if(pan<0):
                    pan=0
                pantilt.vertices[0] = pan
                pub.publish(pantilt)
                rate.sleep()
                key=gk.getkey()
        if(key==gk.keys.RIGHT) :
            while(key==gk.keys.RIGHT) :
                tilt=tilt+5
                if(tilt>180):
                    tilt=180
                if(tilt<0):
                    tilt=0
                pantilt.vertices[1] = tilt
                pub.publish(pantilt)
                rate.sleep()
                key=gk.getkey()
        if(key==gk.keys.LEFT) :
            while(key==gk.keys.LEFT) :
                tilt=tilt-5
                if(tilt>180):
                    tilt=180
                if(tilt<0):
                    tilt=0
                pantilt.vertices[1] = tilt
                pub.publish(pantilt)
                rate.sleep()
                key=gk.getkey()
        if(key=='a' or key=='A') :
            
            while(key=='a' or key=='A') :
                antena=1
                pantilt.vertices[2] = antena
                pub.publish(pantilt)
                rate.sleep()
                key=gk.getkey()
            antena=0
            pantilt.vertices[2] = antena
            pub.publish(pantilt)
        if(key=='D' or key=='d') :
            antena=2
            pantilt.vertices[2] = antena
            while(key=='D' or key=='d') :
                pub.publish(pantilt)
                rate.sleep()
                key=gk.getkey()
            antena=0
            pantilt.vertices[2] = antena
            pub.publish(pantilt)
        if(key=='s' or key=='S') :
            pantilt.vertices[2]=0
            pub.publish(pantilt)

    
    
       

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
