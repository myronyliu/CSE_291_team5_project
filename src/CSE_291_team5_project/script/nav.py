#!/usr/bin/env python
import rospy
import pici
import serial
import constant as C
from std_msgs.msg import Int8,Bool
from CSE_291_team5_project.msg import CVMessage # m = CVMessage()
import sys

global driver, state
driver = None
state = C.STATE_Looking_For_Waving


def stop_for_cone(width = None, height = None):
    if not height:
        return True

    if height >= C.STOP_THRESHOLD_HEIGHT_CONE:
        return True
    else:
        return False
    




def stop_for_waving(width = None, height = None):
    if not height:
        return True
    if height >= C.STOP_THRESHOLD_HEIGHT_WAVING:
        return True
    else:
        return False

def get_drive_action(x,y):
    global driver
    if x >= C.STRAIGHT_THRESHOLD_X:
        driver.publish(C.MOVE_LEFT)
    elif x <= -C.STRAIGHT_THRESHOLD_X:
        driver.publish(C.MOVE_RIGHT)
    else:        
        driver.publish(C.MOVE_UP)
    



def cv_cone_callback(msg):
    global driver,state
    d = msg.data
    x = d.x
    y = d.y
    w = d.width
    h = d.height
    if state == C.STATE_Looking_For_Cone:        
        if stop_for_cone(width=w,height=h):
            driver.publish(C.STOP)
        else:
            driver.publish(get_drive_action(x,y))


def cv_waving_callback(msg):
    global driver,state
    d = msg.data
    x = d.x
    y = d.y
    w = d.width
    h = d.height
    if state == C.STATE_Looking_For_Waving:        
        if stop_for_waving(width=w,height=h):
            state = C.STATE_Waiting_For_Comand
            driver.publish(C.STOP)
        else:
            driver.publish(get_drive_action(x,y))


def cv_gesture_callback(msg):
    global state
    if state == C.STATE_Waiting_For_Comand:        
        if msg.data == C.GESTURE_PUT_STUFF:
            state = C.STATE_Looking_For_Cone
        elif msg.data == C.GESTURE_REMOVE_STUFF:
            state = C.STATE_Looking_For_Waving
    



def nav():
    global driver
    rospy.init_node('nav', anonymous=True)
    driver = rospy.Publisher(C.DRIVER_ADDR, Int8, queue_size=20);


    rospy.Subscriber(C.CV_CONE_ADDR,CVMessage, cv_cone_callback)
    rospy.Subscriber(C.CV_WAVING_ADDR,CVMessage, cv_waving_callback)    
    rospy.Subscriber(C.CV_GESTURE_ADDR, Int8, cv_gesture_callback)    
    rospy.spin()
    




if __name__ == '__main__':
    nav()        


