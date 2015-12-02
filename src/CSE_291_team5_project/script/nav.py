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
state = C.STATE_LISTEN


def stop_for_cone(width = None, height = None):
    if not height:
        return True
    if height >= C.STOP_THRESHOLD_HEIGHT_CONE:
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

        
def stop_for_face(width = None, height = None):
    if not height:
        return True
    if height >= C.STOP_THRESHOLD_HEIGHT_FACE:
        return True
    else:
        return False


def random_turn():
    global driver
    driver.publish(C.MOVE_LEFT)
    


def cv_cone_callback(msg):
    global driver,state
    d = msg.data
    if state == C.STATE_DILIVER:
        if d.height == C.ITEM_NOT_FOUND:
            random_turn()
        
        if stop_for_cone(width=d.width,height=d.height):
            driver.publish(C.STOP)
        else:
            driver.publish(get_drive_action(d.x,d.y))

def cv_face_callback(msg):
    global driver,state
    d = msg.data
    if state == C.STATE_COME:
        if d.height == C.ITEM_NOT_FOUND:
            random_turn()
            
        if stop_for_face(width=d.width,height=d.height):
            state = C.STATE_WAIT_CMD
            driver.publish(C.STOP)
        else:
            driver.publish(get_drive_action(d.x,d.y))


            
def speech_command_callback(msg):
    global state
    cmd = msg.data
    if cmd == C.COMMAND_COME_HERE:
        state = C.STATE_COME
    elif cmd == C.COMMAND_STOP:
        state = C.STATE_LISTEN
    elif cmd == C.COMMAND_DILIVER and state == C.STATE_WAIT_CMD:
        state = C.STATE_DILIVER

            

def nav():
    global driver
    rospy.init_node('nav', anonymous=True)
    driver = rospy.Publisher(C.DRIVER_ADDR, Int8, queue_size=20);

    rospy.Subscriber(C.CV_CONE_ADDR,CVMessage, cv_cone_callback)
    rospy.Subscriber(C.CV_FACE_ADDR, CVMessage, cv_face_callback)
    rospy.Subscriber(C.SPEECH_COMMAND_ADDR, CVMessage, speech_command_callback)
    
    
    rospy.spin()
    




if __name__ == '__main__':
    nav()        


