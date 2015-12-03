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
        # driver.publish(C.MOVE_RIGHT)
        random_turn('right')
    elif x <= -C.STRAIGHT_THRESHOLD_X:
        # driver.publish(C.MOVE_LEFT)
        random_turn('left')
    else:        
        driver.publish(C.MOVE_UP)

        
def stop_for_face(width = None,height = None):
    if not height:
        return True
    if height >= C.STOP_THRESHOLD_HEIGHT_FACE:
        return True
    else:
        return False

import random
def random_turn(direction = None):
    global driver
    # driver.publish(C.STOP)
    if not direction:        
        if random.randint(1,10) == 1:        
            driver.publish(C.MOVE_LEFT)
        else:
            driver.publish(C.STOP)
    elif direction == "right":
        if random.randint(1,10) <= 5:        
            driver.publish(C.MOVE_RIGHT)
        else:
            driver.publish(C.STOP)
    elif direction == "left":        
        if random.randint(1,10) <= 5:        
            driver.publish(C.MOVE_LEFT)
        else:
            driver.publish(C.STOP)


def cv_cone_callback(d):
    global driver,state    
    if state == C.STATE_DELIVER:
        if d.height == C.ITEM_NOT_FOUND:
            print "no cone"
            random_turn()
            return
            # random_turn()
            
        if stop_for_cone(width=d.width,height=d.height):
            print "close to the cone"
            state = C.STATE_LISTEN
            driver.publish(C.STOP)
        else:
            driver.publish(get_drive_action(d.x,d.y))

def cv_face_callback(d):
    global driver,state
    if state == C.STATE_COME:        
        if d.height == C.ITEM_NOT_FOUND:
            print "no face"
            random_turn()
            return
            # random_turn()
            
        if stop_for_face(width=d.width,height=d.height):
            print "state = wait_for_command"
            state = C.STATE_WAIT_CMD
            driver.publish(C.STOP)
        else:
            driver.publish(get_drive_action(d.x,d.y))


            
def speech_command_callback(msg):
    global state,driver
    print "==============nav.py================="
    print "recieve msg"
    
    cmd = msg.data
    if cmd == C.COMMAND_COME_HERE:
        print "state = comming"
        state = C.STATE_COME
    elif cmd == C.COMMAND_STOP:
        driver.publish(C.STOP)
        print "state = stop"
        state = C.STATE_LISTEN
    elif cmd == C.COMMAND_DELIVER and state == C.STATE_WAIT_CMD:
        print "state = deliver"                
        state = C.STATE_DELIVER


        

def nav():
    global driver
    rospy.init_node('nav')
    driver = rospy.Publisher(C.DRIVER_ADDR, Int8, queue_size=20);

    rospy.Subscriber(C.CV_CONE_ADDR,CVMessage, cv_cone_callback)
    rospy.Subscriber(C.CV_FACE_ADDR, CVMessage, cv_face_callback)
    rospy.Subscriber(C.SPEECH_COMMAND_ADDR, Int8, speech_command_callback)
    
    
    rospy.spin()
    




if __name__ == '__main__':
    nav()        


