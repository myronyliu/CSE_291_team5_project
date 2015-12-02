#!/usr/bin/env python
import rospy
import pici
import serial
import constant as C
from std_msgs.msg import Int8,Bool
from sensor_msgs.msg import Image
from CSE_291_team5_project.msg import CVMessage # m = CVMessage(), m.x = 12, m.y = 232, m.height = 232; cone.publish(m)
import cv2
from cv_bridge import CvBridge, CvBridgeError

global cone,gesture,waving

bridge = CvBridge()


def sendCone(x=None,y=None,height=None):
    global cone
    m = CVMessage()
    m.x = x
    m.y = y
    m.height = height
    cone.publish(m)

def sendWaving(x=None,y=None,height=None):
    global waving
    m = CVMessage()
    m.x = x
    m.y = y
    m.height = height
    waving.publish(m)


def sendGesture(x):
    global gesture
    gesture.publish(x)

def scaled_rect_coords(px, py, pw, ph, nRows, nCols):
    xMid =  2.0*(px + pw/2.0)/nRows - 1.0
    yMid = -2.0*(py + ph/2.0)/nCols + 1.0
    return (xMid, yMid, ph / nRows)


def get_cone(cv_image):

    msg = CVMessage()
    msg.x = 0
    msg.y = 0
    msg.height = 0

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_green = np.array([40, 20, 0], dtype=np.uint8)
    upper_green = np.array([100, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_green, upper_green).astype(np.uint8)

    kernel = np.ones((11,11), np.uint8)

    denoised = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    denoised = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    allContours, hierarchy = cv2.findContours(denoised, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_SIMPLE)

    if len(allContours) == 0:
        return msg

    largestContour = allContours[0]
    largestArea = cv2.contourArea(largestContour)

    for i in range (1, len(allContours)):
        area = cv2.contourArea(allContours[i])
        if area > largestArea:
            largestContour = allContours[i]
            largestArea = area

    x, y, w, h = cv2.boundingRect(largestContour)

    #cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 4)
    #cv2.imshow('raw', hsv)

    if (w*h > 256):
        nRows_nCols = cv_image.shape
        msg.x, msg.y, msg.height = scaled_rect_coords(x, y, w, h, nRows_nCols[0], nRows_nCols[1])

    return msg;

def get_gesture(cv_image):
    

def camera_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	cone_msg = get_cone(cv_image)
	sendCone(cone_msg.x,cone_msg.y,cone_msg.height)
    get_gesture(cv_image)

def brain():
    global cone,gesture,waving
    rospy.Subscriber(C.CAMERA_ADDR,Image, camera_callback)
    cone = rospy.Publisher(C.CV_CONE_ADDR, CVMessage,queue_size=20)
    gesture = rospy.Publisher(C.CV_GESTURE_ADDR, Int8,queue_size=20)
    waving = rospy.Publisher(C.CV_WAVING_ADDR,CVMessage,queue_size=20)
    





if __name__ == '__main__':
    brain()
