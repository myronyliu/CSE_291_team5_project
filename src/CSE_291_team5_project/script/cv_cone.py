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
import numpy as np
import util



if C.Debug:    
    cv2.startWindowThread()
    cv2.namedWindow("cone")



def get_cone(cv_image):    
    msg = CVMessage()
    msg.x = 0
    msg.y = 0
    msg.height = 0
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_green = np.array([60, 50, 0], dtype=np.uint8)
    upper_green = np.array([100, 200, 200], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_green, upper_green).astype(np.uint8)
    kernel = np.ones((11,11), np.uint8)
    denoised = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    denoised = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    

    allContours, hierarchy = cv2.findContours(denoised, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_SIMPLE)

    if len(allContours) == 0:
        if C.Debug:            
            cv2.imshow('cone',cv_image)
        return None
    else:        
        largestContour = allContours[0]
        largestArea = cv2.contourArea(largestContour)

        for i in range (1, len(allContours)):
            area = cv2.contourArea(allContours[i])
            if area > largestArea:
                largestContour = allContours[i]
                largestArea = area

        x, y, w, h = cv2.boundingRect(largestContour)
        
        if C.Debug:            
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 4)
            cv2.imshow('cone',cv_image)
        if (w*h > 512):
            nRows_nCols = cv_image.shape
            msg.x, msg.y,msg.width, msg.height = util.scaled_rect_coords(x, y, w, h, nRows_nCols[0], nRows_nCols[1])        
        else:
            return None
    
        return msg;
