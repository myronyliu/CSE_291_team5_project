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




def camera_callback(data):
    global cone,gesture,waving  # just use cone.publish()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # TODO:
    # analyze the cv_image
    pass




def brain():
    global cone,gesture,waving
    rospy.Subscriber(C.CAMERA_ADDR,Image, camera_callback)
    cone = rospy.Publisher(C.CV_CONE_ADDR, CVMessage,queue_size=20)
    gesture = rospy.Publisher(C.CV_GESTURE_ADDR, Int8,queue_size=20)
    waving = rospy.Publisher(C.CV_WAVING_ADDR,CVMessage,queue_size=20)
    





if __name__ == '__main__':
    brain()
