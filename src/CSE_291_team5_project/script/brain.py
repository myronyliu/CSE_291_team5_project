#!/usr/bin/env python
import speech_recognition as sr
import rospy
import pici
import serial
import constant as C
from std_msgs.msg import Int8,Bool
from sensor_msgs.msg import Image
from CSE_291_team5_project.msg import CVMessage # m = CVMessage(), m.x = 12, m.y = 232, m.height = 232; cone.publish(m)
import cv2
from cv_bridge import CvBridge, CvBridgeError
import cv_cone,cv_face,command

global cone,face,speech
cone = face = speech = None

bridge = CvBridge()


def sendCone(m):
    global cone    
    cone.publish(m)
def sendFace(m):
    global face
    face.publish(m)


not_found_msg = CVMessage()
not_found_msg.height = C.ITEM_NOT_FOUND



    
def camera_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cone_msg = cv_cone.get_cone(cv_image)
    face_msg = cv_face.get_face(cv_image)
    if not cone_msg:        
        sendCone(cone_msg)
    else:
        sendCone(not_found_msg)
    if not face_msg:
        sendFace(face_msg)
    else:
        sendFace(not_found_msg)





r = sr.Recognizer()
while(1):    
    with sr.Microphone() as source:
        audio = r.listen(source)
    try:            
        text = r.recognize_google(audio)
        cmd = command.parse(text)        
        if not cmd:            
            speech.publish(cmd)
    except:
        pass
        

    
        

def brain():
    global cone,face,speech
    rospy.Subscriber(C.CAMERA_ADDR,Image, camera_callback)
    cone = rospy.Publisher(C.CV_CONE_ADDR, CVMessage,queue_size=20)
    face = rospy.Publisher(C.CV_FACE_ADDR, CVMessage, queue_size=20)
    speech = rospy.Publisher(C.SPEECH_COMMAND_ADDR, Int8, queue_size=20)
    





if __name__ == '__main__':
    brain()
