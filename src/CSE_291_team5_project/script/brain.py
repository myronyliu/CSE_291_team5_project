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



# import cv2
# cv2.startWindowThread()
# cv2.namedWindow("preview")
def camera_callback(data):
    
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # cv2.imshow('preview',cv_image)
    
    cone_msg = cv_cone.get_cone(cv_image.copy())
    face_msg = cv_face.get_face(cv_image.copy())
    if cone_msg:
        # print cone_msg
        sendCone(cone_msg)
    else:
        sendCone(not_found_msg)
    if face_msg:
        # print face_msg
        sendFace(face_msg)
    else:
        sendFace(not_found_msg)





        

    
        

def brain():
    global cone,face,speech
    rospy.init_node('brain')    
    rospy.Subscriber(C.CAMERA_ADDR,Image, camera_callback)
    cone = rospy.Publisher(C.CV_CONE_ADDR, CVMessage,queue_size=4)
    face = rospy.Publisher(C.CV_FACE_ADDR, CVMessage, queue_size=4)
    speech = rospy.Publisher(C.SPEECH_COMMAND_ADDR, Int8, queue_size=4)

    # rospy.spin()


    while(1):
        try:
            m = sr.Microphone()    
            r = sr.Recognizer()
            with m as source:
                # audio = r.listen(source)
        
                # r.adjust_for_ambient_noise(source)
                # print("Set minimum energy threshold to {}".format(r.energy_threshold))
        
            
                print "listening:"
                audio = r.listen(source)
                text = r.recognize_google(audio)
                print "=================================="
                print text
                print "=================================="
                cmd = command.parse(text)
                print "cmd is ", cmd
                if cmd:
                    print "publish cmd"
                    speech.publish(cmd)
        except:
            pass
            
    
    




if __name__ == '__main__':
    brain()
