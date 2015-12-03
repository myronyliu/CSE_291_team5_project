import cv2
import sys
import constant as C
import util
import os
from CSE_291_team5_project.msg import CVMessage # m = CVMessage(), m.x = 12, m.y = 232, m.height = 232; cone.publish(m)

__dirname = os.path.dirname(os.path.abspath(__file__))
cascPath = os.path.abspath(os.path.join(__dirname,'../other/face.xml'))
faceCascade = cv2.CascadeClassifier(cascPath)

print cascPath



if C.Debug:    
    cv2.startWindowThread()
    cv2.namedWindow("face")


def get_face(frame):
    # print frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )

    
    if len(faces) == 0:
        if C.Debug:
            # print frame
            cv2.imshow('face', frame)            
        return None
    else:        
        # Draw a rectangle around the faces
        mx = my = mh = mw = 0
        mh = 1
        mw = 256
        for (x, y, w, h) in faces:            
            if w*h > mh*mw:
                if C.Debug:                
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                mx = x
                my = y
                mh = h
                mw = w
                
        if C.Debug:
            # print frame
            cv2.imshow('face', frame)            
        if mx == my and mx == 0:
            return None
        
        msg = CVMessage()
        shape = frame.shape
        msg.x,msg.y,msg.width,msg.height = util.scaled_rect_coords(mx,my,mw,mh,shape[0],shape[1])
        if C.Debug:            
            # print "face height: ",msg.height
            pass
            
        return msg
        
    
