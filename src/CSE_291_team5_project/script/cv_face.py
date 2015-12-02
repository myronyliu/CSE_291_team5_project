import cv2
import sys
import util
import os
from CSE_291_team5_project.msg import CVMessage # m = CVMessage(), m.x = 12, m.y = 232, m.height = 232; cone.publish(m)

__dirname = os.path.dirname(os.path.abspath(__file__))
cascPath = os.path.join(__dirname,'../face.xml')
faceCascade = cv2.CascadeClassifier(cascPath)



def get_face(frame):
        
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )

    if len(faces) == 0:
        return None
    else:        
        # Draw a rectangle around the faces
        mx = my = mh = mw = 0
        for (x, y, w, h) in faces:
            if w*h > mh*mw:
                mx = x
                my = y
                mh = h
                mw = w
            
        msg = CVMessage()
        shape = frame.shape
        msg.x,msg.y,msg.height = util.scaled_rect_coords(mx,my,mw,mh,shape[0],shape[1])
        return msg
        
    
