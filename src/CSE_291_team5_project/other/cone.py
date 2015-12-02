import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(1):
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_green = np.array([40, 20, 20], dtype=np.uint8)
    upper_green = np.array([100, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_green, upper_green).astype(np.uint8)

    kernel = np.ones((11,11), np.uint8)

    denoised = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    denoised = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    allContours, hierarchy = cv2.findContours(denoised, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_SIMPLE)

    if (len(allContours) == 0):
        continue

    contour = allContours[0]
    x, y, w, h = cv2.boundingRect(contour)
    cv2.drawContours(denoised, contour, -1, (255,0,0), 4)
    cv2.rectangle(denoised, (x, y), (x+w, y+h), (255,255,255),4)


#    cv2.imshow('raw', frame)
    cv2.imshow('mask', denoised)
#
#    if (w*h < 256):
#        print "no cone"
#    else:
#	    xMid =  2.0*(x + (w / 2.0)) / cv_image.cols - 1.0
#	    yMid = -2.0*(y + (h / 2.0)) / cv_image.rows
#	    msg = CVMessage()
#	    msg.x = xMid
#	    msg.y = yMid
#	    msg.height = h

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
