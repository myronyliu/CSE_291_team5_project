import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def scaled_rect_coords(px, py, pw, ph, nCols, nRows):
    xMid =  2.0*(px + pw/2.0)/nRows - 1.0
    yMid = -2.0*(py + ph/2.0)/nCols + 1.0
    return (xMid, yMid, (ph + 0.0)/ nRows)


while True:
    ret, frame = cap.read()    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_green = np.array([60, 50, 0], dtype=np.uint8)
    upper_green = np.array([100, 200, 200], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_green, upper_green).astype(np.uint8)

    kernel = np.ones((11,11), np.uint8)

    denoised = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    denoised = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    allContours, hierarchy = cv2.findContours(denoised, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_SIMPLE)
    
    

    
    if len(allContours) == 0:
        cv2.imshow('raw', frame)
    else:
        
        largestContour = allContours[0]
        largestArea = cv2.contourArea(largestContour)

        for i in range (1, len(allContours)):
            area = cv2.contourArea(allContours[i])
            if area > largestArea:
                largestContour = allContours[i]
                largestArea = area

        x, y, w, h = cv2.boundingRect(largestContour)
        #cv2.drawContours(denoised, largestContour, -1, (255,0,0), 4)

    

        if (w*h < 1024):
            # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 4)
            cv2.imshow('raw',frame)
            pass
        else:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 4)
            cv2.imshow('raw',frame)
            nRows_nCols = frame.shape
            print nRows_nCols
            (px, py, pheight) = scaled_rect_coords(x, y, w, h, nRows_nCols[0],nRows_nCols[1])
            print px,py,pheight


        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

cap.release()
cv2.destroyAllWindows()
