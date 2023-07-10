import cv2
import numpy as np

def colourDetection(frame,colour):
    #convrt to opencv bgr
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    #convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #define range of colour in hsv
    if colour == "red":
        lower = np.array([0,50,50])
        upper = np.array([10,255,255])
    elif colour == "green":
        lower = np.array([50,100,100])
        upper = np.array([70,255,255])
    elif colour == "blue":
        lower = np.array([110,50,50])
        upper = np.array([130,255,255])
    #threshold hsv image to get only colour
    mask = cv2.inRange(hsv, lower, upper)
    #bitwise and mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #display frame
    #cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
    #check if user pressed 'q'
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return res

def contourDetection(frame,original):
    # copy frame
    original = original.copy()
    # convert to grayscale
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # draw all contours
    cv2.drawContours(original, contours, -1, (0, 255, 0), 3)
    # find center of contour
    M = cv2.moments(contours[0])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    # draw center of contour
    cv2.circle(original, (cx, cy), 4, (0, 0, 255), -1)
    return cx,cy,original