#!/usr/bin/env python3

import cv2
import numpy as np

# terminal in the png folder
# blue color figures RGB=(0,100,200) or BGR=(200,100,0)
frame = cv2.imread("road_view1.png", cv2.IMREAD_COLOR)
cv2.imshow("blue circle", frame)
# Convert BGR to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# Convert blue color BGR=[200,100,0]
color = np.uint8([[[0,255,255]]]) # yelow color
hsv_color=cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
print("HSV Color= " + str(hsv_color))
# Exemple blue color figures HSV=(105 255 200)
cv2.imshow("hsv", hsv)
# define range of blue color in HSV
# Take red H range: fom 90 to 130 --> [90, 100,20] and [130, 255, 255] 
# Take S range: from 100 to 255 (for white from 0)
# Tahe V range: from 20 to 255 (for white from 0)
lower_color = np.array([28,0,255])
upper_color = np.array([33,255,255])
# Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv, lower_color, upper_color)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(frame,frame, mask= mask)

cv2.imshow('frame',frame)
cv2.imshow('mask',mask)
cv2.imshow('res',res)
cv2.waitKey(0)
    # It is for removing/deleting created GUI window from screen
    # and memory
cv2.destroyAllWindows()
