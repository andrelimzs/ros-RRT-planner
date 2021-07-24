#!/usr/bin/env python3

import sys
import yaml
from pathlib import Path

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

import cv2
import numpy as np
import matplotlib.pyplot as plt

# If mouse is pressed, True
drawing = False 
# Store start coordinates
ix,iy = -1,-1

# mouse callback function
def draw(event,x,y,flags,param):
    global ix,iy,drawing,img_temp
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            img_temp = 0 * img_temp
            cv2.line(img_temp,(ix,iy),(x,y),(255,255,255),2)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.line(img,(ix,iy),(x,y),(255,255,255),2)
        img_temp = 0 * img_temp
        

if __name__ == '__main__':
    img = np.zeros((500,500,3), np.uint8)
    img_temp = np.zeros((500,500,3), np.uint8)

    cv2.namedWindow('image')
    cv2.setMouseCallback('image',draw)
    
    while(1):
        cv2.imshow('image',img + img_temp)

        k = cv2.waitKey(1) & 0xFF
        
        # Save to file
        if k == ord('s'):
            cv2.imwrite('../resources/new_map.png', ~img)
            break

        # Stop on 'esc'
        elif k == 27:
            break

    cv2.destroyAllWindows()