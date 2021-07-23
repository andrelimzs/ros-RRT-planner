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

def map_server(filename):
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=1, latch=True)
    rospy.init_node('map_server', anonymous=False)

    print(f"Reading {filename}")

    # Read map
    map_img = cv2.imread(filename, 0)

    # Normalise
    map_img = (100 - map_img/255*100)
    # OccupancyGrid is int8
    map_img = map_img.astype(np.int8)

    # [DEBUG]
    # cv2.imshow("Map", map_img)
    # cv2.waitKey(0)

    map = OccupancyGrid()
    map.info.map_load_time = rospy.get_rostime()
    # Placeholder resolution
    map.info.resolution = 0.1 
    map.info.width = map_img.shape[0]
    map.info.height = map_img.shape[1]
    # Center map on origin
    map.info.origin.position.x = map.info.width * map.info.resolution / 2
    map.info.origin.position.y = map.info.height * map.info.resolution / 2
    
    # Flatten because OccupanyGrid data is a 1D array
    map.data = map_img.flatten()
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(map)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage - map_file")
    else:
        # Read config file and extract map_file
        with open(sys.argv[1], 'r') as stream:
            try:
                config = yaml.safe_load(stream)
            except yaml.YAMLError as inst:
                print(inst)

        # Combine abs file path and rel path from config
        filename = str((Path(sys.argv[1]).parent / Path(config['image'])).resolve())

    try:
        map_server(filename)
    except rospy.ROSInterruptException:
        pass