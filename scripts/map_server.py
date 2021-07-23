#!/usr/bin/env python3

import sys
import yaml
from pathlib import Path

import rospy
from nav_msgs.msg import OccupancyGrid

def map_server():
    # Read map
    

    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
    
    rospy.init_node('map_server', anonymous=False)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        map = OccupancyGrid()
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
        filename = (Path(sys.argv[1]).parent / Path(config['image'])).resolve()

        print(f"Loading map from {filename}")

    try:
        map_server(filename)
    except rospy.ROSInterruptException:
        pass