#!/usr/bin/env python3

import tf_conversions
import numpy as np
import yaml
import pygame
import time
import pygame_utils
import matplotlib.image as mpimg
from skimage.draw import disk
from scipy.linalg import block_diag
import rospy 
import os
from std_msgs.msg import String

# msgs
from geometry_msgs.msg import Transform, Pose, Quaternion, PoseStamped, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from visualization_msgs.msg import Marker

def map_callback(msg):

    width = msg.info.width
    height = msg.info.height
    resolution =msg.info.resolution 

    rospy.loginfo('the Width is: %s, The height is: %s', width, height)
    # rospy.loginfo('The Height is: %s', height)

    for i in range(width*height):
        occupancy = msg.data[i]

def logger():
    pub = rospy.Publisher('logger', String, queue_size = 10)
    rospy.init_node('log_info_node')
    rate = rospy.Rate(10)
    # map_sub = rospy.Subscriber ('map', OccupancyGrid, map_callback)
    while not rospy.is_shutdown():
        map_sub = rospy.Subscriber ('map', OccupancyGrid, map_callback)
        # rospy.loginfo(width)
        # rospy.loginfo(height)

if __name__ == '__main__':
    try: 
        logger()

    except rospy.ROSInterruptException:
        pass 

