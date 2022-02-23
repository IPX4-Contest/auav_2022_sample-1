#!/usr/bin/env python3

from math import dist
from turtle import distance
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import statistics as S


from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

class find_pos_rgb:

    def init(self, data):
        self.rgb_array = data 

    def checkR(pixelr):
        if pixelr <= 100:
            return 1
        return 0
    def checkG(pixelg):
        if pixelg <= 100:
            return 1
        return 0
    def checkB(pixelb):
        if pixelb >= 100:
            return 1
        return 0
    
    def findred(self.rgb_array):
        shape = rgb[0].shape
        row = shape[0]
        col = shape[1]
        red = rgb[0]
        green = rgb[1]
        blue = rgb[2]
        redDeter = np.zeros((row, col))
        for i in range(row):
            for j in range(col):
                r = checkR(red[i][j])
                g = checkG(green[i][j])
                b = checkB(blue[i][j])
                if (r & g & b) == 1:
                    redDeter[i][j] = 1
        # print(redDeter)
        return redDeter
    
    def findCenter(findred(self.rgb_array)):
        row = len(matrix)
        col = len(matrix[0])
        arr = np.where(matrix == 1)
        print(arr)
        mean_row = S.mean(arr[0])
        mean_col = S.mean(arr[1])
        center = [mean_row, mean_col]
        return center