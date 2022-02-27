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

class Find_pos_goal:

    def __init__(self, centerRed, drone_pose_x, drone_pose_y, drone_pose_z):
        self.dist_from_pointcloud = 1

        self.eg_per_pix = 120/640   # deg/pixels
        self.hor_pix_from_center = centerRed[0] - 320
        self.hor_degree = deg_per_pix*hor_pix_from_center
        self.hor_rad = hor_degree*(3.14159/180.0)

        self.delta_x = np.cos(hor_rad)*dist_from_pointcloud
        self.delta_y = -np.sin(hor_rad)*dist_from_pointcloud

        self.depth_array

        #goal_rel_pose message initialization
        self.pose.header.seq = 3
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = "map"

        self.drone_pose_x = drone_pose_x
        self.drone_pose_y = drone_pose_y
        self.drone_pose_z = drone_pose_z

    def pos_goal(self):
        # self.x_manual += self.x_step
        # pose.pose.position.x = self.x_manual
        self.pose.pose.position.x = self.drone_pose_x + delta_x
        self.pose.pose.position.y = self.drone_pose_y + delta_y
        self.pose.pose.position.z = 0.0

        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 0.0
        self.rate = rospy.Rate(1) # 0.2hz
        self.rate.sleep()

        print("drone_x: %.2f\ndrone_y: %.2f\ndrone_z: %.2f\n" % (self.drone_pose_x, self.drone_pose_y, self.drone_pose_z))
        print("diff_x: %.2f\ndiff_y: %.2f\n" % (delta_x, delta_y))

        return self.pose

