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

from find_pos_rgb import Find_pos_rgb
from find_pos_goal import Find_pos_goal


class view_gazebo_camera:

  def __init__(self):
    self.x_manual = -5.0
    self.x_step = 1.0

    self.rgb_array = np.empty
    self.depth_array = np.empty
    self.drone_pose_x = 0.0
    self.drone_pose_y = 0.0
    self.drone_pose_z = 0.0

    self centerRed = [0,0]

    self.image_sub = rospy.Subscriber("/drone/camera/rgb/image_raw", Image, self.img_callback)
    self.drone_pose = rospy.Subscriber("/drone/mavros/local_position/pose", PoseStamped, self.pose_callback)
    # self.image_depth_sub = rospy.Subscriber("/drone/camera/depth/image_raw", Image, self.depth_callback)
    self.image_depth_sub = rospy.Subscriber("/drone/camera/depth/points", PointCloud2, self.depth_callback, queue_size=1, buff_size=52428800)
    self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

#update the image 
def img_callback(self,data):
    bridge = CvBridge()
    pose = PoseStamped()

    try:
      image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
      
    self.rgb_array = np.array(image,dtype=np.uint8)
    print(self.rgb_array.shape)

    #print(self.depth_array)

    find_center = Find_pos_rgb(rgb_array)
    #matrixRed = find_pos_rgb.findred  # rgb: numpy array of
    self.centerRed = find_center.findCenter()

    #PointCLoud = self.depth_array(#insert the index collected from center red here)
    #then change the pose to corresond to PC

    print(centerRed)
    # print(self.drone_pose.pose.position.x)
    # print(self.drone_pose.pose.position.y)
    # print(self.drone_pose.pose.position.z)

#update current drone position
def pose_callback(self, data):
  self.drone_pose_x = data.pose.position.x
  self.drone_pose_y = data.pose.position.y
  self.drone_pose_z = data.pose.position.z

#publish the goal position to the topic
def depth_callback(self, data):
    print('working')
    points_list = []

    for data in pc2.read_points(data, skip_nans=True):
      points_list.append([data[0], data[1], data[2], data[3]])

    self.depth_array = np.array(points_list)
    print(self.depth_array.shape)

    find_goal = Find_pos_goal(self.centerRed, drone_pose_x, drone_pose_y, drone_pose_z)
    pose = find_goal.pos_goal()

    self.pub.publish(pose)
    '''
      bridge = CvBridge()
      try:
        # The depth image is a single-channel float32 image
        depth_image = bridge.imgmsg_to_cv2(data,"32FC1")
      except CvBridgeError as e:
        rospy.logerr(e)
      
      self.depth_array = np.array(depth_image, dtype=np.float32)
    '''
  
def main():
  view_gazebo_camera()
  

  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
if __name__ == '__main__':
  rospy.init_node('view_gazebo_camera', anonymous=False)
  main()

    #self.image_depth_pub.publish(depth_array)
    #cv2.imshow("Camera output depth", depth_image)
    #cv2.waitKey(3)


    #print(depth_array.shape)

    #self.image_pub.publish(numpy_array)
    #cv2.imshow("Camera output rgb", image)
    #cv2.waitKey(3)

        #print(numpy_array.shape)


            #numpy_array = np.array(image, dtype=np.uint8)