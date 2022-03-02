#!/usr/bin/env python3

from math import dist
from turtle import distance

from matplotlib import image
from rosgraph import myargv
import rospy
import ros_numpy
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
import scipy.misc

from find_pos_rgb import Find_pos_rgb
from find_rel_pos import Find_pos_goal


class view_gazebo_camera:

  def __init__(self):

    self.rgb_array = np.empty
    self.depth_array = np.empty
    self.drone_pose_x = 0.0
    self.drone_pose_y = 0.0
    self.drone_pose_z = 0.0

    self.centerRed = [0,0]
    self.bridge = CvBridge()

    self.image_sub = rospy.Subscriber("/drone/camera/rgb/image_raw", Image, self.img_callback)
    self.drone_pose = rospy.Subscriber("/drone/mavros/local_position/pose", PoseStamped, self.pose_callback)
    # self.image_depth_sub = rospy.Subscriber("/drone/camera/depth/image_raw", Image, self.depth_callback)
    self.image_depth_sub = rospy.Subscriber("/drone/camera/depth/points", PointCloud2, self.depth_callback, queue_size=1, buff_size=52428800)
    self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    self.img_pub = rospy.Publisher('/move_base_simple/cv_image', Image, queue_size=10)

  def draw_crosshair(self):
    my_array = self.rgb_array
    centerx = self.centerRed[0]
    centery = self.centerRed[1]
    
    for lcv in range(centerx-5,centerx+5):
      my_array[lcv, centery, 1] = 250

    for lcv in range(centery-5,centery+5):
      my_array[centerx, lcv, 1] = 250

    # vis = np.ones((384, 836), np.float32)
    # vis = np.random.randint(0,250,(384,836),dtype=np.uint8)

    vis2 = cv2.cvtColor(my_array, cv2.COLOR_RGB2RGBA)
    vis2 = cv2.cvtColor(vis2, cv2.COLOR_RGBA2RGB)
    image_message = self.bridge.cv2_to_imgmsg(vis2, encoding="passthrough")

    self.img_pub.publish(image_message)






  #update the image 
  def img_callback(self,data):
      bridge = CvBridge()
      pose = PoseStamped()

      try:
        image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
      except CvBridgeError as e:
        rospy.logerr(e)
        
      self.rgb_array = np.array(image,dtype=np.uint8)
      # print(f"self.rgbarray.shape: {self.rgb_array.shape}")

      #print(self.depth_array)

      find_center = Find_pos_rgb(self.rgb_array)
      #matrixRed = find_pos_rgb.findred  # rgb: numpy array of
      self.centerRed = find_center.findCenter()

      #PointCLoud = self.depth_array(#insert the index collected from center red here)
      #then change the pose to corresond to PC

      # print(f"self.centerRed: {self.centerRed}")
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
      # points_list = []

      xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
      # print(xyz_array)
      # print(f"shape: {xyz_array.shape}")

      points_list = xyz_array

      ## x: 0, y: 1, z: 2
      #for elem in pc2.read_points(data, skip_nans=True):
      #  points_list.append(list(elem))
      ## print(f"point_list type: {type(points_list)}")

      # points_list = np.asarray(points_list)

      points_list_mod = points_list.reshape(480, 640, 3)
      # print(f"points_list_mod.shape {points_list_mod.shape}")
      # print(f"sample data: {points_list_mod[1,1,:]}")




      find_goal = Find_pos_goal(self.centerRed, self.drone_pose_x, self.drone_pose_y, self.drone_pose_z, points_list_mod)
      pose = find_goal.pos_goal()

      self.draw_crosshair()
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
    ##cv2.imshow("Camera output rgb", image)
    #cv2.waitKey(3)

        #print(numpy_array.shape)


            #numpy_array = np.array(image, dtype=np.uint8)