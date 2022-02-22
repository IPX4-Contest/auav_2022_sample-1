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

# import ros_numpy

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
  
def findred(rgb):
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
  
def findCenter(matrix):
    row = len(matrix)
    col = len(matrix[0])
    arr = np.where(matrix == 1)
    print(arr)
    mean_row = S.mean(arr[0])
    mean_col = S.mean(arr[1])
    center = [mean_row, mean_col]
    return center

class view_gazebo_camera:

  def __init__(self):
    self.x_manual = -5.0
    self.x_step = 1.0

    self.rgb_array = np.empty
    self.depth_array = np.empty
    self.drone_pose_x = 0.0
    self.drone_pose_y = 0.0
    self.drone_pose_z = 0.0

    self.image_sub = rospy.Subscriber("/drone/camera/rgb/image_raw", Image, self.callback)
    # self.image_depth_sub = rospy.Subscriber("/drone/camera/depth/image_raw", Image, self.depth_callback)
    self.image_depth_sub = rospy.Subscriber("/drone/camera/depth/points", PointCloud2, self.depth_callback, queue_size=1, buff_size=52428800)
    self.drone_pose = rospy.Subscriber("/drone/mavros/local_position/pose", PoseStamped, self.pose_callback)

    self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

  def depth_callback(self, data):
    print('working')
    points_list = []

    for data in pc2.read_points(data, skip_nans=True):
      points_list.append([data[0], data[1], data[2], data[3]])

    self.depth_array = np.array(points_list)
    print(self.depth_array.shape)

    '''
      bridge = CvBridge()
      try:
        # The depth image is a single-channel float32 image
        depth_image = bridge.imgmsg_to_cv2(data,"32FC1")
      except CvBridgeError as e:
        rospy.logerr(e)
      
      self.depth_array = np.array(depth_image, dtype=np.float32)
    '''

  def pose_callback(self, data):
    self.drone_pose_x = data.pose.position.x
    self.drone_pose_y = data.pose.position.y
    self.drone_pose_z = data.pose.position.z
  
  def callback(self,data):
    bridge = CvBridge()
    pose = PoseStamped()

    try:
      image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
      

    self.rgb_array = np.array(image,dtype=np.uint8)
    print(self.rgb_array.shape)

    #print(self.depth_array)


    matrixRed = findred(self.rgb_array)  # rgb: numpy array of
    centerRed = findCenter(matrixRed)

    #PointCLoud = self.depth_array(#insert the index collected from center red here)
    #then change the pose to corresond to PC

    print(centerRed)
    # print(self.drone_pose.pose.position.x)
    # print(self.drone_pose.pose.position.y)
    # print(self.drone_pose.pose.position.z)

    dist_from_pointcloud = 1

    deg_per_pix = 120/640   # deg/pixels
    hor_pix_from_center = centerRed[0] - 320
    hor_degree = deg_per_pix*hor_pix_from_center
    hor_rad = hor_degree*(3.14159/180.0)

    delta_x = np.cos(hor_rad)*dist_from_pointcloud
    delta_y = -np.sin(hor_rad)*dist_from_pointcloud

    # Create ros msg to be sent to /move_base_simple/goal
    pose.header.seq = 3
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"

    # self.x_manual += self.x_step
    # pose.pose.position.x = self.x_manual
    pose.pose.position.x = self.drone_pose_x + delta_x
    pose.pose.position.y = self.drone_pose_y + delta_y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    self.pub.publish(pose)
    rate = rospy.Rate(1) # 0.2hz
    rate.sleep()

    print("drone_x: %.2f\ndrone_y: %.2f\ndrone_z: %.2f\n" % (self.drone_pose_x, self.drone_pose_y, self.drone_pose_z))
    print("diff_x: %.2f\ndiff_y: %.2f\n" % (delta_x, delta_y))



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