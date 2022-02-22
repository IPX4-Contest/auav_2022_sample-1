#!/usr/bin/env python3


## inspiration from ros.answer.org

from asyncore import ExitNow
import rospy
import sys

import cv2 as C
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class view_gazebo_camera:

    
    def __init__(self):
        print("working so far")
        rospy.init_node("view_gazebo_camera")



        #creates window for image
        C.namedWindow('Image RBG', C.WINDOW_NORMAL)
        C.moveWindow('Image RBG', 25, 350)


        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/drone/camera/rgb/image_raw", Image,self.image_callback)
        #self.image_sub = rospy.Subscriber("/drone/camera/rgb/image_color", Image,self.image_callback)
        ##self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image,self.image_callback)
        #self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image,self.depth_callback)
        rospy.loginfo("no image topics")
        
    def image_callback(self, ros_image):

        print("workign inside call back")

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgsg_to_cv(ros_image, "bgr8")
        except CvBridgeError:
            print("CvBridgeError!!!!!!!!!!!!!!")
        # Convert the image to a Numpy array since most cv2 functions

        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)

        # Process the frame using the process_image() function
        display_image = self.process_image(frame)

        # Display the image.
        C.imShow('imagergb', display_image)

        # Process any keyboard commands
        self.keystroke = C.WaitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")


if __name__ == '__main__':

    view_gazebo_camera()
    rospy.spin()