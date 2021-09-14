#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import controller_manager_msgs.srv
import rospy
import math
import time
import datetime

import trajectory_msgs.msg
import actionlib
import control_msgs.msg
import geometry_msgs.msg
import os
import sys
import signal

from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Int32
import numpy as np

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
from cv_bridge import CvBridge, CvBridgeError
import cv2
cv_img=None
bridge=CvBridge()

def signal_handler(signal,frame):
    print("EXIT!")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

rospy.init_node('datareceiver')
tfbuffer=tf2_ros.Buffer(rospy.Duration(10.0))
tflistener=tf2_ros.TransformListener(tfbuffer)

pub_depth = rospy.Publisher('/pnu/depth_rect_raw',Image, queue_size=1)
pub_rgb = rospy.Publisher('/pnu/rgb_rect_raw_input',Image, queue_size=1)



# wait to establish connection between the controller
# while pub_depth.get_num_connections() == 0: rospy.sleep(0.1)
# print "DEPTH CONNECTED"
# while pub_rgb.get_num_connections() == 0:  rospy.sleep(0.1)
# print "RGB CONNECTED"

def depth_callback(data):
    global rgb_msg,rgb_count
    global cv_img
    # print("Encoding",data.format)
    # print("RGB size:", sys.getsizeof(data.data))
    if sys.getsizeof(data.data)>1000:
      np_arr = np.fromstring(data.data, np.uint8)
      image_np = cv2.imdecode(np_arr, 0) #0:8 bit grey
      #we map 0-255 to 500mm - 2550mm
      # print "depth jpg:", np_arr.shape
      # print "decode jpg:",image_np.shape
      image_np16 = image_np.astype(np.uint16)*10+500
      image_np16_0 = 1-np.clip(image_np.astype(np.uint16),0,1)
      image_np16_255 = image_np.astype(np.uint16)/255
      image_np16 = image_np16 - 3050*image_np16_255-500*image_np16_0

      depth_msg=Image()
      depth_msg.header=data.header
      depth_msg.width=424
      depth_msg.height=240
      # depth_msg.encoding="mono16"
      depth_msg.encoding="16UC1"
      depth_msg.step = 424*2
      depth_msg.data=image_np16.tostring()
      pub_depth.publish(depth_msg)
      print("Depth: packed %d KB org %d KB" % (sys.getsizeof(data.data)/1024,sys.getsizeof(image_np16)/1024))

def rgb_callback(data):
    global rgb_msg,rgb_count
    global cv_img
    # print("Encoding",data.format)
    if sys.getsizeof(data.data)>1000:
      np_arr = np.fromstring(data.data, np.uint8)
      image_np = cv2.imdecode(np_arr, 1)
      img_msg = bridge.cv2_to_imgmsg(image_np, "bgr8")
      img_msg.header=data.header
      img_msg.width=424
      img_msg.height=240
      pub_rgb.publish(img_msg)
      print("RGB: packed %d KB org %d KB" % (sys.getsizeof(data.data)/1024,sys.getsizeof(image_np)/1024))


rospy.Subscriber("/pnu/depth_rect_compressed", CompressedImage, depth_callback)
rospy.Subscriber("/pnu/rgb_rect_compressed", CompressedImage,rgb_callback)


last_time = time.time()
t_last_send=last_time
t_last_send_pcl=last_time+0.5
send_interval = 1.0
send_interval_pcl = 3.0

rospy.spin()
