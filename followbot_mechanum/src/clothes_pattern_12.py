#!/usr/bin/env python
import rospy

from openpose_ros_msgs.msg import OpenPoseHumanList
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import sys
import cv2
import numpy as np
from PIL import Image as pil_image
import glob
import rospkg
import pyrealsense2 as rs2

from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import tf2_ros

from math import pow, sqrt
import time

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('followbot_mechanum')
clothes_color = 'blue'

# Use clothes area and face area together.
# Get depth inform if there is face area.

class clothes_pattern:

	def __init__(self):
		self.bridge = CvBridge()
		self.person_clothes_hists = []
		for filename in glob.glob(pkg_path + '/src/person_clothes_img/' + clothes_color + '/*.png'):
			im = pil_image.open(filename)
			im_cv = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2HSV)
			hist = cv2.calcHist([im_cv],[0],None,[64],[0,256])
			hist = cv2.normalize(hist, hist).flatten()
			self.person_clothes_hists.append(hist)

		self.frame = None

		# camera
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		self.clothes_similarity = 0.95
		self.clothes_depth_arr_position = 0.1
		self.img_openpose_pub = rospy.Publisher('/camera/color/image_raw_openpose', Image, queue_size=1)

		self.intrinsics = None
		self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)

		self.valid_size_of_person = 30 # image pixels distance

		self.img_processing = False
		self.personPose_pub = rospy.Publisher('person_pose_clothes', PoseStamped, queue_size=1)

		self.depth_msg = None
		self.depth_sub = rospy.Subscriber('/pnu/depth_rect_raw', Image, self.depth_callback)
		self.rgb_msg = None
		self.rgb_sub = rospy.Subscriber('/pnu/rgb_rect_raw_input', Image, self.rgb_callback)

		self.openpose_sub = rospy.Subscriber('/openpose_ros/human_list', OpenPoseHumanList, self.openpose_callback)


	def depth_callback(self, msg):
		self.depth_msg = msg

	def rgb_callback(self, msg):
		self.rgb_msg = msg


	def openpose_callback(self, openpose_msg):

		if not self.img_processing:
			self.img_processing = True
			depth_msg = self.depth_msg
			rgb_msg = self.rgb_msg

			try:
				time_ = depth_msg.header.stamp
			except:
				self.img_processing = False
				return

			try:
				camera_transform = self.tf_buffer.lookup_transform('map', 'camera_link', time_)
			except:
				self.img_processing = False
				return

			# Image
			cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")

			# Depth Image
			depth_img = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)

			self.callback_humanlist(openpose_msg, cv_image, depth_img, camera_transform, time_)
			self.img_processing = False


	def imageDepthInfoCallback(self, cameraInfo):
		try:
			if self.intrinsics:
				return
			self.intrinsics = rs2.intrinsics()
			self.intrinsics.width = cameraInfo.width
			self.intrinsics.height = cameraInfo.height
			self.intrinsics.ppx = cameraInfo.K[2]
			self.intrinsics.ppy = cameraInfo.K[5]
			self.intrinsics.fx = cameraInfo.K[0]
			self.intrinsics.fy = cameraInfo.K[4]

			if cameraInfo.distortion_model == 'plumb_bob':
				self.intrinsics.model = rs2.distortion.brown_conrady
			elif cameraInfo.distortion_model == 'equidistant':
				self.intrinsics.model = rs2.distortion.kannala_brandt4
				self.intrinsics.coeffs = [i for i in cameraInfo.D]
		except CvBridgeError as e:
			print(e)
			return


	def callback_humanlist(self, human_list_msg, cv_image, openpose_depth_img, camera_transform, time_):

		depth_img = openpose_depth_img
		frame = cv_image
		found_target = False

		for human in human_list_msg.human_list:

			# clothes area
			main_points = [1,2,5,8]
			x = []
			y = []

			for idx in main_points:
				if human.body_key_points_with_prob[idx] != 0:
					if human.body_key_points_with_prob[idx].x != 0 and human.body_key_points_with_prob[idx].y != 0:
						x.append(human.body_key_points_with_prob[idx].x)
						y.append(human.body_key_points_with_prob[idx].y)

			if len(x) != 4:
				continue

			#(top_, right_, bottom_, left_) = (int(min(y)), int(max(x)), int(max(y)), int(min(x)))
			(top, right, bottom, left) = (int((3*min(y)+max(y))/4), int((min(x)+3*max(x))/4), int((min(y)+3*max(y))/4), int((3*min(x)+max(x))/4))

			diagonal_size = sqrt(pow((right-left), 2) + pow((top-bottom), 2))

			# neglect unvalid small wrong person object from openpose
			if diagonal_size < self.valid_size_of_person:
				continue

			col = int(round((left + right)/2))
			row = int(round((bottom + top)/2))
			clothes_block = frame[top:bottom, left:right]
			depth_arr = depth_img[top:bottom, left:right]

			cv2.rectangle(frame, (left, bottom), (right, top), (0, 0, 255), 2)
			# BGR 2 HSV
			try:
				im_cv = cv2.cvtColor(np.array(clothes_block), cv2.COLOR_BGR2HSV)
			except:
				break
			hist = cv2.calcHist([im_cv],[0],None,[64],[0,256])
			hist = cv2.normalize(hist, hist).flatten()

			pose_transformed = None

			# select the depth located in (self.clothes_depth_arr_position)(ex 2/3), because some obstacle can cover person's clothes in front of person.
			depth_list = []
			for i in range(0, len(depth_arr)):
				for j in range(0, len(depth_arr[0])):
					if depth_arr[i][j] != 0:
						depth_list.append(depth_arr[i][j])

			if len(depth_list) == 0:
				continue

			sorted_depth_list = sorted(depth_list, reverse=False)
			selected_idx = int(len(sorted_depth_list)*self.clothes_depth_arr_position)
			selected_depth = sorted_depth_list[selected_idx]

			# personPose : [-y, x, z] by camera_link frame unit : mm
			# data_to_send.data = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], selected_depth) # [col, row]
			rs2_pose = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], selected_depth) # [col, row]

			pose_stamped = PoseStamped()

			pose_stamped.header.frame_id = 'camera_link'
			# personPose : [-y, x, z] unit : mm
			pose_stamped.pose.position.x = rs2_pose[1]/1000
			pose_stamped.pose.position.y = -rs2_pose[0]/1000
			pose_stamped.pose.position.z = rs2_pose[2]/1000
			pose_stamped.pose.orientation.z = 0.0
			pose_stamped.pose.orientation.w = 1.0
			pose_stamped.header.stamp = rospy.Time.now()

			pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, camera_transform)

			for idx,val in enumerate(self.person_clothes_hists):

				# HISTCMP_INTERSECT = 2
				similarity = cv2.compareHist(hist,val,2)
				#print('sim - ', similarity)

				if similarity > self.clothes_similarity :
					found_target = True

					#print(name)
					print('sim - ', similarity)
					#print('col : ', col, 'row : ', row)
					print('distance - ', selected_depth)
					#print('data_to_send : ', data_to_send)

					send_pose_transformed = pose_transformed

					#print('pose_transformed : ', send_pose_transformed)
					#print('color - ', clothes_color)


					self.personPose_pub.publish(send_pose_transformed)
					break

		self.frame = frame


def main(args):
	rospy.init_node('clothes_pattern', anonymous=True)
	cp = clothes_pattern()

	r = rospy.Rate(5)

	while not rospy.is_shutdown():
		if cp.frame is not None:
			cv2.imshow("Frame", cp.frame)
			cv2.waitKey(1)

	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
