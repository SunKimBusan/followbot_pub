#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import sys
import os
import cv2
import numpy as np
import pyrealsense2 as rs2
import rospkg
import face_recognition

from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import tf2_ros

import message_filters

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('followbot_mechanum')
person_name = 'taeyang'

class face_check:
	def __init__(self):
		self.bridge = CvBridge()
		self.intrinsics = None
		self.known_face_encodings = []
		self.known_face_names = []
		self.frame = None

		# Load sample pictures and learn how to recognize it.
		dirname = pkg_path + '/src/knowns'
		files = os.listdir(dirname)
		for filename in files:
			name, ext = os.path.splitext(filename)
			if ext == '.jpg':
				self.known_face_names.append(name)
				pathname = os.path.join(dirname, filename)
				img = face_recognition.load_image_file(pathname)
				face_encoding = face_recognition.face_encodings(img)[0]
				self.known_face_encodings.append(face_encoding)

		self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
		self.personPose_pub = rospy.Publisher('person_pose_face', PoseStamped, queue_size=1)

		# camera
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		self.img_processing = False
		depth_sub = message_filters.Subscriber('/pnu/depth_rect_raw', Image)
		img_sub = message_filters.Subscriber('/pnu/rgb_rect_raw_input', Image)
		ts = message_filters.ApproximateTimeSynchronizer([depth_sub, img_sub], 10, 0.2, allow_headerless=False)
		ts.registerCallback(self.depth_rgb_callback)


	def depth_rgb_callback(self, depth_msg, img_msg):

		if self.intrinsics is None:
			return

		if not self.img_processing:

			self.img_processing = True

			try:
				camera_transform = self.tf_buffer.lookup_transform('map', 'camera_link', depth_msg.header.stamp)
			except:
				self.img_processing = False
				return

			cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
			depth_img = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
			self.frame = self.face_check(cv_image, depth_img, camera_transform)
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

	def face_check(self, rgb_img, depth_img, camera_transform):

		frame = rgb_img

		# Find all the faces and face encodings in the current frame of video
		face_locations = face_recognition.face_locations(frame)
		face_encodings = face_recognition.face_encodings(frame, face_locations)

		face_names = []
		for face_encoding in face_encodings:
			# See if the face is a match for the known face(s)
			distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
			min_value = min(distances)
			print("min_value ", min_value)

			# tolerance: How much distance between faces to consider it a match. Lower is more strict.
			# 0.6 is typical best performance.
			name = "Unknown_1"

			if min_value < 0.35:
				print(" face detected")
				index = np.argmin(distances)
				name = self.known_face_names[index]

			face_names.append(name.split('_')[0])

		# Display the results
		for (top, right, bottom, left), name in zip(face_locations, face_names):

			col = int(round((left + right)/2))
			row = int(round((bottom + top)/2))
			face_depth_arr = depth_img[top:bottom,left:right]

			min_depth = 100000
			for i in range(0, len(face_depth_arr)):
				for j in range(0, len(face_depth_arr[0])):
					if face_depth_arr[i][j] != 0:
						if face_depth_arr[i][j] < min_depth:
							min_depth = face_depth_arr[i][j]

			if name == person_name :

				# personPose : [-y, x, z] by camera_link frame unit : mm
				# data_to_send.data = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], min_depth) # [col, row]
				rs2_pose = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], min_depth) # [col, row]
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

				print(min_depth)
				print('pose_transformed : ', pose_transformed)
				self.personPose_pub.publish(pose_transformed)

			# Draw a box around the face
			cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
			# Draw a label with a name below the face
			cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
			font = cv2.FONT_HERSHEY_DUPLEX
			cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

		return frame


def main(args):
	rospy.init_node('face_check', anonymous=True)
	fc = face_check()

	r = rospy.Rate(5)

	while not rospy.is_shutdown():
		if fc.frame is not None:
			cv2.imshow("Frame", fc.frame)
			cv2.waitKey(1)

	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
