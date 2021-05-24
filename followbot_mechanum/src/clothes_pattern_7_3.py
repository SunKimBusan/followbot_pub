#!/usr/bin/env python
import rospy

from openpose_ros_msgs.msg import OpenPoseHumanList
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
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

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('followbot_mechanum')
person_name = 'taeyang'

import message_filters

# Use clothes area and face area together.
# Get depth inform if there is face area.

class clothes_pattern:

	def __init__(self):
		self.bridge = CvBridge()
		self.person_clothes_hists = []
		self.person_names = []
		for filename in glob.glob(pkg_path + '/src/person_clothes_img/*.png'):
			im = pil_image.open(filename)
			im_cv = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
			hist = cv2.calcHist([im_cv],[0,1,2],None,[8,8,8],[0,256,0,256,0,256])
			hist = cv2.normalize(hist, hist).flatten()
			self.person_names.append(filename.split('/')[-1])
			self.person_clothes_hists.append(hist)

		self.frame = None

		# camera
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		self.clothes_similarity = 1
		self.face_depth_arr_position = 0.1
		self.clothes_depth_arr_position = 0.1
		self.img_openpose_pub = rospy.Publisher('/camera/color/image_raw_openpose', Image, queue_size=10)

		self.intrinsics = None
		self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)

		self.last_person_pose = None
		self.valid_vel = 3 # m/s
		self.valid_time_from_last_person = 2 # seconds
		self.valid_size_of_person = 30 # image pixels distance
		self.init_unauth_case_num = 5
		self.unauth_case_cnt = self.init_unauth_case_num

		self.img_processing = False
		self.personPose_pub = rospy.Publisher('person_pose', PoseStamped, queue_size=10)
		depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw_throttle', Image)
		img_sub = message_filters.Subscriber('/camera/color/image_raw_throttle', Image)
		ts = message_filters.ApproximateTimeSynchronizer([depth_sub, img_sub], 10, 0.1, allow_headerless=False)
		ts.registerCallback(self.depth_img_callback)


	def depth_img_callback(self, depth_data, img_data):

		if not self.img_processing:
			self.img_processing = True

			time_ = depth_data.header.stamp

			try:
				camera_transform = self.tf_buffer.lookup_transform('odom', 'camera_link', time_)
			except:
				self.img_processing = False
				return

			cv_image = self.bridge.imgmsg_to_cv2(img_data, "bgr8")

			depth_img = self.bridge.imgmsg_to_cv2(depth_data, depth_data.encoding)
			self.img_openpose_pub.publish(img_data)

			try:
				human_list_msg = rospy.wait_for_message('/openpose_ros/human_list', OpenPoseHumanList, 0.5)
			except:
				self.img_processing = False
				return


			self.callback_humanlist(human_list_msg, cv_image, depth_img, camera_transform, time_)

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

		if len(human_list_msg.human_list) == 0:
			return

		depth_img = openpose_depth_img
		frame = cv_image
		found_target = False
		pos_list = []

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
			#print((top, right, bottom, left))

			diagonal_size = sqrt(pow((right-left), 2) + pow((top-bottom), 2))
			print(diagonal_size)

			# neglect unvalid small wrong person object from openpose
			if diagonal_size < self.valid_size_of_person:
				continue

			col = int(round((left + right)/2))
			row = int(round((bottom + top)/2))
			clothes_block = frame[top:bottom, left:right]
			depth_arr = depth_img[top:bottom, left:right]

			cv2.rectangle(frame, (left, bottom), (right, top), (0, 0, 255), 2)
			hist = cv2.calcHist([clothes_block],[0,1,2],None,[8,8,8],[0,256,0,256,0,256])
			hist = cv2.normalize(hist, hist).flatten()


			# face area
			use_face_area = False
			face_points = [1, 17, 18]
			f_x = []
			f_y = []

			for idx in face_points:
				if human.body_key_points_with_prob[idx] != 0:
					if human.body_key_points_with_prob[idx].x != 0 and human.body_key_points_with_prob[idx].y != 0:
						f_x.append(human.body_key_points_with_prob[idx].x)
						f_y.append(human.body_key_points_with_prob[idx].y)

			if len(f_x) == 3:
				use_face_area = True

			pose_transformed = None
			f_pose_transformed = None

			if use_face_area:
				(top, right, bottom, left) = (int(min(f_y)), int(max(f_x)), int(max(f_y)), int(min(f_x)))

				f_col = int(round((left + right)/2))
				f_row = int(round((bottom + top)/2))
				face_block = frame[top:bottom, left:right]
				f_depth_arr = depth_img[top:bottom, left:right]

				# select the depth located in (self.clothes_depth_arr_position)(ex 2/3), because some obstacle can cover person's clothes in front of person.
				depth_list = []
				for i in range(0, len(f_depth_arr)):
					for j in range(0, len(f_depth_arr[0])):
						if f_depth_arr[i][j] != 0:
							depth_list.append(f_depth_arr[i][j])

				if len(depth_list) == 0:
					continue
				else:
					sorted_depth_list = sorted(depth_list, reverse=False)
					selected_idx = int(len(sorted_depth_list)*self.face_depth_arr_position)
					selected_depth = sorted_depth_list[selected_idx]

					cv2.rectangle(frame, (left, bottom), (right, top), (0, 0, 255), 2)

					# personPose : [-y, x, z] by camera_link frame unit : mm
					# data_to_send.data = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], selected_depth) # [col, row]
					rs2_pose = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [f_col, f_row], selected_depth) # [col, row]

					pose_stamped = PoseStamped()

					pose_stamped.header.frame_id = 'camera_link'
					# personPose : [-y, x, z] unit : mm
					pose_stamped.pose.position.x = rs2_pose[1]/1000
					pose_stamped.pose.position.y = -rs2_pose[0]/1000
					pose_stamped.pose.position.z = rs2_pose[2]/1000
					pose_stamped.pose.orientation.z = 0.0
					pose_stamped.pose.orientation.w = 1.0
					pose_stamped.header.stamp = rospy.Time.now()

					f_pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, camera_transform)

					pos_list.append(f_pose_transformed)

			else:

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

				pos_list.append(pose_transformed)

			print('--------------------')

			for idx,val in enumerate(self.person_clothes_hists):

				name = self.person_names[idx]
				if name.split('_')[0] == person_name :
					# HISTCMP_INTERSECT = 2
					similarity = cv2.compareHist(hist,val,2)

					if similarity > self.clothes_similarity :
						found_target = True

						#print(name)
						#print(similarity)
						#print('col : ', col, 'row : ', row)
						#print(selected_depth)
						#print('data_to_send : ', data_to_send)

						send_pose_transformed = None

						if use_face_area:
							send_pose_transformed = f_pose_transformed
						else :
							send_pose_transformed = pose_transformed

						#print('pose_transformed : ', send_pose_transformed)

						self.personPose_pub.publish(send_pose_transformed)
						self.last_person_pose = [send_pose_transformed, time_]
						self.unauth_case_cnt = self.init_unauth_case_num
						break

			if found_target:
				break

		# when we can't find target by clothes histogram, then we try to check all person's position.
		# if specific position is near from last target position, then, we can use this as new target position.

		if (not found_target) and (self.last_person_pose is not None) and (self.unauth_case_cnt != 0):

			last_time = self.last_person_pose[1]
			time_interval = time_ - last_time

			if time_interval.to_sec() < self.valid_time_from_last_person:

				if len(pos_list) != 0:

					last_x = self.last_person_pose[0].pose.position.x
					last_y = self.last_person_pose[0].pose.position.y

					min_vel = None
					min_idx = None

					for idx, pos in enumerate(pos_list):

						pos_x = pos.pose.position.x
						pos_y = pos.pose.position.y

						inc_x = pos_x - last_x
						inc_y = pos_y - last_y

						euclidean_distance = sqrt(pow((inc_x), 2) + pow((inc_y), 2))
						velocity = euclidean_distance/time_interval.to_sec()

						if min_vel is None:
							min_vel = velocity
							min_idx = idx
						elif velocity < min_vel:
							min_vel = velocity
							min_idx = idx

					if min_vel < self.valid_vel:
						self.personPose_pub.publish(pos_list[min_idx])
						self.last_person_pose = [pos_list[min_idx], time_]
						self.unauth_case_cnt -= 1

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
