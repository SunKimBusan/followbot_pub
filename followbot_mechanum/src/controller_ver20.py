#!/usr/bin/env python
import rospy

from std_msgs.msg import String, Bool, Int32
import sys
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from math import atan2, degrees, pow, sqrt
import time
import tf2_geometry_msgs
import tf2_ros

from tf import transformations
from sensor_msgs.msg import LaserScan
import numpy as np
import actionlib
from actionlib_msgs.msg import *
from visualization_msgs.msg import Marker
from leg_tracker.msg import PersonArray

from openpose_ros_msgs.msg import OpenPoseHumanList

# Consider face, clothes, legs

class Controller():
    def __init__(self):

        self.moving_mode = False
        self.odom = None
        self.personPose = None
        self.start_time = None
        self.integrated_person_pos_hist = []
        self.hist_size = 64

        self.waiting_time_for_rotation = 8

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.personPose_face_sub = rospy.Subscriber('person_pose_face', PoseStamped, self.face_Callback)
        self.personPose_clothes_sub = rospy.Subscriber('person_pose_clothes', PoseStamped, self.clothes_Callback)
        self.auth_time = None
        self.auth_state = False
        self.auth_time_limit = 20

        self.marker_pub = rospy.Publisher('person_marker', Marker, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        self.leg_valid_time_from_last_leg = 1
        self.last_legs_inform = []
        self.legs_moving_check_vel = 0.25 # m/s
        self.legs_valid_vel_limit = 2 # m/s
        self.max_distance_to_leg = 3.5 # m
        #Leg part works well when a person is alone.
        #but if there are other people around you, there is a high probability that the robot will chase after other.
        self.legs_sub = rospy.Subscriber('people_tracked_throttle', PersonArray, self.callback_legs)

        self.job_msg = None
        self.sub_job = rospy.Subscriber("/job", Int32, self.job_cb, queue_size=1)
        self.new_goal = False

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)



    def job_cb(self, msg):
        self.job_msg = msg

    def pub_goal_destination(self, x, y, z, ori_z, ori_w):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.z = ori_z
        goal.pose.orientation.w = ori_w
        self.goal_pub.publish(goal)

    def callback_legs(self,data):

        leg_time = time.time()

        if self.auth_state:
            diff_time = time.time() - self.auth_time
            if diff_time > self.auth_time_limit:
                self.auth_state = False
                self.auth_time = None

        if not (not self.new_goal and self.auth_state):
            return

        transformed_legs_list = []

        for person in data.people:

            leg_x = person.pose.position.x
            leg_y = person.pose.position.y

            leg_inform = [leg_x, leg_y, person.id]
            transformed_legs_list.append(leg_inform)

        legs_inform = [transformed_legs_list, leg_time]

        if len(self.last_legs_inform) != 0:
            last_legs_inform = self.last_legs_inform
            last_leg_time = last_legs_inform[1]

            if (leg_time - last_leg_time) < self.leg_valid_time_from_last_leg:
                self.find_target_legs(last_legs_inform, legs_inform)
            else:
                self.last_legs_inform = legs_inform
        else:
            self.last_legs_inform = legs_inform


    def find_target_legs(self, last_legs_inform, legs_inform):

        person_list = []

        # check present id also exist in last_legs
        for present_person in legs_inform[0] :
            for last_person in last_legs_inform[0] :
                if present_person[2] == last_person[2]: # id
                    person_list.append([present_person, last_person])
                    break

        moving_person_list = []

        #print("person_list --- ", len(person_list))

        # Is present legs position moving ?
        if len(person_list) == 0:
            return
        else:
            time_interval = legs_inform[1] - last_legs_inform[1]
            for person_set in person_list:

                present_person = person_set[0]
                last_person = person_set[1]

                inc_x = present_person[0] - last_person[0]
                inc_y = present_person[1] - last_person[1]

                euclidean_distance = sqrt(pow((inc_x), 2) + pow((inc_y), 2))
                velocity = euclidean_distance/time_interval

                if velocity > self.legs_moving_check_vel and velocity < self.legs_valid_vel_limit:
                    moving_person_list.append(present_person)


        nearest_person = None
        min_dist = None

        #print("person_list --- ", len(moving_person_list))

        # Is nearest legs position valid ?
        if len(moving_person_list) == 0:
            return
        else:
            odom_msg = self.odom

            for person in moving_person_list:

                transform = self.tf_buffer.lookup_transform('map', 'odom', odom_msg.header.stamp)
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'odom'
                pose_stamped.pose.position.x = odom_msg.pose.pose.position.x
                pose_stamped.pose.position.y = odom_msg.pose.pose.position.y
                pose_stamped.pose.position.z = odom_msg.pose.pose.position.z
                pose_stamped.pose.orientation.z = odom_msg.pose.pose.orientation.z
                pose_stamped.pose.orientation.w = odom_msg.pose.pose.orientation.w
                pose_stamped.header.stamp = odom_msg.header.stamp
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

                x = pose_transformed.pose.position.x
                y = pose_transformed.pose.position.y
                rot_q = pose_transformed.pose.orientation
                (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

                inc_x = person[0] - x
                inc_y = person[1] - y

                theta = 180/math.pi*theta
                theta_to_target = 180/math.pi*atan2(inc_y, inc_x)

                dtheta = round((theta-theta_to_target),2)
                dtheta = min(dtheta%360, dtheta%360 + 360, key=abs)

                if abs(dtheta) < 50:
                    continue

                euclidean_distance = sqrt(pow((inc_x), 2) + pow((inc_y), 2))

                if euclidean_distance > self.max_distance_to_leg:
                    continue

                if nearest_person is None:
                    nearest_person = person
                    min_dist = euclidean_distance
                else:
                    if euclidean_distance < min_dist:
                        nearest_person = person
                        min_dist = euclidean_distance

        if nearest_person is not None:

            print("------ legs ------")

            odom_msg = self.odom
            transform = self.tf_buffer.lookup_transform('map', 'odom', odom_msg.header.stamp)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.pose.position.x = odom_msg.pose.pose.position.x
            pose_stamped.pose.position.y = odom_msg.pose.pose.position.y
            pose_stamped.pose.position.z = odom_msg.pose.pose.position.z
            pose_stamped.pose.orientation.z = odom_msg.pose.pose.orientation.z
            pose_stamped.pose.orientation.w = odom_msg.pose.pose.orientation.w
            pose_stamped.header.stamp = odom_msg.header.stamp
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

            x = pose_transformed.pose.position.x
            y = pose_transformed.pose.position.y
            rot_q = pose_transformed.pose.orientation
            (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

            inc_x = nearest_person[0] - x
            inc_y = nearest_person[1] - y

            theta_to_target = atan2(inc_y, inc_x)
            quat = quaternion_from_euler(0,0,theta_to_target)
            input_list = [nearest_person[0], nearest_person[1], quat[2], quat[3], legs_inform[1]] # x, y, time

            self.integrated_person_pos_hist.insert(0, input_list)
            if len(self.integrated_person_pos_hist) > self.hist_size:
                self.integrated_person_pos_hist.pop()

            ## send marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = nearest_person[0]
            marker.pose.position.y = nearest_person[1]
            marker.pose.position.z = 0.0
            self.marker_pub.publish(marker)

            self.start_time = time.time()
            self.moving_mode = True
            self.new_goal = True

        self.last_legs_inform = legs_inform


    def face_Callback(self,data):
        self.auth_time = time.time()
        self.auth_state = True
        self.personpose_ft(data)
        print("------ face ------")

    def clothes_Callback(self,data):
        if self.auth_state:
            diff_time = time.time() - self.auth_time
            if diff_time < self.auth_time_limit:
                self.auth_time = time.time()
                self.personpose_ft(data)
                print("------ clothes ------")
            else:
                self.auth_state = False
                self.auth_time = None

    def personpose_ft(self,data):

        time_ = time.time()
        odom_msg = self.odom
        transform = self.tf_buffer.lookup_transform('map', 'odom', odom_msg.header.stamp)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = odom_msg.pose.pose.position.x
        pose_stamped.pose.position.y = odom_msg.pose.pose.position.y
        pose_stamped.pose.position.z = odom_msg.pose.pose.position.z
        pose_stamped.pose.orientation.z = odom_msg.pose.pose.orientation.z
        pose_stamped.pose.orientation.w = odom_msg.pose.pose.orientation.w
        pose_stamped.header.stamp = odom_msg.header.stamp
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

        x = pose_transformed.pose.position.x
        y = pose_transformed.pose.position.y
        rot_q = pose_transformed.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        inc_x = data.pose.position.x - x
        inc_y = data.pose.position.y - y

        theta_to_target = atan2(inc_y, inc_x)
        quat = quaternion_from_euler(0,0,theta_to_target)
        input_list = [data.pose.position.x, data.pose.position.y, quat[2], quat[3], time_] # x, y, time

        self.integrated_person_pos_hist.insert(0, input_list)
        if len(self.integrated_person_pos_hist) > self.hist_size:
            self.integrated_person_pos_hist.pop()

        ## send marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = data.pose.position.x
        marker.pose.position.y = data.pose.position.y
        marker.pose.position.z = 0.0
        self.marker_pub.publish(marker)

        self.start_time = time_
        self.moving_mode = True
        self.new_goal = True


    def callback_odom(self,data):
        self.odom = data

    def move2goal(self):
        self.new_goal = False

        print('---- moving to target ----')
        person_x = self.integrated_person_pos_hist[0][0]
        person_y = self.integrated_person_pos_hist[0][1]
        person_ori_z = self.integrated_person_pos_hist[0][2]
        person_ori_w = self.integrated_person_pos_hist[0][3]

        odom_msg = self.odom
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        inc_x = person_x - x
        inc_y = person_y - y
        euclidean_distance = sqrt(pow((inc_x), 2) + pow((inc_y), 2))

        self.pub_goal_destination(person_x, person_y, 0, person_ori_z, person_ori_w)

        while True:
            if self.job_msg.data == 0:
                print('---- arrive ----')
                break
            time.sleep(0.1)

    def rotation(self):

        print('---- rotating for detection ----')
        start_time = time.time()

        while True:

            time_interval =  time.time() - start_time
            if int(time_interval) % 3 == 0:
                time.sleep(1)

            if self.moving_mode:
                break

            speed = Twist()
            speed.linear.x = 0.0
            speed.angular.z = 0.4
            self.cmd_vel_pub.publish(speed)

            time.sleep(0.3)

def main():
    rospy.init_node('controller', anonymous=False)
    controller  = Controller()

    r = rospy.Rate(3)

    while not rospy.is_shutdown():

        if controller.start_time is not None:
            time_disappeard =  time.time() - controller.start_time
            if time_disappeard > controller.waiting_time_for_rotation:
                print('---- Target disappeared ----')
                controller.moving_mode = False

        if controller.moving_mode and controller.new_goal:
            controller.move2goal()
        elif not controller.moving_mode:
            controller.rotation()

        r.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
