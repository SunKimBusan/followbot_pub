#!/usr/bin/env python
import rospy

from std_msgs.msg import String, Bool, Int32
import sys
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, degrees, pow, sqrt
import time
import tf2_ros
import tf2_geometry_msgs
from tf import transformations
from sensor_msgs.msg import LaserScan
import numpy as np
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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
        self.moving_duration = 16
        self.goal = PoseStamped()

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.person_pose_valid_vel = 3 # m/s
        self.personPose_sub = rospy.Subscriber('person_pose', PoseStamped, self.personPoseCallback)
        self.marker_pub = rospy.Publisher('person_marker', Marker, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_sent = False
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        #rospy.on_shutdown(self.shutdown)
        #self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Allow up to 5 seconds for the action server to come up
        #self.move_base.wait_for_server(rospy.Duration(5))

        self.use_leg = False
        self.use_leg_count_init = 30
        self.use_leg_count = 0
        self.leg_valid_time_from_last_leg = 1
        self.leg_valid_time_from_last_person = 2
        self.last_legs_inform = []
        self.legs_moving_check_vel = 0.5 # m/s
        self.legs_valid_vel_limit = 2 # m/s
        #Leg part works well when a person is alone.
        #but if there are other people around you, there is a high probability that the robot will chase after other.
        #self.legs_sub = rospy.Subscriber('people_tracked_throttle', PersonArray, self.callback_legs)

        self.job_msg = None
        self.sub_job = rospy.Subscriber("/job", Int32, self.job_cb, queue_size=1)


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

        # We can use leg position n times after getting position from clothes or face.
        if not self.use_leg:
            return

        leg_time = time.time()

        if len(self.integrated_person_pos_hist) == 0:
            return

        last_person_ = self.integrated_person_pos_hist[0]
        leg_time_interval = leg_time - last_person_[2]

        if leg_time_interval > self.leg_valid_time_from_last_person:
            return

        transformed_legs_list = []

        for person in data.people:

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.header.stamp = data.header.stamp
            goal.target_pose.pose = Pose(Point(person.pose.position.x, person.pose.position.y, person.pose.position.z),
                Quaternion(person.pose.orientation.x, person.pose.orientation.y, person.pose.orientation.z, person.pose.orientation.w))

            leg_x = goal.target_pose.pose.position.x
            leg_y = goal.target_pose.pose.position.y

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

        print("person_list --- ", len(person_list))

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

                if velocity > self.legs_moving_check_vel:
                    moving_person_list.append(present_person)

        nearest_person = None
        min_vel = None
        last_integrated_person = self.integrated_person_pos_hist[0]
        time_interval = legs_inform[1] - last_integrated_person[2]

        print("person_list --- ", len(moving_person_list))

        # Is nearest legs position valid ?
        if len(moving_person_list) == 0:
            return
        else:

            for person in moving_person_list:

                inc_x = person[0] - last_integrated_person[0]
                inc_y = person[1] - last_integrated_person[1]

                euclidean_distance = sqrt(pow((inc_x), 2) + pow((inc_y), 2))
                velocity = euclidean_distance/time_interval

                if velocity < self.legs_valid_vel_limit:
                    if nearest_person is None:
                        nearest_person = person
                        min_vel = velocity
                    else :
                        if velocity < min_vel:
                            nearest_person = person
                            min_vel = velocity

        if nearest_person is not None:

            print("------ legs ------")

            input_list = [nearest_person[0], nearest_person[1], legs_inform[1]] # x, y, time

            self.integrated_person_pos_hist.insert(0, input_list)
            if len(self.integrated_person_pos_hist) > self.hist_size:
                self.integrated_person_pos_hist.pop()

            # We can use leg position n times after getting position from clothes or face.
            self.use_leg_count -= 1
            if self.use_leg_count == 0:
                self.use_leg = False

            ## send marker
            marker = Marker()
            marker.header.frame_id = "odom"
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

            self.moving_mode = True

        self.last_legs_inform = legs_inform


    def personPoseCallback(self,data):

        time_ = time.time()
        self.start_time = time_

        # Is velocity valid ?
        if len(self.integrated_person_pos_hist) != 0:

            time_interval = time_ - self.integrated_person_pos_hist[0][2]

            if time_interval < 2:

                person_x = self.integrated_person_pos_hist[0][0]
                person_y = self.integrated_person_pos_hist[0][1]

                inc_x = data.pose.position.x - person_x
                inc_y = data.pose.position.y - person_y

                euclidean_distance = sqrt(pow((inc_x), 2) + pow((inc_y), 2))

                velocity = euclidean_distance/time_interval

                if velocity > self.person_pose_valid_vel:
                    return


        # input to list
        input_list = [data.pose.position.x, data.pose.position.y, time_]
        self.integrated_person_pos_hist.insert(0, input_list)
        if len(self.integrated_person_pos_hist) > self.hist_size:
            self.integrated_person_pos_hist.pop()

        # We can use leg position n times after getting position from clothes or face.
        self.use_leg = True
        self.use_leg_count = self.use_leg_count_init

        ## send marker
        marker = Marker()
        marker.header.frame_id = "odom"
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

        self.moving_mode = True


    def callback_odom(self,data):
        self.odom = data

    def move2goal(self):

        self.goal_sent = True
        print('---- moving to target ----')

        odom_msg = self.odom

        person_x = self.integrated_person_pos_hist[0][0]
        person_y = self.integrated_person_pos_hist[0][1]

        self.pub_goal_destination(person_x, person_y, 0, 0, 1)
        time.sleep(0.1)

        while True:
            if self.job_msg.data == 0:
                break
            time.sleep(0.1)

    def rotation(self):

        print('---- rotating for detection ----')
        start_time = time.time()

        r = rospy.Rate(3)
        while not rospy.is_shutdown():

            time_interval =  time.time() - start_time
            if int(time_interval) % 3 == 0:
                time.sleep(1)

            if self.moving_mode:
                break

            speed = Twist()
            speed.linear.x = 0.0
            speed.angular.z = 0.4
            self.cmd_vel_pub.publish(speed)

            r.sleep()

def main():
    rospy.init_node('controller', anonymous=False)
    controller  = Controller()

    r = rospy.Rate(3)

    while not rospy.is_shutdown():
        print('here')

        if controller.start_time is not None:
            time_disappeard =  time.time() - controller.start_time
            if time_disappeard > controller.waiting_time_for_rotation:
                print('---- Target disappeared ----')
                controller.moving_mode = False

        if controller.moving_mode:
            controller.move2goal()
        else:
            controller.rotation()

        r.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
