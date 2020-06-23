import numpy as np
import csv

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from agent_utils import (
    get_actuation,
    nearest_point_on_trajectory_py2,
    first_point_on_trajectory_intersecting_circle,
)


class Agent(object):
    def __init__(self, csv_path):
        # TODO: load waypoints from csv
        self.waypoints = None
        self.safe_speed = 0.5

    def plan(self, obs):
        pass


class PurePursuitAgent(Agent):
    def __init__(self, csv_path, lookahead_distance, safe_speed=3.5):
        super(PurePursuitAgent, self).__init__(csv_path)
        self.lookahead_distance = lookahead_distance
        self.safe_speed = safe_speed
        with open(csv_path, 'rb') as f:
            wpts = [tuple(line) for line in csv.reader(f)]
            self.waypoints = np.array([(float(pt[0]), float(pt[1]), float(pt[2]), float(pt[3]), float(pt[4]), float(pt[5])) for pt in wpts])

	self.drive_pub = rospy.Publisher(
            '/drive',
            AckermannDriveStamped,
            queue_size=10,
        )
        self.odom_sub = rospy.Subscriber(
            '/odom',
            Odometry,
            self.odom_callback,
            queue_size=10,
        )

    def odom_callback(self, odom_msg):
        pose_x = odom_msg.pose.pose.position.x
        pose_y = odom_msg.pose.pose.position.y

        orient = odom_msg.pose.pose.orientation
        orient_euler = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        yaw = orient_euler[-1]  # The last, third Euler angle is yaw
        pose_x = odom_msg.pose.pose.position.x
        pose_y = odom_msg.pose.pose.position.y
        position = np.array([pose_x, pose_y])

        orient = odom_msg.pose.pose.orientation
        orient_euler = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        yaw = orient_euler[-1]  # The last, third Euler angle is yaw

        point_ahead = position + self.lookahead_distance * np.array([np.cos(yaw), np.sin(yaw)])
        dists = np.linalg.norm(self.waypoints[:, :2] - point_ahead, axis=1)
        which_closest = np.argmin(dists)
        closest_point = self.waypoints[which_closest, :2]

        delta = closest_point - position
        target_yaw = np.arctan2(delta[1], delta[0])
        yaw_correction = target_yaw - yaw
        if yaw_correction > np.pi:
            yaw_correction -= 2*np.pi
        elif yaw_correction < -np.pi:
            yaw_correction += 2*np.pi

        drive = AckermannDriveStamped()
        drive.drive.speed = self.safe_speed
        drive.drive.steering_angle = yaw_correction
        self.drive_pub.publish(drive)
