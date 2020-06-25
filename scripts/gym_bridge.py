#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped

from f1tenth_gym_ros.msg import RaceInfo

from tf2_ros import transform_broadcaster
from tf.transformations import quaternion_from_euler

import numpy as np

import gym


class GymBridge(object):
    def __init__(self):
        # Get params
        self.ego_scan_topic = rospy.get_param('ego_scan_topic')
        self.ego_odom_topic = rospy.get_param('ego_odom_topic')
        self.ego_drive_topic = rospy.get_param('ego_drive_topic')
        self.race_info_topic = rospy.get_param('race_info_topic')

        self.scan_distance_to_base_link = rospy.get_param('scan_distance_to_base_link')

        scan_fov = rospy.get_param('scan_fov')
        scan_beams = rospy.get_param('scan_beams')
        self.angle_min = -scan_fov / 2.
        self.angle_max =  scan_fov / 2.
        self.angle_inc =  scan_fov / scan_beams

        # Init gym backend
        self.racecar_env = gym.make('f110_gym:f110-v0')

        # Init the map
        map_path = rospy.get_param('map_path')
        map_img_ext = rospy.get_param('map_img_ext')
        exec_dir = rospy.get_param('executable_dir')
        self.racecar_env.init_map(map_path, map_img_ext, False, False)

        # Init the racecar
        wheelbase = rospy.get_param('wheelbase')
        mass = rospy.get_param('mass')
        l_r = rospy.get_param('l_r')
        I_z = rospy.get_param('I_z')
        mu = rospy.get_param('mu')
        h_cg = rospy.get_param('h_cg')
        cs_f = rospy.get_param('cs_f')
        cs_r = rospy.get_param('cs_r')
        self.racecar_env.update_params(mu, h_cg, l_r, cs_f, cs_r, I_z, mass, exec_dir, double_finish=True)

        # Initial state of the racecar
        initial_state = {
            'x': [0.0, 200.0],
            'y': [0.0, 200.0],
            'theta': [0.0, 0.0],
        }
        self.obs, _, self.done, _ = self.racecar_env.reset(initial_state)
        self.ego_pose = [0., 0., 0.]
        self.ego_speed = [0., 0., 0.]
        self.ego_steer = 0.0

        # Keep track of latest sim state
        self.ego_scan = self.obs['scans'][0]

        # Transform broadcaster
        self.br = transform_broadcaster.TransformBroadcaster()

        # Publishers
        qsize = 1
        self.ego_scan_pub = rospy.Publisher(self.ego_scan_topic, LaserScan, queue_size=qsize)
        self.ego_odom_pub = rospy.Publisher(self.ego_odom_topic, Odometry, queue_size=qsize)
        self.info_pub = rospy.Publisher(self.race_info_topic, RaceInfo, queue_size=qsize)

        # Subscribers
        self.drive_sub = rospy.Subscriber(
            self.ego_drive_topic,
            AckermannDriveStamped,
            self.drive_callback,
            queue_size=qsize,
        )

        # Timer
        timestep = rospy.Duration(0.004)
        self.timer = rospy.Timer(timestep, self.timer_callback)

    def update_sim_state(self):
        self.ego_scan = self.obs['scans'][0]
        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

    def drive_callback(self, drive_msg):
        ego_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        opp_speed = 0.
        opp_steer = 0.
        action = {
            'ego_idx': 0,
            'speed': [ego_speed, opp_speed],
            'steer': [self.ego_steer, opp_steer],
        }
        self.obs, step_reward, self.done, info = self.racecar_env.step(action)
        self.update_sim_state()

    def timer_callback(self, timer):
        ts = rospy.Time.now()

        # pub scan
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = 'ego_racecar/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        # pub tf
        self.publish_odom(ts)
        self.publish_transforms(ts)
        self.publish_laser_transforms(ts)
        self.publish_wheel_transforms(ts)

        # pub race info
        self.publish_race_info(ts)

    def publish_race_info(self, ts):
        info = RaceInfo()
        info.header.stamp = ts
        info.ego_collision = self.obs['collisions'][0]
        info.ego_elapsed_time = self.obs['lap_times'][0]
        info.ego_lap_count = self.obs['lap_counts'][0]
        self.info_pub.publish(info)

    def publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = '/map'
        ego_odom.child_frame_id = 'ego_racecar/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = quaternion_from_euler(0., 0., self.ego_pose[2])
        ego_odom.pose.pose.orientation.x = ego_quat[0]
        ego_odom.pose.pose.orientation.y = ego_quat[1]
        ego_odom.pose.pose.orientation.z = ego_quat[2]
        ego_odom.pose.pose.orientation.w = ego_quat[3]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

    def publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = quaternion_from_euler(0.0, 0.0, self.ego_pose[2])
        ego_t.rotation.x = ego_quat[0]
        ego_t.rotation.y = ego_quat[1]
        ego_t.rotation.z = ego_quat[2]
        ego_t.rotation.w = ego_quat[3]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = '/map'
        ego_ts.child_frame_id = 'ego_racecar/base_link'

        self.br.sendTransform(ego_ts)

    def publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = quaternion_from_euler(0., 0., self.ego_steer)
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[0]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[3]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = 'ego_racecar/front_left_hinge'
        ego_wheel_ts.child_frame_id = 'ego_racecar/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = 'ego_racecar/front_right_hinge'
        ego_wheel_ts.child_frame_id = 'ego_racecar/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

    def publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = 'ego_racecar/base_link'
        ego_scan_ts.child_frame_id = 'ego_racecar/laser'
        self.br.sendTransform(ego_scan_ts)


if __name__ == '__main__':
    rospy.init_node('gym_bridge')
    gym_bridge = GymBridge()
    rospy.spin()
