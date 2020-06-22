#!/usr/bin/env python
import sys
import os

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from agents import PurePursuitAgent



if __name__ == '__main__':
    rospy.init_node('pure_pursuit_agent')

    path_to_waypoint = os.path.join(
        os.path.dirname(__file__),
        '..',
        'waypoints',
        'waypoints6000.csv',
    )

    pure_pursuit_agent = PurePursuitAgent(
        csv_path=path_to_waypoint,
        lookahead_distance=2.0,
        wheelbase=0.18,
    )

    rospy.spin()
