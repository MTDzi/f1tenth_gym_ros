#!/usr/bin/env python
import sys
import rospy
import pygame
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Agent:
    def __init__(self):
        self.drive_pub = rospy.Publisher(
            '/drive',
            AckermannDriveStamped,
            queue_size=1,
        )
        self.scan_sub = rospy.Subscriber(
            '/scan',
            LaserScan,
            self.scan_callback,
            queue_size=1,
        )
        self.odom_sub = rospy.Subscriber(
            '/odom',
            Odometry,
            self.odom_callback,
            queue_size=1,
        )

        pygame.init()
        pygame.joystick.init()
        try:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        except:
            print('No joystick found')
            sys.exit(1)

        self.step = 0
        self.odom = None
        print('x,y')

    def odom_callback(self, odom_msg):
        self.odom = odom_msg

    def scan_callback(self, scan_msg):
        for event in pygame.event.get():
            pass

        self.joystick.init()

        drive = AckermannDriveStamped()
        drive.drive.speed = -4*self.joystick.get_axis(3)
        drive.drive.steering_angle = -self.joystick.get_axis(0)
        self.drive_pub.publish(drive)
        '''
        self.step = (self.step + 1) % 10
        if self.step == 0 and self.odom is not None:
            print(
                '{},{}'
                .format(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
            )
        '''


if __name__ == '__main__':
    rospy.init_node('dummy_agent')
    dummy_agent = Agent()
    rospy.spin()
