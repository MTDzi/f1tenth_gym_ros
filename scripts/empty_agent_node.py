#!/usr/bin/env python
import sys
import rospy


class Agent:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('empty_agent')
    dummy_agent = Agent()
    rospy.spin()
