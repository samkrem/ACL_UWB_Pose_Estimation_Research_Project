#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int16

import math


class MetronomeScheduler:
    def __init__(self):
        rospy.init_node("MetronomeScheduler")

        self.rate = rospy.Rate(2)
        # self.theta_list = [(2, 2), (3, 2)]
        self.target_angle_pub = rospy.Publisher("target_angle", Int16, queue_size=2)
        self.bot_dist = rospy.Publisher("dist", Float32, queue_size=2)

    def angle_sender(self, ang):
        self.rate.sleep()
        self.target_angle_pub.publish(ang)
        rospy.sleep(2)

    def distance_sender(self, dist):
        self.rate.sleep()
        self.bot_dist.publish(dist)
        rospy.sleep(2)

    def angle_distance_planner(self):
        xz_list = [
            (1, 1),
            (2, 1),
            (1, 2),
            (-1, 1),
            (-2, 1),
            (-3, 1),
        ]  # list of (x,z) coordinates
        for i in range(len(xz_list)):
            theta = (180 / math.pi) * math.atan(xz_list[i][1] / xz_list[i][0])
            rospy.loginfo("Sent " + str(round(theta)) + "° to driver.")
            self.angle_sender(round(theta))

            r = math.hypot(xz_list[i][1], xz_list[i][0])
            self.distance_sender(int(r))
            rospy.sleep(2)

    def round(self, num):
        return int(num) if num - int(num) < 0.5 else int(num) + 1


if __name__ == "__main__":
    # while not rospy.is_shutdown():
    # rospy.loginfo("metronomescheduler is active")
    ms = MetronomeScheduler()
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing")
        ms.angle_distance_planner()
