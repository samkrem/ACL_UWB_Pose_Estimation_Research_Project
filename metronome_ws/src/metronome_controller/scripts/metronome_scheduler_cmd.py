#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32MultiArray
import pandas as pd
import math


class metronome_scheduler_cmd:
    def __init__(self):
        rospy.init_node("metronome_scheduler_cmd")

        self.rate = rospy.Rate(3)

        self.az_el_pub_1 = rospy.Publisher(
            "az_el_angle_1", Float32MultiArray, queue_size=2
        )
        self.az_el_pub_2 = rospy.Publisher(
            "az_el_angle_2", Float32MultiArray, queue_size=2
        )

        self.x_y_yaw_pub_1 = rospy.Publisher(
            "x_y_yaw_coord1", Float32MultiArray, queue_size=2
        )
        self.x_y_yaw_pub_2 = rospy.Publisher(
            "x_y_yaw_coord2", Float32MultiArray, queue_size=2
        )

    def actuator_sender(self, az_el_r1, az_el_r2):
        """
        sends data to AX12A servo
        """
        self.rate.sleep()
        az_el_ang_1 = Float32MultiArray()
        az_el_ang_1.data = az_el_r1

        az_el_ang_2 = Float32MultiArray()
        az_el_ang_2.data = az_el_r2

        self.az_el_pub_1.publish(az_el_ang_1)
        self.az_el_pub_2.publish(az_el_ang_2)

        rospy.sleep(3)

    def turtlebot_sender(self, x_y_yaw_r1, x_y_yaw_r2):
        """
        Sends data to turtlebot
        """
        self.rate.sleep()
        x_y_yaw_comb_1 = Float32MultiArray()
        x_y_yaw_comb_1.data = x_y_yaw_r1

        x_y_yaw_comb_2 = Float32MultiArray()
        x_y_yaw_comb_2.data = x_y_yaw_r2

        self.x_y_yaw_pub_1.publish(x_y_yaw_comb_1)
        self.x_y_yaw_pub_2.publish(x_y_yaw_comb_2)

        rospy.sleep(3)

    def angle_distance_planner(self, r1_poses, r2_poses):
        x1, y1, yaw1, az1, el1 = r1_poses
        x2, y2, yaw2, az2, el2 = r2_poses

        for i in range(len(x1)):
            rospy.loginfo(
                f"Sent (az1, el1): ({round(az1[i])}°, {round(el1[i])}°) to metronome driver 1. "
            )
            rospy.loginfo(
                f"Sent (az2, el2): ({round(az2[i])}°, {round(el2[i])}°) to metronome driver 2. "
            )

            rospy.loginfo(
                f"Sent (x1, y1, yaw1): ({round(x1[i])}, {round(y1[i])}, {round(yaw1[i])}) to turtlebot. "
            )
            rospy.loginfo(
                f"Sent (x2, y2, yaw2): ({round(x2[i])}, {round(y2[i])}, {round(yaw2[i])}) to turtlebot. "
            )

            self.actuator_sender((az1[i], el1[i]), (az2[i], el2[i]))
            self.turtlebot_sender((x1[i], y1[i], yaw1[i]), (x2[i], y2[i], yaw2[i]))
            rospy.sleep(3)

    def round(self, num):
        return int(num) if num - int(num) < 0.5 else int(num) + 1


def read_csv_file(file_path):
    # Read CSV file into a Pandas DataFrame
    df = pd.read_csv(file_path)

    # Extract columns as NumPy arrays
    x1 = df["x1"].values
    y1 = df["y1"].values
    yaw1 = df["yaw1"].values
    az1 = df["az1"].values
    el1 = df["el1"].values

    x2 = df["x2"].values
    y2 = df["y2"].values
    yaw2 = df["yaw2"].values
    az2 = df["az2"].values
    el2 = df["el2"].values

    return (x1, y1, yaw1, az1, el1), (x2, y2, yaw2, az2, el2)


if __name__ == "__main__":
    rospy.loginfo("metronomescheduler is active")
    file_path = "~/workspace/uwb-workspace/metronome_ws/src/metronome_controller/waypoints_ex.csv"
    r1_waypoints, r2_waypoints = read_csv_file(file_path)

    ms = metronome_scheduler_cmd.py()
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing")
        ms.angle_distance_planner(r1_waypoints, r2_waypoints)