#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Bool
import pandas as pd
import math
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from rover_trajectory_msgs.msg import RoverState


class metronome_scheduler_cmd:
    def __init__(self):
        rospy.init_node("metronome_scheduler_cmd")

        self.rate = rospy.Rate(3)
        self.met1_rotating = False
        self.met2_rotating = False

        #get waypoints
        self.waypoints_file = rospy.get_param("~metronome_command/waypoints_file")
        df = pd.read_csv(self.waypoints_file, header = 0)
        waypoints = np.array([df.iloc[:,1:4]])
        self.wp_idx = 0
        self.num_wps = waypoints.shape[0]

        # subscribe to mocap and velocity commands
        self.sub_pose = [rospy.Subscriber(f"{rover}/world", PoseStamped, self.pose_cb, queue_size=1, callback_args=rover) for rover in self.rovers]
        self.sub_auto_cmd = rospy.Subscriber("cmd_vel_auto", Twist, self.vel_cb, queue_size=1)


        self.az_el_pub_1 = rospy.Publisher(
            "az_el_angle_1", Float32MultiArray, queue_size=2
        )
        self.az_el_pub_2 = rospy.Publisher(
            "az_el_angle_2", Float32MultiArray, queue_size=2
        )
        self.met1_rot_status = rospy.Publisher(
            "met1_rotating", Bool, queue_size = 2
        )
        self.met2_rot_status = rospy.Publisher(
            "met2_rotating", Bool, queue_size = 2
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

  
    def angle_distance_planner(self, r1_poses, r2_poses):
        x1, y1, yaw1, az1, el1 = r1_poses
        x2, y2, yaw2, az2, el2 = r2_poses

        met1_rotating = Bool()
        met2_rotating = Bool()
        self.met1_rotating, self.met2_rotating = True, True
        met1_rotating.data = self.met1_rotating
        met2_rotating.data = self.met2_rotating
        self.met1_rot_status.publish(met1_rotating)
        self.met2_rot_status.publish(met2_rotating)

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
            rospy.sleep(3)
        
        self.met1_rotating, self.met2_rotating = False, False
        met1_rotating.data = self.met1_rotating
        met2_rotating.data = self.met2_rotating
        self.met1_rot_status.publish(met1_rotating)
        self.met2_rot_status.publish(met2_rotating)


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
    rospy.loginfo("metronome_scheduler_cmd is active")
    file_path = "~/metronome_ws/src/metronome_controller/waypoints_ex.csv"
    r1_waypoints, r2_waypoints = read_csv_file(file_path)

    ms = metronome_scheduler_cmd.py()
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing")
        # outer loop: while we have not gone through all the waypoints
        # if (current_pose - current_waypoint_pose is 0 or within error bound): call angle_distance_planner
        # once that's done, move on to next waypoint

        ms.angle_distance_planner(r1_waypoints, r2_waypoints)
