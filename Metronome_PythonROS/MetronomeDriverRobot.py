#!/usr/bin/env python3
import serial
import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Int16


class MetronomeDriver1:
    def __init__(self):
        rospy.init_node("MetronomeDriver1")
        self.target_angle_sub = rospy.Subscriber(
            "target_angle", Int16, callback=self.target_angle_callback
        )
        self.arduino_port = serial.Serial("/dev/ttyACM0", baudrate=1000000, timeout=1)

    def target_angle_callback(self, msg: Int16):
        target_angle_bytes = bytes(str(msg.data), "utf-8")
        rospy.loginfo("Master received " + str(msg.data) + "°. Sent to Arduino.")
        self.arduino_port.write(target_angle_bytes)
        time.sleep(0.5)
        arduino_response = self.arduino_port.readline()
        rospy.loginfo(arduino_response)


if __name__ == "__main__":
    # rospy.init_node("MetronomeDiver1")
    rospy.loginfo("MetronomeDriver1 is active")
    driver = MetronomeDriver1()

    while not rospy.is_shutdown():
        rospy.spin()
        print("Spin is over")
