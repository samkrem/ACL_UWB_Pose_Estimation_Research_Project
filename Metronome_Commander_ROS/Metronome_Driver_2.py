#!/usr/bin/env python3
import serial
import rospy
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16


class MetronomeDriver1:
    def __init__(self):
        rospy.init_node("MetronomeDriver1")
        self.az_el_angle_sub = rospy.Subscriber(
            "az_el_angle_1", Float32MultiArray, callback=self.az_el_angle_callback
        )
        self.arduino_port = serial.Serial("/dev/ttyACM1", baudrate=1000000, timeout=1)

    def az_el_angle_callback(self, msg: Float32MultiArray):
        az, el = msg.data
        az_el_angle_bytes = bytes(f"({az},{el})", "utf-8")
        rospy.loginfo(f"Master received ({round(az)}°, {round(el)}°). Sent to Arduino.")
        self.arduino_port.write(az_el_angle_bytes)
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