#!/usr/bin/env python3
import serial
import rospy
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16


class metronome_driver_1:
    def __init__(self):
        rospy.init_node("metronome_driver_1")
        self.az_el_angle_sub = rospy.Subscriber(
            "az_el_angle_1", Float32MultiArray, callback=self.az_el_angle_callback
        )
        self.arduino_port = serial.Serial("/dev/ttyACM0", baudrate=1000000, timeout=1)
        # change object name
        self.mocap_angle = rospy.Subscriber('world/mocap_angle', Float32MultiArray, callback=self.mocap_callback)
        
        self.prev_error_az = 0
        self.prev_error_el = 0
        
    def mocap_callback(self, msg: Float32MultiArray):
        #when do callback, convert quaternion to euler angle
        self.mocap_az, self.mocap_el = msg.data

    def az_el_angle_callback(self, msg: Float32MultiArray):
        self.az, self.el = msg.data
        az_el_angle_bytes = bytes(f"({self.az},{self.el})", "utf-8")
        rospy.loginfo(f"Master received ({round(self.az)}°, {round(self.el)}°). Sent to Arduino.")
        self.arduino_port.write(az_el_angle_bytes)
        #change sleep time as needed to give actuators enough time to reach set point
        # time.sleep(0.5)
        arduino_response = self.arduino_port.readline()
        rospy.loginfo(arduino_response) 
        #change sleep time as needed to give actuators enough time to reach set point
        time.sleep(0.5)
        #change the error bound as needed
        self.err_bnd = 5
        # check the error + PID
        mod_az, mod_el, err_az, err_el = self.controller()
        if abs(err_az) < self.err_bnd and abs(err_el) < self.err_bnd:
            return
        if abs(err_az) > self.err_bnd: #change error magnitude as needed
            if abs(err_el) > self.err_bnd:
                az_el_angle_bytes = bytes(f"({mod_az},{mod_el})", "utf-8")
            else:
                az_el_angle_bytes = bytes(f"({mod_az},{self.el})", "utf-8")
        else:
            if abs(err_el) > self.err_bnd:
                az_el_angle_bytes = bytes(f"({self.az},{mod_el})", "utf-8")
            
        rospy.loginfo(f"Errors {err_az}, {err_el}). Sent corrections to Arduino.")
        self.arduino_port.write(az_el_angle_bytes)
        arduino_response = self.arduino_port.readline()
        rospy.loginfo(arduino_response) 
        #change sleep time as needed to give actuators enough time to reach set point
        time.sleep(0.5)
            
            
    def controller(self):
        """
        Given scheduler (estimate) and mocap (real) azimuth elevation angles, controller corrects the angle
        and returns error associated with it
        """
        kp = 1 # to be changed s.t. two kp value, one for az one for el
        kd = 0.5
        const = .1        

        error_az = self.mocap_az - self.az
        error_el = self.mocap_el - self.el

        dedt_az = error_az - self.prev_error_az
        dedt_el = error_el - self.prev_error_el

        u_az = kp * error_az + kd * (dedt_az) #kp * e(t) + kd * de/dt
        u_el = kp * error_el + kd * (dedt_el)

        self.prev_error_az = error_az
        self.prev_error_el = error_el
        
        return (self.mocap_az + const * u_az , self.mocap_el + const * u_el, error_az, error_el)
        

if __name__ == "__main__":
    rospy.loginfo("metronome_driver_1 is active")
    driver = metronome_driver_1()

    while not rospy.is_shutdown():
        rospy.spin()
        print("Spin is over")