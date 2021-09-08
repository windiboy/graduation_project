#!/usr/bin/env python
# coding=utf-8
import rospy
import serial
import math
from std_msgs.msg import Int32MultiArray

class PlatForm:
    def __init__(self):
        rospy.init_node('paltform_control')
        self.point_sub = rospy.Subscriber("/platform_control",Int32MultiArray,self.callback)
        port = rospy.get_param("paltform_port", default="/dev/ttyUSB1")
        self.ser = serial.Serial(port, 115200)
        self.ser.write("!RD\r\n")
        self.rate = rospy.Rate(30)
        rospy.loginfo("[paltform_control] init successful!")
        self.run()
    
    def callback(self, data):
        if len(data.data) < 2:
            return
        msg = "!M {} {} {}\r\n".format(data.data[0], data.data[1], data.data[0])
        rospy.loginfo("[paltform_control] send msg = {}".format(msg))
        self.ser.write(msg)

    def run(self):
        while not rospy.is_shutdown():
            count = self.ser.inWaiting()
            # rospy.loginfo("[paltform_control] msg waiting= {}".format(count))
            if count > 0:
                data = self.ser.read(count) 
                rospy.loginfo("[paltform_control] read msg = {}".format(data))
            self.rate.sleep()
        self.ser.close()
            

if __name__ == "__main__":
    plat_form = PlatForm()
