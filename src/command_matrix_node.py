#!/usr/bin/env python
import time, math, rospy
from vradio.msg import Radio
from multi_wii.msg import IMU
from copter_control.msg import CommandMatrix
import numpy as np

maxLog = 20

class Translator:
    def __init__(self):
        rospy.init_node("command_matrix_node", anonymous=False)
        radio = rospy.get_param("~radio")
        imu = rospy.get_param("~imu")
        self.subRadio = rospy.Subscriber(radio, Radio, self.radioC queue_size=10)
        self.subIMU = rospy.Subscriber(imu, IMU, self.imuC, queue_size=10)
        self.pub = rospy.Publisher("command_matrix", CommandMatrix, queue_size=10)

        self.radioLog = []
        self.imuLog = []
        self.newIMU = True
        self.newRadio = True

        rate = rospy.Rate(10)
        rospy.spin()

    def radioC(self, data):
        self.radioLog.insert(0, data)
        self.prune()
        self.newRadio = True
        if self.newIMU:
            self.pubMatrix()
        
    def imuC(self, data):
        self.imuLog.insert(0, data)
        self.prune()
        self.newIMU = True
        if self.newRadio:
            self.pubMatrix()

    def prune(self):
        self.radioLog = self.radioLog[:maxLog]
        self.imuLog = self.imuLog[:maxLog]

    def pubMatrix(self):
        if(len(self.radioLog) > 2) and (len(self.imuLog) > 2):
            out = CommandMatrix()
            out.ax_c1 = (self.imuLog[1].ax - self.imuLog[0].ax) / (self.radioLog[1].chan1 - self.radioLog[0].chan1)
            out.ay_c1 = (self.imuLog[1].ay - self.imuLog[0].ay) / (self.radioLog[1].chan1 - self.radioLog[0].chan1)
            out.az_c1 = (self.imuLog[1].az - self.imuLog[0].az) / (self.radioLog[1].chan1 - self.radioLog[0].chan1)

            out.ax_c2 = (self.imuLog[1].ax - self.imuLog[0].ax) / (self.radioLog[1].chan2 - self.radioLog[0].chan2)
            out.ay_c2 = (self.imuLog[1].ay - self.imuLog[0].ay) / (self.radioLog[1].chan2 - self.radioLog[0].chan2)
            out.az_c2 = (self.imuLog[1].az - self.imuLog[0].az) / (self.radioLog[1].chan2 - self.radioLog[0].chan2)
            
            out.ax_c3 = (self.imuLog[1].ax - self.imuLog[0].ax) / (self.radioLog[1].chan3 - self.radioLog[0].chan3)
            out.ay_c3 = (self.imuLog[1].ay - self.imuLog[0].ay) / (self.radioLog[1].chan3 - self.radioLog[0].chan3)
            out.az_c3 = (self.imuLog[1].az - self.imuLog[0].az) / (self.radioLog[1].chan3 - self.radioLog[0].chan3)
            
            out.ax_c4 = (self.imuLog[1].ax - self.imuLog[0].ax) / (self.radioLog[1].chan4 - self.radioLog[0].chan4)
            out.ay_c4 = (self.imuLog[1].ay - self.imuLog[0].ay) / (self.radioLog[1].chan4 - self.radioLog[0].chan4)
            out.az_c4 = (self.imuLog[1].az - self.imuLog[0].az) / (self.radioLog[1].chan4 - self.radioLog[0].chan4)
            
            out.ax_c5 = (self.imuLog[1].ax - self.imuLog[0].ax) / (self.radioLog[1].chan5 - self.radioLog[0].chan5)
            out.ay_c5 = (self.imuLog[1].ay - self.imuLog[0].ay) / (self.radioLog[1].chan5 - self.radioLog[0].chan5)
            out.az_c5 = (self.imuLog[1].az - self.imuLog[0].az) / (self.radioLog[1].chan5 - self.radioLog[0].chan5)
            
            out.ax_c6 = (self.imuLog[1].ax - self.imuLog[0].ax) / (self.radioLog[1].chan6 - self.radioLog[0].chan6)
            out.ay_c6 = (self.imuLog[1].ay - self.imuLog[0].ay) / (self.radioLog[1].chan6 - self.radioLog[0].chan6)
            out.az_c6 = (self.imuLog[1].az - self.imuLog[0].az) / (self.radioLog[1].chan6 - self.radioLog[0].chan6)
            
            out.ax_c7 = (self.imuLog[1].ax - self.imuLog[0].ax) / (self.radioLog[1].chan7 - self.radioLog[0].chan7)
            out.ay_c7 = (self.imuLog[1].ay - self.imuLog[0].ay) / (self.radioLog[1].chan7 - self.radioLog[0].chan7)
            out.az_c7 = (self.imuLog[1].az - self.imuLog[0].az) / (self.radioLog[1].chan7 - self.radioLog[0].chan7)
            
            out.ax_c8 = (self.imuLog[1].ax - self.imuLog[0].ax) / (self.radioLog[1].chan8 - self.radioLog[0].chan8)
            out.ay_c8 = (self.imuLog[1].ay - self.imuLog[0].ay) / (self.radioLog[1].chan8 - self.radioLog[0].chan8)
            out.az_c8 = (self.imuLog[1].az - self.imuLog[0].az) / (self.radioLog[1].chan8 - self.radioLog[0].chan8)

            # Euler angles

            out.aex_c1 = (self.imuLog[1].aex - self.imuLog[0].aex) / (self.radioLog[1].chan1 - self.radioLog[0].chan1)
            out.aey_c1 = (self.imuLog[1].aey - self.imuLog[0].aey) / (self.radioLog[1].chan1 - self.radioLog[0].chan1)
            out.aez_c1 = (self.imuLog[1].aez - self.imuLog[0].aez) / (self.radioLog[1].chan1 - self.radioLog[0].chan1)

            out.aex_c2 = (self.imuLog[1].aex - self.imuLog[0].aex) / (self.radioLog[1].chan2 - self.radioLog[0].chan2)
            out.aey_c2 = (self.imuLog[1].aey - self.imuLog[0].aey) / (self.radioLog[1].chan2 - self.radioLog[0].chan2)
            out.aez_c2 = (self.imuLog[1].aez - self.imuLog[0].aez) / (self.radioLog[1].chan2 - self.radioLog[0].chan2)
            
            out.aex_c3 = (self.imuLog[1].aex - self.imuLog[0].aex) / (self.radioLog[1].chan3 - self.radioLog[0].chan3)
            out.aey_c3 = (self.imuLog[1].aey - self.imuLog[0].aey) / (self.radioLog[1].chan3 - self.radioLog[0].chan3)
            out.aez_c3 = (self.imuLog[1].aez - self.imuLog[0].aez) / (self.radioLog[1].chan3 - self.radioLog[0].chan3)
            
            out.aex_c4 = (self.imuLog[1].aex - self.imuLog[0].aex) / (self.radioLog[1].chan4 - self.radioLog[0].chan4)
            out.aey_c4 = (self.imuLog[1].aey - self.imuLog[0].aey) / (self.radioLog[1].chan4 - self.radioLog[0].chan4)
            out.aez_c4 = (self.imuLog[1].aez - self.imuLog[0].aez) / (self.radioLog[1].chan4 - self.radioLog[0].chan4)
            
            out.aex_c5 = (self.imuLog[1].aex - self.imuLog[0].aex) / (self.radioLog[1].chan5 - self.radioLog[0].chan5)
            out.aey_c5 = (self.imuLog[1].aey - self.imuLog[0].aey) / (self.radioLog[1].chan5 - self.radioLog[0].chan5)
            out.aez_c5 = (self.imuLog[1].aez - self.imuLog[0].aez) / (self.radioLog[1].chan5 - self.radioLog[0].chan5)
            
            out.aex_c6 = (self.imuLog[1].aex - self.imuLog[0].aex) / (self.radioLog[1].chan6 - self.radioLog[0].chan6)
            out.aey_c6 = (self.imuLog[1].aey - self.imuLog[0].aey) / (self.radioLog[1].chan6 - self.radioLog[0].chan6)
            out.aez_c6 = (self.imuLog[1].aez - self.imuLog[0].aez) / (self.radioLog[1].chan6 - self.radioLog[0].chan6)
            
            out.aex_c7 = (self.imuLog[1].aex - self.imuLog[0].aex) / (self.radioLog[1].chan7 - self.radioLog[0].chan7)
            out.aey_c7 = (self.imuLog[1].aey - self.imuLog[0].aey) / (self.radioLog[1].chan7 - self.radioLog[0].chan7)
            out.aez_c7 = (self.imuLog[1].aez - self.imuLog[0].aez) / (self.radioLog[1].chan7 - self.radioLog[0].chan7)
            
            out.aex_c8 = (self.imuLog[1].aex - self.imuLog[0].aex) / (self.radioLog[1].chan8 - self.radioLog[0].chan8)
            out.aey_c8 = (self.imuLog[1].aey - self.imuLog[0].aey) / (self.radioLog[1].chan8 - self.radioLog[0].chan8)
            out.aez_c8 = (self.imuLog[1].aez - self.imuLog[0].aez) / (self.radioLog[1].chan8 - self.radioLog[0].chan8)

            self.newIMU = False
            self.newRadio = False
            self.pub.publish(out)   


if __name__ == "__main__":
    try:
        m = Translator()
    except rospy.ROSInterruptException:
        pass
