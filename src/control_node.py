#!/usr/bin/env python
import time, math, rospy
from vradio.msg import Radio
from copter_control.msg import CopterDirect
from copter_control.msg import CommandMatrix
import numpy as np

# adjust per settings
def translationMatrix(ax, ay, az, aex, aey, aez, matrix):
    #       ax                  ay              az              aex             aey          aez           c (middle stick pos, no acceleration)
    tmat = [[matrix.ax_c1, matrix.ay_c1, matrix.az_c1,     matrix.aex_c1, matrix.aey_c1, matrix.aez_c1,    1500],    #channel 1
            [matrix.ax_c2, matrix.ay_c2, matrix.az_c2,     matrix.aex_c2, matrix.aey_c2, matrix.aez_c2,    1500],    #channel 2
            [matrix.ax_c3, matrix.ay_c3, matrix.az_c3,     matrix.aex_c3, matrix.aey_c3, matrix.aez_c3,    1500],    #channel 3
            [matrix.ax_c4, matrix.ay_c4, matrix.az_c4,     matrix.aex_c4, matrix.aey_c4, matrix.aez_c4,    1500],    #channel 4
            [matrix.ax_c5, matrix.ay_c5, matrix.az_c5,     matrix.aex_c5, matrix.aey_c5, matrix.aez_c5,    1500],    #channel 5
            [matrix.ax_c6, matrix.ay_c6, matrix.az_c6,     matrix.aex_c6, matrix.aey_c6, matrix.aez_c6,    1500],    #channel 6
            [matrix.ax_c7, matrix.ay_c7, matrix.az_c7,     matrix.aex_c7, matrix.aey_c7, matrix.aez_c7,    1500],    #channel 7
            [matrix.ax_c8, matrix.ay_c8, matrix.az_c8,     matrix.aex_c8, matrix.aey_c8, matrix.aez_c8,    1500]]    #channel 8
                #tmat = np.transpose(tmat)
    mat = np.transpose([[ax, ay, az, aex, aey, aez, 1]])
    return np.dot(tmat, mat)

class Translator:
    def __init__(self):
        rospy.init_node("translation", anonymous=False)
        self.pub = rospy.Publisher("radio", Radio, queue_size=10)
        controller = rospy.get_param("~controller")
        matrix = rospy.get_param("~matrix")
        self.sub = rospy.Subscriber(controller, CopterDirect, self.callback, queue_size=10)
        self.subMatrix = rospy.Subscriber(matrix, CommandMatrix, self.matrixCallback, queue_size=10)
        self.lastMatrix = None
        rate = rospy.Rate(10)
        print("Done")
        rospy.spin()

    def matrixCallback(self, data):
        self.lastMatrix = data
            
    def callback(self, data):
        if self.lastMatrix != None:
            com = Radio()
            m = translationMatrix(data.ax, data.ay, data.az, data.aex, data.aey, data.aez, self.lastMatrix)
            com.chan1 = m[0]
            com.chan2 = m[1]
            com.chan3 = m[2]
            com.chan4 = m[3]
            com.chan5 = m[4]
            com.chan6 = m[5]
            com.chan7 = m[6]
            com.chan8 = m[7]
            rospy.loginfo(com)
            self.pub.publish(com)

if __name__ == "__main__":
    try:
        m = Translator()
    except rospy.ROSInterruptException:
        pass
