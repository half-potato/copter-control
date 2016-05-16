#!/usr/bin/env python
import time, math, rospy
from vradio.msg import Radio
from copter_control.msg import CopterDirect
import numpy as np

# adjust per settings
def translationMatrix(ax, ay, az, aex, aey, aez):
    #       ax ay az    aex aey aez   c
    tmat = [[1, 1, 1,     1, 1, 1,    1],    #channel 1
            [1, 1, 1,     1, 1, 1,    1],    #channel 2
            [1, 1, 1,     1, 1, 1,    1],    #channel 3
            [1, 1, 1,     1, 1, 1,    1],    #channel 4
            [1, 1, 1,     1, 1, 1,    1],    #channel 5
            [1, 1, 1,     1, 1, 1,    1],    #channel 6
            [1, 1, 1,     1, 1, 1,    1],    #channel 7
            [1, 1, 1,     1, 1, 1,    1]]    #channel 8
    #tmat = np.transpose(tmat)
    mat = np.transpose([[ax, ay, az, aex, aey, aez, 1]])
    return np.dot(tmat, mat)

class Translator:
    def __init__(self):
        rospy.init_node("translation", anonymous=False)
        self.pub = rospy.Publisher("radio", Radio, queue_size=10)
        controller = rospy.get_param("~controller")
        print(controller)
        self.sub = rospy.Subscriber("fake_controls", CopterDirect, self.callback, queue_size=10)
        rate = rospy.Rate(10)
        print("Done")
        rospy.spin()
            
    def callback(self, data):
        com = Radio()
        m = translationMatrix(data.ax, data.ay, data.az, data.aex, data.aey, data.aez)
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
