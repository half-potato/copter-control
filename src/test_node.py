#!/usr/bin/env python
import random, rospy, math, time
from vradio.msg import Radio
from copter_control.msg import CopterDirect

def talk():
    rospy.init_node("random_controller", anonymous=False)
    pub = rospy.Publisher("fake_controls", CopterDirect)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        randmsg = CopterDirect()
        #randmsg.header = str(int(math.floor(time.time())))
        randmsg.ax = random.random() * 10
        randmsg.ay = random.random() * 10
        randmsg.az = random.random() * 10
        randmsg.aex = random.random() * 10
        randmsg.aey = random.random() * 10
        randmsg.aez = random.random() * 10
        rospy.loginfo("Message: %s", randmsg)
        pub.publish(randmsg)
        rate.sleep()

if __name__ == "__main__":
    try:
        talk()
    except rospy.ROSInterruptException:
        pass
