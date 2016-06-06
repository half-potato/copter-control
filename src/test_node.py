#!/usr/bin/env python
import random, rospy, math, time
from vradio.msg import Radio
from copter_control.msg import CopterDirect
from std_msgs.msg import Float32

def talk():
    startTime = time.time()
    rospy.init_node("random_controller", anonymous=False)
    pub = rospy.Publisher("fake_controls", CopterDirect)
    vpub = rospy.Publisher("fake_voltages", Float32)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        voltage = Float32()
        voltage.data = 11.1 * ((time.time() - startTime) % 1)
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
        vpub.publish(voltage)
        rate.sleep()

if __name__ == "__main__":
    try:
        talk()
    except rospy.ROSInterruptException:
        pass
