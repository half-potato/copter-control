import Serial, rospy

port = rospy.get_param("~port")
ser = Serial.Serial(port)
