# client_test
# main_test
#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import json
import os, sys

# msg
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, Float32, String
from macstouch.msg import control_info
from macstouch.msg import action_info


class Client_test():
    def __init__(self):
        rospy.Subscriber('/action_req', action_info, self.callback)
        self.action_done = rospy.Publisher('/action_done', Bool, queue_size=10)
        rospy.spin()

    def callback(self, data):
        print(data)
        rospy.sleep(5)

        print('action_done')
        msg = Bool()
        msg.data = True
        self.action_done.publish(msg)
        


if __name__ == '__main__':
    try:
        rospy.init_node('client_test', anonymous=True)
        rospy.loginfo("client_test_node is on")
        Client_test()

    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')
