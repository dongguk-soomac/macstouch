#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String
from macstouch.msg import order, state, vision_info, control_info, error, material

import os, sys

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import numpy as np

from macstouch_config import *
from task_maker import Task


OrderList = []
IngId = None

TaskList = []


class Burger:
    def __init__(self) -> None:
        self.id = None
        self.menu = None
        self.state = "NotYet" # NotYet / Ing / Done


class GUI:
    def __init__(self) -> None:
        self.order_sub = rospy.Subscriber('/id_order', order, self.order_callback, queue_size=1)
        self.order_pub = rospy.Publisher('/id_complete', order)

    def order_callback(self, msg):
        global OrderList

        burger = Burger()
        burger.id = msg.id
        burger.menu = msg.menu

        OrderList.append(burger)

        print(f"Order ID{msg.id} added !!")

    def order_complete(self, id):
        global OrderList
        OrderList.pop(0)

        complete_msg = order()
        complete_msg.id = id
        self.order_pub.publish(complete_msg)



class WorkSpace:
    def __init__(self) -> None:
        self.mat_sub = rospy.Subscriber('/material_info', material, self.mat_callback, queue_size=1)
        self.mat_pub = rospy.Publisher('/lack', String)
        self.weights_limit = np.array(WeightLimit)
        self.weights = []

    def mat_callback(self, msg):
        self.weights = np.array(msg.weights)
        self.mat_check()

    def mat_check(self):
        lack_mat = ""
        dis = self.weights - self.weights_limit
        for idx, dis in enumerate(dis):
            if dis < 0:
                lack_mat.join(MaterialList[idx]+',')

        if len(lack_mat) > 0:
            self.mat_pub.publish(lack_mat)

    def pickup_check(self):
        pass


class Vision:
    def __init__(self) -> None:
        self.vision_sub = rospy.Subscriber('/pick_coord', vision_info. self.vision_callback, queue_size=1)
        self.coords = [{'ready': False, 'grip_mode': None, 'coord': None, 'size': None} * len(MaterialList)]

    def vision_callback(self, msg):
        self.coords[msg.material] = {'ready': True, 'grip_mode': msg.grip_mode, 'coord': msg.coord, 'size': msg.size}


class Control:
    def __init__(self) -> None:
        self.state_sub = rospy.Subscriber('/robot_state', state, self.state_callback, queue_size=1)
        self.done_sub = rospy.Subscriber('/done', bool, self.done_callback)
        self.error_pub = rospy.Publisher('/error_info', error)

    def state_callback(self, msg):
        pass

    def done_callback(self, msg):
        pass

    def error_check(self):
        pass


class Request:
    def __init__(self):
        self.vision_pub = rospy.Publisher('/vision_req', String)
        self.control_pub = rospy.Publisher('/control_req', control_info)
        self.workspace_pub = rospy.Publisher('/workspace', String)

    def vision_req(self, args):
        pass

    def control_req(self, args):
        pass

    def workspace_req(self, args):
        pass


def main():
    rospy.init_node("main_node")

    req = Request()

    gui = GUI()
    workspace = WorkSpace()
    vision = Vision()
    control = Control()

    while rospy.is_shutdown():
        if IngId is None and len(OrderList) != 0:
            IngId = OrderList[0].id



if __name__ == "__main__":
    main()