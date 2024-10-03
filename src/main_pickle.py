#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String, Bool, Int16
from macstouch.msg import order, state, vision_info, control_info, error, material

import os, sys

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import numpy as np

# from macstouch_config import MaterialList, WeightLimit, ModeList
# from task_maker import Task

MaterialList = ["bread", "meat", "cheeze", "pickle", "onion", "sauce", "tomato", "lettuce"]
WeightLimit = [10, 10, 10, 10, 10, 10, 10, 10]
ModeList = ["init_pos", "tool_get", "vision", "pnp", "tool_return"]


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
        self.vision_sub = rospy.Subscriber('/pick_coord', vision_info, self.vision_callback, queue_size=1)
        self.coords = [{'ready': False, 'grip_mode': None, 'coord': None, 'size': None}] * len(MaterialList)

    def vision_callback(self, msg):
        self.coords[msg.material] = {'ready': True, 'grip_mode': msg.grip_mode, 'coord': msg.coord, 'size': msg.size}


class Control:
    def __init__(self) -> None:
        self.state_sub = rospy.Subscriber('/robot_state', state, self.state_callback, queue_size=1)
        self.done_sub = rospy.Subscriber('/done', Bool, self.done_callback)
        self.error_pub = rospy.Publisher('/error_info', error)

        self.control_done = False

    def state_callback(self, msg):
        pass

    def done_callback(self, msg):
        self.control_done = True

    def error_check(self):
        pass


class Request:
    def __init__(self):
        self.vision_pub = rospy.Publisher('/vision_req', Int16, queue_size=1, latch=True)
        self.control_pub = rospy.Publisher('/control_req', control_info)
        self.workspace_pub = rospy.Publisher('/workspace', String)

    def vision_req(self, material):
        self.vision_pub.publish(material)

    def control_req(self, mode, material=None, grip_mode=None, coord=None, size=None):
        request = control_info()

        if mode == 0: # init_pos
            request.mode = ModeList[mode]
            request.material = -1
            request.grip_mode = 'x'
            request.coord = [0, 0, 0, 0, 0, 0]
            request.grip_size = 30

        elif mode == 1: # tool_return
            request.mode = ModeList[mode]
            request.material = material
            request.grip_mode = 'x'
            request.coord = [0, 0, 0, 0, 0, 0]
            request.grip_size = 0

        elif mode == 2: # vision
            request.mode = ModeList[mode]
            request.material = material
            request.grip_mode = 'x'
            request.coord = [0, 0, 0, 0, 0, 0]
            request.grip_size = 0

        elif mode == 3: # pnp
            request.mode = ModeList[mode]
            request.material = material
            request.grip_mode = "grip_mode"
            request.coord = coord
            request.grip_size = 30

        elif mode == 4: # tool_return
            request.mode = ModeList[mode]
            request.material = material
            request.grip_mode = 'x'
            request.coord = [0, 0, 0, 0, 0, 0]
            request.grip_size = 0

        self.control_pub.publish(request)

    def workspace_req(self, args):
        pass

    def task_control(self, step):
        pass


def main():
    global IngId, TaskList
    rospy.init_node("main_node")

    req = Request()

    gui = GUI()
    workspace = WorkSpace()
    vision = Vision()
    control = Control()

    # task = Task()
    step = 0
    status = None

    mode = 2
    material = 3

    while not rospy.is_shutdown():
        pickle = input("Press 'y' to start pickle test:     ")

        if pickle == 'y':

            # print("step 1: init_pos")

            # # control: init_pos
            # req.control_req(mode=0)
            # while control.control_done == False:
            #     rospy.sleep(0.2)

            print("step 2: tool_get")
            # control: tool_get
            control.control_done = False
            req.control_req(mode=1, material=material)
            while control.control_done == False:
                rospy.sleep(0.2)

            print("step 3: control vision")
            # control: vision - pickle
            control.control_done = False
            req.control_req(mode=2, material=material)
            while control.control_done == False:
                rospy.sleep(0.2)


            # rospy.sleep(2)
            # print("step 4: vision")
            # # vision: pickle
            # control.control_done = False
            # req.vision_req(material=material)
            # while vision.coords[material]['ready'] == False:
            #     rospy.sleep(0.2)

            # print("step 5: control pnp")
            # # control: pnp - pickle
            # vision.coords[material]['ready'] = False
            # req.control_req(mode=3, material=material,
            #                 grip_mode=vision.coords[material]['grip_mode'],
            #                 coord=vision.coords[material]['coord'],
            #                 size=vision.coords[material]['size'])
            # while control.control_done == False:
            #     rospy.sleep(0.2)

            # print("step 6: tool_return")
            # # control: tool_return
            # control.control_done = False
            # req.control_req(mode=4, material=material)
            # while control.control_done == False:
            #     rospy.sleep(0.2)

            # print("step 7: init_pos")
            # # control: init_pos
            # req.control_req(mode=0)
            # while control.control_done == False:
            #     rospy.sleep(0.2)
    

if __name__ == "__main__":
    main()