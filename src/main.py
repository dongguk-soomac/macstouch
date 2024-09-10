#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from macstouch.msg import order, state, vision_info, control_info, error

import os, sys

import numpy as np

class GUI:
    def __init__(self) -> None:
        order_sub = rospy.subscriber('/id_order', order, order_callback, )


class WorkSpace:
    def __init__(self) -> None:
        pass


class Vision:
    def __init__(self) -> None:
        pass


class Control:
    def __init__(self) -> None:
        pass


def main():
    rospy.init_node("main_node")
    gui = GUI()
    workspace = WorkSpace()
    vision = Vision*()
    control = Control()

    rospy.spin()


if __name__ == "__main__":
    main()