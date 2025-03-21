#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 00:27:19 2024

@author: aadi
"""

from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
import time

leader_port = "/dev/ttyACM1"
follower_port = "/dev/ttyACM0"

leader_arm = DynamixelMotorsBus(
    port=leader_port,
    motors={
        # name: (index, model)
        "shoulder_pan": (1, "xl330-m077"),
        "shoulder_lift": (2, "xl330-m077"),
        "elbow_flex": (3, "xl330-m077"),
        "wrist_flex": (4, "xl330-m077"),
        "wrist_roll": (5, "xl330-m077"),
        "gripper": (6, "xl330-m077"),
    },
)
    
follower_arm = DynamixelMotorsBus(
    port=follower_port,
    motors={
        # name: (index, model)
        "shoulder_pan": (1, "xl430-w250"),
        "shoulder_lift": (2, "xl430-w250"),
        "elbow_flex": (3, "xl330-m288"),
        "wrist_flex": (4, "xl330-m288"),
        "wrist_roll": (5, "xl330-m288"),
        "gripper": (6, "xl330-m288"),
    },
)

from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
robot = ManipulatorRobot(
    leader_arms={"main": leader_arm},
    follower_arms={"main": follower_arm},
    calibration_dir=".cache/calibration/koch",
    cameras={
        "phone": OpenCVCamera("/dev/video4", fps=30, width=640, height=480),
        "laptop": OpenCVCamera("/dev/video10", fps=30, width=640, height=480),
    },
)
robot.connect()
# rest_position = follower_arm.read("Present_Position")
rest_position = [  0.9667969 ,128.84766 ,  174.99023,   -16.611328,   -4.8339844  ,34.716797 ]

#ffplay -f video4linux2 -framerate 10 -video_size 480x270 /dev/video10
# [  0.9667969 ,128.84766 ,  174.99023,   -16.611328,   -4.8339844  ,34.716797 ]

# [ -2.9882812  136.05469    179.29688     1.7578125    -7.1191406  34.804688 ]