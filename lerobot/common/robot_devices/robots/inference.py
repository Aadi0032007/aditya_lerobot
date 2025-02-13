#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 10 05:26:32 2024

@author: revolabs
"""

from lerobot.common.policies.act.modeling_act import ACTPolicy
import time
from lerobot.scripts.control_robot import busy_wait
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
import torch
import os
import platform

record_time_s = 30
fps = 60

states = []
actions = []

leader_port = "/dev/ttyACM1"
follower_port = "/dev/ttyACM0"
phone_camera = "/dev/video10"
laptop_camera = "/dev/video4"

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

robot = ManipulatorRobot(
    leader_arms={"main": leader_arm},
    follower_arms={"main": follower_arm},
    calibration_dir=".cache/calibration/koch",
    cameras={
        "phone": OpenCVCamera(phone_camera, fps=30, width=640, height=480),
        "laptop": OpenCVCamera(laptop_camera, fps=30, width=640, height=480),
    },
)
robot.connect()
rest_position = follower_arm.read("Present_Position")

def say(text, blocking=False):
    # Check if mac, linux, or windows.
    if platform.system() == "Darwin":
        cmd = f'say "{text}"'
    elif platform.system() == "Linux":
        cmd = f'spd-say "{text}"'
    elif platform.system() == "Windows":
        cmd = (
            'PowerShell -Command "Add-Type -AssemblyName System.Speech; '
            f"(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('{text}')\""
        )

    if not blocking and platform.system() in ["Darwin", "Linux"]:
        # TODO(rcadene): Make it work for Windows
        # Use the ampersand to run command in the background
        cmd += " &"

    os.system(cmd)


inference_time_s = 120
fps = 30
device = "cuda"  # TODO: On Mac, use "mps" or "cpu"
ckpt_path = "/home/revolabs/aditya/lerobot/outputs/train/act_koch_reach_the_object/checkpoints/last/pretrained_model"
policy = ACTPolicy.from_pretrained(ckpt_path)
policy.to(device)


say("I am going to the objects")
for _ in range(inference_time_s * fps):
    start_time = time.perf_counter()
    # print("observation starting")
    # Read the follower state and access the frames from the cameras
    observation = robot.capture_observation()
    # print("observation captured")
    # Convert to pytorch format: channel first and float32 in [0,1]
    # with batch dimension
    for name in observation:
        if "image" in name:
            observation[name] = observation[name].type(torch.float32) / 255
            observation[name] = observation[name].permute(2, 0, 1).contiguous()
            # Convert to a numpy array suitable for OpenCV
            # image = observation[name].cpu().numpy()  # Move to CPU and convert to numpy
            # image = (image * 255).astype("uint8")  # Convert to uint8 format
            # image = image.transpose(1, 2, 0)  # Convert back to HWC format for OpenCV
            # # Display the image using OpenCV
            # cv2.imshow("Video", image)
        
            # # Wait for a short period to simulate a video
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #   break  # Press 'q' to exit the video
        observation[name] = observation[name].unsqueeze(0)
        observation[name] = observation[name].to(device)
    # Compute the next action with the policy
    # based on the current observation
    # print("selecting action")
    action = policy.select_action(observation)
    # print("squeezing action")
    # Remove batch dimension
    action = action.squeeze(0)
    # print("action to cpu")
    # Move to cpu, if not already the case
    action = action.to("cpu")
    # print("action to robot")
    # Order the robot to move
    robot.send_action(action)
    dt_s = time.perf_counter() - start_time  
    busy_wait(1 / fps - dt_s)
    # print("end")

say("I have reached the object")

current_pos = follower_arm.read("Present_Position")
steps = 30
for i in range(1, steps + 1):
    intermediate_pos = current_pos + (rest_position - current_pos) * (i / steps)
    follower_arm.write("Goal_Position", intermediate_pos)
    time.sleep(0.1) #try busy_wait


