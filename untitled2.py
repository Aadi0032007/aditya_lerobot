#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 01:34:41 2024

@author: revolabs
"""

python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/koch.yaml \
  --fps 30 \
  --num-episodes 10 \
  -p "/home/revolabs/Downloads/koch act lego 5 peices output/outputs - 14102024/train/act_koch_test/checkpoints/last/pretrained_model"
  
  
  python lerobot/scripts/control_robot.py record \
 --robot-path lerobot/configs/robot/koch.yaml \
 --fps 30 \
 --repo-id ${HF_USER}/eval_koch_reach_the_object \
 --tags tutorial eval \
 --warmup-time-s 5 \
 --episode-time-s 30 \
 --reset-time-s 30 \
 --num-episodes 10 \
  -p "/home/revolabs/aditya/lerobot/outputs/train/act_koch_reach_the_object/checkpoints/last/pretrained_model"
 
 
 python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/koch.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/koch_test \
  --tags tutorial \
  --warmup-time-s 10 \
  --episode-time-s 30 \
  --reset-time-s 30 \
  --num-episodes 2 \
  --push-to-hub 0
