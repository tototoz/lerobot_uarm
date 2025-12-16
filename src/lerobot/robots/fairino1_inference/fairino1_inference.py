#!/usr/bin/env python

import logging
import time
from functools import cached_property
from typing import Any
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ..robot import Robot
from .config_fairino1_inference import fairino1_inferenceConfig
import sys
# 添加 Fairino SDK 路径
sys.path.append('/home/zhq/LeRobot-Anything-U-Arm/src/uarm/scripts/Follower_Arm/fairino')
from api import RPC
import numpy as np 
from collections import deque

logger = logging.getLogger(__name__)
        
class fairino1_inference(Robot):  
    config_class = fairino1_inferenceConfig
    name = "fairino1_inference"

    def __init__(self, config: fairino1_inferenceConfig):
        super().__init__(config)
        self.config = config

        # ===== 机器人连接 =====
        self.robot_ip = "192.168.58.2"
        self.robot = RPC(self.robot_ip)

        # ===== 机械参数 =====
        self.tool = 0
        self.user = 0

        # ===== ServoJ 保守参数 =====
        self.epos = [0.0, 0.0, 0.0, 0.0]
        self.cmdT = 0.05
        self.filterT = 0.15
        self.gain = 150.0
        
        # ===== 伺服控制状态 =====
        self.servo_started = False
        self.servo_active = False
        
        # ===== 运动规划参数 =====
        self.target_buffer = deque(maxlen=3)
        self.current_target = None
        self.last_sent_position = None
        
        # ===== 平滑滤波参数 =====
        self.position_buffer = deque(maxlen=5)
        self.velocity_buffer = deque(maxlen=3)
        
        # ===== 运动限制 =====
        self.max_joint_velocity = np.array([8.0, 8.0, 8.0, 12.0, 12.0, 12.0])
        self.max_joint_acceleration = np.array([15.0, 15.0, 15.0, 20.0, 20.0, 20.0])
        
        self.joint_angles = [0.0] * 6

        self.bus = {
            "Joint1": self.joint_angles[0],
            "Joint2": self.joint_angles[1],
            "Joint3": self.joint_angles[2],
            "Joint4": self.joint_angles[3],
            "Joint5": self.joint_angles[4],
            "Joint6": self.joint_angles[5],
        }

        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) 
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        for cam in self.cameras.values():
            cam.connect()

    @property
    def is_calibrated(self) -> bool:
        pass

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        """获取机器人的当前观测数据，包括关节角度和摄像头图像"""

        start = time.perf_counter()
        ret, current_joint = self.robot.GetActualJointPosDegree(0)
        logger.debug(f"GetActualJointPosDegree returned: {ret}, current_joint: {current_joint}")
        if ret == 0:
            self.joint_angles = np.array(current_joint)

        # 更新 bus 字典 - 使用当前最新的 joint_angles
        self.bus = {
            "Joint1": self.joint_angles[0],
            "Joint2": self.joint_angles[1],
            "Joint3": self.joint_angles[2],
            "Joint4": self.joint_angles[3],
            "Joint5": self.joint_angles[4],
            "Joint6": self.joint_angles[5],
        }

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read joint angles: {dt_ms:.1f}ms")

        # 初始化观测字典
        obs_dict = {}

        # 捕获摄像头图像
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        # 将机器人关节角度添加到观测字典
        # 关键：键名必须是 "motor_name.pos" 格式，与 observation_features 中定义的一致
        for motor_name, angle in self.bus.items():
            obs_dict[f"{motor_name}.pos"] = angle

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        goal_pos_values = [val for key, val in action.items() if key.endswith(".pos")]

        self.robot.ServoJ(
            joint_pos=goal_pos_values,
            axisPos=self.epos,
            cmdT=self.cmdT,
            
            filterT=self.filterT,
            gain=self.gain
        )

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}




    def disconnect(self) -> None:
        pass

