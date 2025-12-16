#!/usr/bin/env python

import logging
import time
from functools import cached_property
from typing import Any
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ..robot import Robot
from .config_fairino1 import fairino1Config
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

logger = logging.getLogger(__name__)
        
class fairino1(Robot, Node):  # 同时继承 Robot 和 Node
    config_class = fairino1Config
    name = "fairino1"

    def __init__(self, config: fairino1Config):
        # 检查 ROS2 是否已经初始化，如果没有则初始化
        if not rclpy.ok():
            rclpy.init()
        
        # 首先初始化 Node
        Node.__init__(self, "fairino1_node")  # 初始化 ROS2 节点
        
        # 然后初始化 Robot
        Robot.__init__(self, config)

        self.config = config


        # 创建 ROS2 订阅者，订阅 /robot_current_state 话题
        # 使用最兼容的 QoS 设置 - 尝试匹配 servo2fairino1.py 的发布设置
        # 默认 QoS（通常为可靠传输）
        qos_profile = QoSProfile(
            depth=10,  # 队列大小
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 尝试使用 BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/robot_current_state',
            self.listener_callback,
            qos_profile
        )
        logger.info("Subscribed to /robot_current_state with BEST_EFFORT QoS")
        

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

    def listener_callback(self, msg):
        """回调函数，当接收到 /robot_current_state 数据时更新 joint_angles"""
        self.joint_angles = list(msg.data)  # 转换为 list 以确保是可变的
        logger.debug(f"Updated joint angles: {self.joint_angles}")

        # 更新 bus 字典
        self.bus["Joint1"] = self.joint_angles[0] 
        self.bus["Joint2"] = self.joint_angles[1] 
        self.bus["Joint3"] = self.joint_angles[2] 
        self.bus["Joint4"] = self.joint_angles[3] 
        self.bus["Joint5"] = self.joint_angles[4] 
        self.bus["Joint6"] = self.joint_angles[5] 

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

        # 处理任何待处理的 ROS2 回调，确保数据是最新的
        rclpy.spin_once(self, timeout_sec=0.001)

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
        pass

    def disconnect(self) -> None:
        """断开连接并清理资源"""
        logger.info(f"Disconnecting {self.name}...")
        try:
            # 先销毁订阅者
            if hasattr(self, 'sub') and self.sub is not None:
                self.destroy_subscription(self.sub)
                self.sub = None
            
            # 再关闭节点
            self.shutdown()
            
            logger.info(f"{self.name} disconnected successfully.")
        except Exception as e:
            logger.warning(f"Error during disconnect: {e}")

    def shutdown(self):
        """关闭ROS2节点"""
        try:
            # 检查 ROS2 是否还在运行
            if rclpy.ok():
                # 先销毁节点
                self.destroy_node()
                # 不调用 rclpy.shutdown()，让主程序处理
            logger.info(f"{self.name} shutdown successfully.")
        except Exception as e:
            logger.warning(f"Error during shutdown: {e}")