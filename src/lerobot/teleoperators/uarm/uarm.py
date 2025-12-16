#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import logging
import time
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ..teleoperator import Teleoperator
from .config_uarm import uarmConfig
import numpy as np

logger = logging.getLogger(__name__)
_uarm_latest_gripper = 0.0
def get_uarm_gripper_state():
    """模块级函数,可以被外部直接导入使用"""
    global _uarm_latest_gripper
    return _uarm_latest_gripper

class uarm(Teleoperator, Node):
    config_class = uarmConfig
    name = "uarm"

    def __init__(self, config: uarmConfig):
        # 检查 ROS2 是否已经初始化，如果没有则初始化
        if not rclpy.ok():
            rclpy.init()
        
        # 初始化 ROS2 节点
        Node.__init__(self, "uarm_node")  # 初始化 ROS2 节点
        Teleoperator.__init__(self, config)  # 初始化 Teleoperator

        self.config = config

        # 初始化舵机角度数组
        self.joint_angles = [0.0] * 7
        self.init_qpos = np.array([-13.904, -107.669, 99.473, -82.616, -91.726, 85.256])
        # _uarm_latest_gripper = 0.0

        # 创建 ROS2 订阅者，订阅 /servo_angles 话题
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/servo_angles',
            self.listener_callback,
            10
        )
        logger.info("Subscribed to /servo_angles")

        # 初始化总线字典（bus）
        self.bus = {
            "Joint1": self.joint_angles[0],
            "Joint2": self.joint_angles[1],
            "Joint3": self.joint_angles[2],
            "Joint4": self.joint_angles[3],
            "Joint5": self.joint_angles[4],
            "Joint6": self.joint_angles[5],
            "gripper": self.joint_angles[6],
        }

    def listener_callback(self, msg):
        """回调函数，当接收到 /servo_angles 数据时更新 joint_angles"""
        self.joint_angles = msg.data
        logger.debug(f"Updated joint angles: {self.joint_angles}")

        # 更新 bus 字典
        self.bus["Joint1"] = self.joint_angles[0] 
        self.bus["Joint2"] = self.joint_angles[1] 
        self.bus["Joint3"] = self.joint_angles[2] 
        self.bus["Joint4"] = self.joint_angles[3] 
        self.bus["Joint5"] = self.joint_angles[4] 
        self.bus["Joint6"] = self.joint_angles[5] 
        self.bus["gripper"] = self.joint_angles[6]

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return True  # 假设 ROS2 订阅者已连接

    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting {self.name}...")
        # ROS2 已在 __init__ 中初始化
        logger.info(f"{self.name} connected successfully.")

    @property
    def is_calibrated(self) -> bool:
        return True  # 假设始终已校准

    def calibrate(self) -> None:
        logger.info(f"Calibrating {self.name}...")
        # 添加校准逻辑（如果需要）
        logger.info(f"{self.name} calibrated successfully.")

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        """获取动作数据，键名格式必须是 'motor_name.pos'"""
        start = time.perf_counter()

        try:
            # 处理 ROS2 回调
            rclpy.spin_once(self, timeout_sec=0.001)

             # 更新 joint_angles[6]，根据条件进行设置
            if self.joint_angles[6] < -20:
                self.joint_angles[6] = 1
            else:
                self.joint_angles[6] = 0
            
            global _uarm_latest_gripper
            _uarm_latest_gripper = float(self.joint_angles[6])
            
            # 更新 bus 字典
            self.bus = {
                "Joint1": self.joint_angles[0]+ self.init_qpos[0],
                "Joint2": self.joint_angles[1]+ self.init_qpos[1],
                "Joint3": self.joint_angles[2]+ self.init_qpos[2],
                "Joint4": self.joint_angles[3]+ self.init_qpos[3],
                "Joint5": self.joint_angles[4]+ self.init_qpos[4],
                "Joint6": self.joint_angles[5]+ self.init_qpos[5],
                "gripper": self.joint_angles[6],
            }

            # 构建动作字典，键名必须是 "motor_name.pos" 格式
            action = {}
            for motor_name, angle in self.bus.items():
                action[f"{motor_name}.pos"] = angle

        except Exception as e:
            logger.error(f"Error getting action: {e}")
            return {}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")

        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

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