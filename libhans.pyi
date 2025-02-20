# libhans.pyi
from typing import List, Optional, Any

class HansRobot:
    """大族机器人
    """

    def __init__(self) -> None: ...

    def __repr__(self) -> str: ...

    def connect(self, ip: str, port: int = 10003) -> None:
        """连接到机器人
        Args:
            ip: 机器人的 IP 地址
            port: 机器人的端口号（默认 10003）
        """
        ...

    def disconnect(self) -> None:
        """断开与机器人的连接"""
        ...

    def move_joint(self, joint: List[float]) -> None:
        """以关节角度方式移动机器人
        Args:
            joint: 关节角度列表（长度必须为 HANS_DOF）
        """
        ...

    def move_joint_rel(self, joint_rel: List[float]) -> None:
        """以关节角度方式相对移动机器人
        Args:
            joint_rel: 相对关节角度列表（长度必须为 HANS_DOF）
        """
        ...

    def move_linear_with_euler(self, pose: List[float]) -> None:
        """以笛卡尔坐标系（欧拉角）移动机器人
        Args:
            pose: 位姿列表 [x, y, z, rx, ry, rz]
        """
        ...

    def move_linear_rel_with_euler(self, pose_rel: List[float]) -> None:
        """以笛卡尔坐标系（欧拉角）相对移动机器人
        Args:
            pose_rel: 相对位姿列表 [dx, dy, dz, drx, dry, drz]
        """
        ...

    def set_speed(self, speed: float) -> None:
        """设置运动速度
        Args:
            speed: 速度系数（0.0~1.0）
        """
        ...

    def version(self) -> str:
        """获取机器人版本信息
        Returns:
            版本号字符串
        """
        ...

    def init(self) -> None:
        """初始化机器人"""
        ...

    def shutdown(self) -> None:
        """关闭机器人"""
        ...

    def enable(self) -> None:
        """使能机器人"""
        ...

    def disable(self) -> None:
        """去使能机器人"""
        ...

    def stop(self) -> None:
        """停止机器人运动"""
        ...

    def resume(self) -> None:
        """恢复机器人运动"""
        ...

    def emergency_stop(self) -> None:
        """紧急停止机器人"""
        ...

    def clear_emergency_stop(self) -> None:
        """清除紧急停止状态"""
        ...

    def is_moving(self) -> bool:
        """检查机器人是否在运动中
        Returns:
            bool: 是否在运动状态
        """
        ...