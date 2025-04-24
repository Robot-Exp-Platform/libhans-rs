from robot_behavior import Arm, ArmPreplannedMotion, ArmPreplannedMotionExt

class HansRobot(Arm, ArmPreplannedMotion, ArmPreplannedMotionExt):
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

    def move_linear_with_euler(self, pose: list[float], speed: float) -> None:
        """以笛卡尔坐标系（欧拉角）移动机器人
        Args:
            pose: 位姿列表 [x, y, z, rx, ry, rz]
            speed: 运动速度（0.0~1.0）  
        """
        ...
        
    def move_linear_with_euler_async(self, pose: list[float], speed: float) -> None:
        """以笛卡尔坐标系（欧拉角）异步移动机器人
        Args:
            pose: 位姿列表 [x, y, z, rx, ry, rz]
            speed: 运动速度（0.0~1.0）
        """
        ...

    def move_linear_path_with_euler(self, pose: list[list[float]], speed: float) -> None:
        """以笛卡尔坐标系（欧拉角）移动机器人
        Args:
            pose: 位姿列表 [[x, y, z, rx, ry, rz], ...]
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def set_speed(self, speed: float) -> None:
        """设置运动速度
        Args:
            speed: 速度系数（0.0~1.0）
        """
        ...
        
    def read_joint(self) -> list[float]:
        """读取机器人关节角度
        Returns:
            关节角度列表
        """
        ...
        
    def read_joint_vel(self) -> list[float]:
        """读取机器人关节速度
        Returns:
            关节速度列表
        """
        ...
    
    def read_cartesian_euler(self) -> list[float]:
        """读取机器人笛卡尔坐标系（欧拉角）
        Returns:
            位姿列表 [x, y, z, rx, ry, rz]
        """
        ...
        
    def read_cartesian_vel(self) -> list[float]:
        """读取机器人笛卡尔坐标系速度
        Returns:
            速度列表 [vx, vy, vz, wx, wy, wz]
        """
        ...
    ...