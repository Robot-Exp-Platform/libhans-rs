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

    def move_joint(self, joint: List[float], speed: float) -> None:
        """以关节角度方式移动机器人
        Args:
            joint: 关节角度列表（长度必须为 HANS_DOF）
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_joint_async(self, joint: List[float], speed: float) -> None:
        """以关节角度方式异步移动机器人
        Args:
            joint: 关节角度列表（长度必须为 HANS_DOF）
            speed: 运动速度（0.0~1.0）
        """
        ...

    def move_joint_rel(self, joint_rel: List[float], speed: float) -> None:
        """以关节角度方式相对移动机器人
        Args:
            joint_rel: 相对关节角度列表（长度必须为 HANS_DOF）
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_joint_rel_async(self, joint_rel: List[float], speed: float) -> None:
        """以关节角度方式异步相对移动机器人
        Args:
            joint_rel: 相对关节角度列表（长度必须为 HANS_DOF）
            speed: 运动速度（0.0~1.0）
        """
        ...
    
    def move_joint_path(self, joints: List[List[float]], speed: float) -> None:
        """以关节角度方式移动机器人
        Args:
            joints: 关节角度列表
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_cartesian(self, pose: List[float] | tuple[List[float], List[float]] | tuple[List[float], List[float], float], speed: float) -> None:
        """以笛卡尔坐标系移动机器人
        Args:
            pose: 位姿列表 
                [x, y, z] 位置
                ([x, y, z], [rx, ry, rz]) 欧拉角
                [x, y, z, qx, qy, qz, qw] 四元数
                [float; 16] 4x4 齐次变换矩阵，按列存储
                ([x, y, z], [rx, ry, rz], theta) 轴角描述
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_cartesian_async(self, pose: List[float] | tuple[List[float], List[float]] | tuple[List[float], List[float], float], speed: float) -> None:
        """以笛卡尔坐标系异步移动机器人
        Args:
            pose: 位姿列表 
                [x, y, z] 位置
                ([x, y, z], [rx, ry, rz]) 欧拉角
                [x, y, z, qx, qy, qz, qw] 四元数
                [float; 16] 4x4 齐次变换矩阵，按列存储
                ([x, y, z], [rx, ry, rz], theta) 轴角描述
            speed: 运动速度（0.0~1.0）
        """
        ...

    def move_cartesian_rel(self, pose_rel: List[float] | tuple[List[float], List[float]] | tuple[List[float], List[float], float], speed: float) -> None:
        """以笛卡尔坐标系相对移动机器人
        Args:
            pose_rel: 相对位姿列表 
                [dx, dy, dz] 位置
                ([dx, dy, dz], [drx, dry, drz]) 欧拉角
                [dx, dy, dz, qx, qy, qz, qw] 四元数
                [float; 16] 4x4 齐次变换矩阵，按列存储
                ([dx, dy, dz], [drx, dry, drz], dtheta) 轴角描述
            speed: 运动速度（0.0~1.0）
        """
        ...
    
    def move_cartesian_rel_async(self, pose_rel: List[float] | tuple[List[float], List[float]] | tuple[List[float], List[float], float], speed: float) -> None:
        """以笛卡尔坐标系异步相对移动机器人
        Args:
            pose_rel: 相对位姿列表 
                [dx, dy, dz] 位置
                ([dx, dy, dz], [drx, dry, drz]) 欧拉角
                [dx, dy, dz, qx, qy, qz, qw] 四元数
                [float; 16] 4x4 齐次变换矩阵，按列存储
                ([dx, dy, dz], [drx, dry, drz], dtheta) 轴角描述
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_cartesian_path(self, poses: List[List[float] | tuple[List[float], List[float]] | tuple[List[float], List[float], float]], speed: float) -> None:
        """以笛卡尔坐标系移动机器人
        Args:
            poses: 位姿列表 
                [[dx, dy, dz] 位置]
                [([dx, dy, dz], [drx, dry, drz]) 欧拉角]
                [[dx, dy, dz, qx, qy, qz, qw] 四元数]
                [[float; 16] 4x4 齐次变换矩阵，按列存储]
                [([dx, dy, dz], [drx, dry, drz], dtheta) 轴角描述]
            speed: 运动速度（0.0~1.0）
        """
        ...
        
    def move_path_from_file(self, path: str, speed: float) -> None:
        """从文件中读取关节角度路径并执行
        Args:
            path: 文件路径
            speed: 运动速度（0.0~1.0）
        """
        ...

    def move_linear_with_euler(self, pose: List[float], speed: float) -> None:
        """以笛卡尔坐标系（欧拉角）移动机器人
        Args:
            pose: 位姿列表 [x, y, z, rx, ry, rz]
            speed: 运动速度（0.0~1.0）  
        """
        ...
        
    def move_linear_with_euler_async(self, pose: List[float], speed: float) -> None:
        """以笛卡尔坐标系（欧拉角）异步移动机器人
        Args:
            pose: 位姿列表 [x, y, z, rx, ry, rz]
            speed: 运动速度（0.0~1.0）
        """
        ...

    def move_linear_path_with_euler(self, pose: List[List[float]], speed: float) -> None:
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
        
    def read_joint(self) -> List[float]:
        """读取机器人关节角度
        Returns:
            关节角度列表
        """
        ...
        
    def read_joint_vel(self) -> List[float]:
        """读取机器人关节速度
        Returns:
            关节速度列表
        """
        ...
    
    def read_cartesian_euler(self) -> List[float]:
        """读取机器人笛卡尔坐标系（欧拉角）
        Returns:
            位姿列表 [x, y, z, rx, ry, rz]
        """
        ...
        
    def read_cartesian_vel(self) -> List[float]:
        """读取机器人笛卡尔坐标系速度
        Returns:
            速度列表 [vx, vy, vz, wx, wy, wz]
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