from robot_behavior import (
    Arm,
    ArmState,
    FlangeMotion,
    FlangeTrajectory,
    JointMotion,
    JointSample,
    JointState,
    LoadState,
    MotionType,
    Pose,
    SpatialSample,
    SpatialState,
    Vec,
)


class HansS30(Arm, JointMotion, FlangeMotion, FlangeTrajectory):
    def __init__(self, ip: str) -> None: ...
    def connect(self, ip: str, port: int) -> None: ...
    def disconnect(self) -> None: ...
    def read_joint(self) -> Vec: ...
    def read_joint_vel(self) -> Vec: ...
    def read_cartesian_euler(self) -> Vec: ...
    def read_cartesian_vel(self) -> Vec: ...
