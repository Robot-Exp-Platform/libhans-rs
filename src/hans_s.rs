use robot_behavior::{ArmParam, Coord, OverrideOnce, behavior::*};
use std::f64::consts::FRAC_PI_2;

use crate::{HansRobot, robot::HansType, robot_impl::RobotImpl};

pub const HANS_S30_DOF: usize = 6;

pub type HansS30 = HansRobot<{ HansType::S30 }, 6>;

impl HansS30 {
    /// 新建一个机器人实例，使用传入的机器人 ip 与默认端口 [PORT_IF](crate::network::PORT_IF)
    pub fn new(ip: &str) -> Self {
        let robot_impl = RobotImpl::new(ip);
        let mut robot = HansRobot {
            robot_impl,
            is_moving: false,
            coord: OverrideOnce::new(Coord::OCS),
            max_vel: OverrideOnce::new(Self::JOINT_VEL_BOUND),
            max_acc: OverrideOnce::new(Self::JOINT_ACC_BOUND),
            max_cartesian_vel: OverrideOnce::new(Self::CARTESIAN_VEL_BOUND),
            max_cartesian_acc: OverrideOnce::new(Self::CARTESIAN_ACC_BOUND),
        };
        let _ = robot.set_speed(0.1);
        robot
    }
}

impl<const T: HansType, const N: usize> ArmParam<HANS_S30_DOF> for HansRobot<T, N> {
    const DH: [[f64; 4]; HANS_S30_DOF] = HANS_ROBOT_DH;
    const JOINT_MIN: [f64; HANS_S30_DOF] = [-360.; HANS_S30_DOF];
    const JOINT_MAX: [f64; HANS_S30_DOF] = [360.; HANS_S30_DOF];
    const JOINT_VEL_BOUND: [f64; HANS_S30_DOF] = [120., 120., 120., 180., 180., 180.];
    const JOINT_ACC_BOUND: [f64; HANS_S30_DOF] = [2.5; HANS_S30_DOF];
    const CARTESIAN_VEL_BOUND: f64 = 3.7;
    const CARTESIAN_ACC_BOUND: f64 = 2.0;
}

pub const HANS_ROBOT_MIN_JOINTS: [f64; HANS_S30_DOF] = [-360.; HANS_S30_DOF];
pub const HANS_ROBOT_MAX_JOINTS: [f64; HANS_S30_DOF] = [360.; HANS_S30_DOF];
pub const HANS_ROBOT_MAX_LOAD: f64 = 30.0;
pub const HANS_ROBOT_JOINT_VEL: [f64; HANS_S30_DOF] = [120., 120., 120., 180., 180., 180.];
pub const HANS_ROBOT_JOINT_ACC: [f64; HANS_S30_DOF] = [2.5; HANS_S30_DOF];
pub const HANS_ROBOT_MAX_CARTESIAN_VEL: f64 = 3.7;
pub const HANS_ROBOT_MAX_CARTESIAN_ACC: f64 = 2.0;
pub const HANS_ROBOT_DH: [[f64; 4]; HANS_S30_DOF] = [
    [0.0, 0.1857, 0.0, FRAC_PI_2],
    [0.0, 0.264, -0.85, 0.0],
    [0.0, 0.2065, -0.7915, 0.0],
    [0.0, 0.1585, 0.0, FRAC_PI_2],
    [0.0, 0.1585, 0.0, FRAC_PI_2],
    [0.0, 0.1345, 0.0, 0.0],
];
