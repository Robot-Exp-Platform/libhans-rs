use robot_behavior::{ArmParam, Coord, OverrideOnce, behavior::*};
use std::{f64::consts::FRAC_PI_2, marker::PhantomData};

use crate::{HansRobot, robot::HansType, robot_impl::RobotImpl};

pub struct _HansS30;
impl HansType for _HansS30 {
    const N: usize = 6;
}

pub type HansS30 = HansRobot<_HansS30, { _HansS30::N }>;

impl HansS30 {
    /// 新建一个机器人实例，使用传入的机器人 ip 与默认端口 [`PORT_IF`](crate::network::PORT_IF)
    pub fn new(ip: &str) -> Self {
        let robot_impl = RobotImpl::new(ip);
        let mut robot = HansRobot {
            marker: PhantomData,
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

impl ArmParam<{ _HansS30::N }> for HansRobot<_HansS30, { _HansS30::N }> {
    const JOINT_MIN: [f64; _HansS30::N] = [-360.; _HansS30::N];
    const JOINT_MAX: [f64; _HansS30::N] = [360.; _HansS30::N];
    const JOINT_VEL_BOUND: [f64; _HansS30::N] = [120., 120., 120., 180., 180., 180.];
    const JOINT_ACC_BOUND: [f64; _HansS30::N] = [2.5; _HansS30::N];
    const CARTESIAN_VEL_BOUND: f64 = 3.7;
    const CARTESIAN_ACC_BOUND: f64 = 2.0;
}

pub const HANS_ROBOT_MIN_JOINTS: [f64; _HansS30::N] = [-360.; _HansS30::N];
pub const HANS_ROBOT_MAX_JOINTS: [f64; _HansS30::N] = [360.; _HansS30::N];
pub const HANS_ROBOT_MAX_LOAD: f64 = 30.0;
pub const HANS_ROBOT_JOINT_VEL: [f64; _HansS30::N] = [120., 120., 120., 180., 180., 180.];
pub const HANS_ROBOT_JOINT_ACC: [f64; _HansS30::N] = [2.5; _HansS30::N];
pub const HANS_ROBOT_MAX_CARTESIAN_VEL: f64 = 3.7;
pub const HANS_ROBOT_MAX_CARTESIAN_ACC: f64 = 2.0;
pub const HANS_ROBOT_DH: [[f64; 4]; _HansS30::N] = [
    [0.0, 0.1857, 0.0, FRAC_PI_2],
    [0.0, 0.264, -0.85, 0.0],
    [0.0, 0.2065, -0.7915, 0.0],
    [0.0, 0.1585, 0.0, FRAC_PI_2],
    [0.0, 0.1585, 0.0, FRAC_PI_2],
    [0.0, 0.1345, 0.0, 0.0],
];
