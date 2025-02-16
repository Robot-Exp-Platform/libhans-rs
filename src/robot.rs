use nalgebra as na;
use robot_behavior::{ArmBehavior, ControlType, MotionType, RobotBehavior, RobotException};

use crate::{
    HANS_DOF, HANS_VERSION, HansResult, RobotMode,
    robot_impl::RobotImpl,
    types::{RelJ, RelL, StartPushMovePathJ, StartPushMovePathL, WayPointEx},
};

#[derive(Default)]
pub struct HansRobot {
    pub robot_impl: RobotImpl,
    is_moving: bool,
}

impl HansRobot {
    /// 新建一个机器人实例，使用传入的机器人 ip 与默认端口 [PORT_IF](crate::network::PORT_IF)
    pub fn new(ip: &str) -> Self {
        let robot_impl = RobotImpl::new(ip);
        HansRobot {
            robot_impl,
            ..HansRobot::default()
        }
    }

    /// 连接网络，使用指定的 ip 与端口
    pub fn connect(&mut self, ip: &str, port: u16) {
        self.robot_impl.connect(ip, port);
    }

    /// 断开网络连接
    pub fn disconnect(&mut self) {
        self.robot_impl.disconnect();
    }

    pub fn move_joint(&mut self, joints: [f64; HANS_DOF]) -> HansResult<()> {
        self.move_to(MotionType::Joint(joints))
    }

    pub fn move_joint_rel(&mut self, joints: [f64; HANS_DOF]) -> HansResult<()> {
        self.move_rel(MotionType::Joint(joints))
    }

    pub fn move_linear_with_quaternion(
        &mut self,
        position: [f64; 3],
        quaternion: [f64; 4],
    ) -> HansResult<()> {
        let rotation = na::UnitQuaternion::from_quaternion(na::Quaternion::new(
            quaternion[3],
            quaternion[0],
            quaternion[1],
            quaternion[2],
        ));
        let pose = na::Isometry3::from_parts(position.into(), rotation);
        self.move_to(MotionType::CartesianQuat(pose))
    }

    pub fn move_linear_with_homogeneous(&mut self, _: [f64; 16]) -> HansResult<()> {
        unimplemented!("move_linear_with_homogeneous is not implemented")
    }

    pub fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> HansResult<()> {
        self.move_to(MotionType::CartesianEuler(pose))
    }

    pub fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> HansResult<()> {
        self.move_rel(MotionType::CartesianEuler(pose))
    }
}

impl RobotBehavior for HansRobot {
    fn version(&self) -> String {
        format!("HansRobot v{}", HANS_VERSION)
    }

    fn init(&mut self) -> HansResult<()> {
        if self.robot_impl.is_connected() {
            self.robot_impl.robot_power_on(())?;
            Ok(())
        } else {
            Err(RobotException::NetworkError(
                "Robot is not connected".to_string(),
            ))
        }
    }

    fn shutdown(&mut self) -> HansResult<()> {
        self.disable()?;
        Ok(())
    }

    fn enable(&mut self) -> HansResult<()> {
        self.robot_impl.robot_enable(0)?;
        Ok(())
    }

    fn disable(&mut self) -> HansResult<()> {
        self.robot_impl.robot_disable(0)?;
        Ok(())
    }

    fn reset(&mut self) -> HansResult<()> {
        self.robot_impl.robot_reset(0)?;
        Ok(())
    }

    fn is_moving(&mut self) -> bool {
        if !self.is_moving {
            return false;
        }
        self.is_moving = self.robot_impl.state_read_cur_fsm(0).unwrap() != RobotMode::StandBy;
        self.is_moving
    }

    fn stop(&mut self) -> HansResult<()> {
        self.robot_impl.robot_move_stop(0)?;
        Ok(())
    }

    fn resume(&mut self) -> HansResult<()> {
        self.robot_impl.robot_move_continue(0)?;
        Ok(())
    }

    fn emergency_stop(&mut self) -> HansResult<()> {
        unimplemented!("hans robot does not support emergency stop")
    }

    fn clear_emergency_stop(&mut self) -> HansResult<()> {
        unimplemented!("hans robot does not support clear emergency stop")
    }
}

impl ArmBehavior<HANS_DOF> for HansRobot {
    fn move_to(&mut self, target: MotionType<HANS_DOF>) -> HansResult<()> {
        self.move_to_async(target)?;
        loop {
            let state = self.robot_impl.state_read_cur_fsm(0)?;
            if state == RobotMode::StandBy {
                break;
            }
        }

        self.is_moving = false;
        Ok(())
    }

    fn move_to_async(&mut self, target: MotionType<HANS_DOF>) -> HansResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;

        match target {
            MotionType::Joint(joint) => {
                let move_config = WayPointEx {
                    joint,
                    vel: 25.,
                    acc: 100.,
                    radius: 5.,
                    move_mode: 0,
                    use_joint: true,
                    command_id: "0".into(),
                    ..WayPointEx::default()
                };
                self.robot_impl.move_way_point_ex((0, move_config))?;
            }
            MotionType::CartesianEuler(pose) => {
                let move_config = WayPointEx {
                    pose,
                    vel: 25.,
                    acc: 100.,
                    radius: 5.,
                    move_mode: 1,
                    use_joint: false,
                    command_id: "0".into(),
                    ..WayPointEx::default()
                };
                self.robot_impl.move_way_point_ex((0, move_config))?;
            }
            _ => {
                return Err(RobotException::UnprocessableInstructionError(
                    "Unsupported motion type".into(),
                ));
            }
        }
        Ok(())
    }

    fn move_rel(&mut self, rel: MotionType<HANS_DOF>) -> HansResult<()> {
        self.move_rel_async(rel)?;
        loop {
            let state = self.robot_impl.state_read_cur_fsm(0)?;
            if state == RobotMode::StandBy {
                break;
            }
        }

        self.is_moving = false;
        Ok(())
    }

    fn move_rel_async(&mut self, rel: MotionType<HANS_DOF>) -> HansResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;

        match rel {
            MotionType::Joint(joint) => {
                for (id, joint) in joint.iter().enumerate().take(HANS_DOF) {
                    if *joint == 0. {
                        continue;
                    }
                    let dir = *joint > 0.;
                    let move_config = RelJ {
                        id: id as u8,
                        dir,
                        dis: joint.abs(),
                    };
                    self.robot_impl.move_joint_rel((0, move_config))?;
                }
            }
            MotionType::CartesianEuler(pose) => {
                for (id, pose) in pose.iter().enumerate().take(HANS_DOF) {
                    if *pose == 0. {
                        continue;
                    }
                    let dir = *pose >= 0.;
                    let move_config = RelL {
                        id: id as u8,
                        dir,
                        dis: pose.abs(),
                        coord: 0,
                    };
                    self.robot_impl.move_line_rel((0, move_config))?;
                }
            }
            _ => {
                return Err(RobotException::UnprocessableInstructionError(
                    "Unsupported motion type".into(),
                ));
            }
        }
        Ok(())
    }

    fn move_path(&mut self, path: Vec<MotionType<HANS_DOF>>) -> HansResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;

        let first_point = path[0];
        self.move_to(first_point)?;

        let path_name = "my_path";

        match first_point {
            MotionType::Joint(_) => {
                let path_config = StartPushMovePathJ {
                    path_name: path_name.into(),
                    speed: 25.,
                    radius: 5.,
                };
                self.robot_impl.start_push_move_path_j((0, path_config))?;
                for point in path {
                    if let MotionType::Joint(joint) = point {
                        self.robot_impl
                            .push_move_path_j((0, path_name.into(), joint))?;
                    }
                }
                self.robot_impl.end_push_move_path((0, path_name.into()))?;
                self.robot_impl.move_path_j((0, path_name.into()))?;
            }
            MotionType::CartesianEuler(_) => {
                let path_config = StartPushMovePathL {
                    path_name: path_name.into(),
                    vel: 25.,
                    acc: 100.,
                    radius: 5.,
                    ucs_name: "Base".into(),
                    tcp_name: "Tcp".into(),
                };
                self.robot_impl.start_push_move_path_l((0, path_config))?;
                for point in path {
                    if let MotionType::CartesianEuler(pose) = point {
                        self.robot_impl.push_move_path_l((0, pose))?;
                    }
                }
                self.robot_impl.end_push_move_path((0, path_name.into()))?;
                self.robot_impl.move_path_l((0, path_name.into()))?;
            }
            _ => {
                return Err(RobotException::UnprocessableInstructionError(
                    "Unsupported motion type".into(),
                ));
            }
        }

        self.is_moving = false;
        Ok(())
    }

    fn control_with(&mut self, _: ControlType<HANS_DOF>) -> HansResult<()> {
        unimplemented!("control_with is not implemented")
    }
}
