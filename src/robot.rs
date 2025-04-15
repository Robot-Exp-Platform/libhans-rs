use robot_behavior::{
    ArmBehavior, ArmPreplannedMotion, ArmPreplannedMotionExt, ArmState, ControlType, LoadState,
    MotionType, Pose, RobotBehavior, RobotException, RobotResult,
};

use crate::{
    HANS_DOF, HANS_VERSION, RobotMode,
    robot_impl::RobotImpl,
    robot_state::RobotState,
    types::{Load, RelJ, RelL, StartPushMovePathJ, StartPushMovePathL, WayPointEx},
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

    pub fn set_speed(&mut self, speed: f64) -> RobotResult<()> {
        self.robot_impl.state_set_override((0, speed))?;
        Ok(())
    }
}

impl RobotBehavior for HansRobot {
    fn version(&self) -> String {
        format!("HansRobot v{}", HANS_VERSION)
    }

    fn init(&mut self) -> RobotResult<()> {
        if self.robot_impl.is_connected() {
            self.robot_impl.robot_power_on(())?;
            Ok(())
        } else {
            Err(RobotException::NetworkError(
                "Robot is not connected".to_string(),
            ))
        }
    }

    fn shutdown(&mut self) -> RobotResult<()> {
        self.disable()?;
        Ok(())
    }

    fn enable(&mut self) -> RobotResult<()> {
        self.robot_impl.robot_enable(0)?;
        Ok(())
    }

    fn disable(&mut self) -> RobotResult<()> {
        self.robot_impl.robot_disable(0)?;
        Ok(())
    }

    fn reset(&mut self) -> RobotResult<()> {
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

    fn stop(&mut self) -> RobotResult<()> {
        self.robot_impl.robot_move_stop(0)?;
        Ok(())
    }

    fn pause(&mut self) -> RobotResult<()> {
        self.robot_impl.robot_move_pause(0)?;
        Ok(())
    }

    fn resume(&mut self) -> RobotResult<()> {
        self.robot_impl.robot_move_continue(0)?;
        Ok(())
    }

    fn emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!("hans robot does not support emergency stop")
    }

    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!("hans robot does not support clear emergency stop")
    }
}

impl ArmBehavior<HANS_DOF> for HansRobot {
    type State = RobotState;
    fn read_state(&mut self) -> RobotResult<Self::State> {
        unimplemented!()
    }
    fn state(&mut self) -> RobotResult<ArmState<HANS_DOF>> {
        let joint = self.robot_impl.state_read_act_pos(0)?.joint;
        let pose = Pose::Euler(self.robot_impl.state_read_act_pos(0)?.pose_o_to_ee);
        let joint_vel = self.robot_impl.state_read_act_joint_vel(0)?;
        let pose_vel = self.robot_impl.state_read_act_tcp_vel(0)?;

        let state = ArmState {
            joint: Some(joint),
            joint_vel: Some(joint_vel),
            joint_acc: None,
            tau: None,
            pose_o_to_ee: Some(pose),
            pose_f_to_ee: None,
            pose_ee_to_k: None,
            cartesian_vel: Some(pose_vel),
            load: None,
        };
        Ok(state)
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        self.robot_impl.state_set_payload((
            0,
            Load {
                mass: load.m,
                centroid: load.x,
            },
        ))
    }
}

impl ArmPreplannedMotion<HANS_DOF> for HansRobot {
    fn move_to(&mut self, target: MotionType<HANS_DOF>, speed: f64) -> RobotResult<()> {
        self.move_to_async(target, speed)?;
        loop {
            let state = self.robot_impl.state_read_cur_fsm(0)?;
            if state == RobotMode::StandBy {
                break;
            }
        }

        self.is_moving = false;
        Ok(())
    }

    fn move_to_async(&mut self, target: MotionType<HANS_DOF>, speed: f64) -> RobotResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;

        self.set_speed(speed)?;

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

    fn move_rel(&mut self, rel: MotionType<HANS_DOF>) -> RobotResult<()> {
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

    fn move_rel_async(&mut self, rel: MotionType<HANS_DOF>) -> RobotResult<()> {
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
                    break;
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

    fn move_path(&mut self, path: Vec<MotionType<HANS_DOF>>, speed: f64) -> RobotResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;

        let first_point = path[0];
        self.move_to(first_point, speed)?;

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
    fn control_with(&mut self, _: ControlType<HANS_DOF>) -> RobotResult<()> {
        unimplemented!("control_with is not implemented")
    }
}

impl ArmPreplannedMotionExt<HANS_DOF> for HansRobot {
    fn move_path_prepare(
        &mut self,
        path: Vec<MotionType<HANS_DOF>>,
    ) -> robot_behavior::RobotResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;

        let path_name = "hans_path";

        match path[0] {
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
            }
            _ => {
                return Err(RobotException::UnprocessableInstructionError(
                    "Unsupported motion type".into(),
                ));
            }
        }

        Ok(())
    }

    fn move_path_start(&mut self) -> robot_behavior::RobotResult<()> {
        self.robot_impl.move_path_j((0, "hans_path".to_string()))?;
        Ok(())
    }
}
