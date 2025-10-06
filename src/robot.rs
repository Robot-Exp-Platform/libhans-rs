use std::{marker::ConstParamTy, thread::sleep, time::Duration};

use robot_behavior::{
    ArmState, Coord, LoadState, MotionType, OverrideOnce, Pose, RobotBehavior, RobotException,
    RobotResult, behavior::*,
};
use serde::{Deserialize, Serialize};

use crate::{RobotMode, robot_impl::RobotImpl, robot_param::*, robot_state::RobotState, types::*};

#[derive(ConstParamTy, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub enum HansType {
    S30,
}
pub struct HansRobot<const T: HansType, const N: usize> {
    pub robot_impl: RobotImpl<N>,
    pub(crate) is_moving: bool,

    pub(crate) coord: OverrideOnce<Coord>,
    pub(crate) max_vel: OverrideOnce<[f64; N]>,
    pub(crate) max_acc: OverrideOnce<[f64; N]>,
    pub(crate) max_cartesian_vel: OverrideOnce<f64>,
    pub(crate) max_cartesian_acc: OverrideOnce<f64>,
}

impl<const T: HansType, const N: usize> HansRobot<T, N> {
    /// 连接网络，使用指定的 ip 与端口
    pub fn connect(&mut self, ip: &str, port: u16) {
        self.robot_impl.connect(ip, port);
    }

    /// 断开网络连接
    pub fn disconnect(&mut self) {
        self.robot_impl.disconnect();
    }
}

impl<const T: HansType, const N: usize> RobotBehavior for HansRobot<T, N> {
    type State = RobotState;
    fn version() -> String {
        format!("HansRobot v{HANS_VERSION}")
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

    fn read_state(&mut self) -> RobotResult<Self::State> {
        unimplemented!()
    }
}

impl<const T: HansType, const N: usize> ArmBehavior<N> for HansRobot<T, N>
where
    HansRobot<T, N>: ArmParam<N>,
{
    fn state(&mut self) -> RobotResult<ArmState<N>> {
        let act_pose = self.robot_impl.state_read_act_pos(0)?;
        let joint_vel = self.robot_impl.state_read_act_joint_vel(0)?;
        let pose_vel = self.robot_impl.state_read_act_tcp_vel(0)?;

        let state = ArmState::<N> {
            joint: Some(act_pose.joint),
            joint_vel: Some(joint_vel),
            joint_acc: None,
            tau: None,
            pose_o_to_ee: Some(Pose::Euler(
                act_pose.pose_o_to_ee[0..3].try_into().unwrap(),
                act_pose.pose_o_to_ee[3..6].try_into().unwrap(),
            )),
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
    fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        self.coord.set(coord);
        Ok(())
    }

    fn set_speed(&mut self, speed: f64) -> RobotResult<()> {
        self.max_vel.set(Self::JOINT_VEL_BOUND.map(|v| v * speed));
        self.max_acc.set(Self::JOINT_ACC_BOUND.map(|v| v * speed));
        self.max_cartesian_vel
            .set(Self::CARTESIAN_VEL_BOUND * speed);
        self.max_cartesian_acc
            .set(Self::CARTESIAN_ACC_BOUND * speed);

        self.robot_impl.state_set_override((0, speed))?;
        Ok(())
    }

    fn with_coord(&mut self, coord: Coord) -> &mut Self {
        self.coord.once(coord);
        self
    }
    fn with_speed(&mut self, speed: f64) -> &mut Self {
        self.max_vel.once(Self::JOINT_VEL_BOUND.map(|v| v * speed));
        self.max_acc.once(Self::JOINT_ACC_BOUND.map(|v| v * speed));
        self.max_cartesian_vel
            .once(Self::CARTESIAN_VEL_BOUND * speed);
        self.max_cartesian_acc
            .once(Self::CARTESIAN_ACC_BOUND * speed);
        self
    }
    fn with_velocity(&mut self, joint_vel: &[f64; N]) -> &mut Self {
        self.max_vel.once(*joint_vel);
        self
    }
    fn with_acceleration(&mut self, joint_acc: &[f64; N]) -> &mut Self {
        self.max_acc.once(*joint_acc);
        self
    }
    fn with_jerk(&mut self, _joint_jerk: &[f64; N]) -> &mut Self {
        self
    }
    fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut Self {
        self.max_cartesian_vel.once(cartesian_vel);
        self
    }
    fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut Self {
        self.max_cartesian_acc.once(cartesian_acc);
        self
    }
    fn with_cartesian_jerk(&mut self, _cartesian_jerk: f64) -> &mut Self {
        self
    }
    fn with_rotation_velocity(&mut self, _rotation_vel: f64) -> &mut Self {
        self
    }
    fn with_rotation_acceleration(&mut self, _rotation_acc: f64) -> &mut Self {
        self
    }
    fn with_rotation_jerk(&mut self, _rotation_jerk: f64) -> &mut Self {
        self
    }
}

impl<const T: HansType, const N: usize> ArmPreplannedMotionImpl<N> for HansRobot<T, N>
where
    HansRobot<T, N>: ArmParam<N>,
{
    fn move_joint(&mut self, target: &[f64; N]) -> RobotResult<()> {
        self.move_joint_async(target)?;
        loop {
            let state = self.robot_impl.state_read_cur_fsm(0)?;
            if state == RobotMode::StandBy {
                break;
            }
        }

        self.is_moving = false;
        Ok(())
    }

    fn move_joint_async(&mut self, target: &[f64; N]) -> RobotResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;
        match self.coord.get() {
            Coord::OCS => {
                let move_config = WayPointEx {
                    joint: *target,
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
            Coord::Interial | Coord::Shot => {
                for (id, joint) in target.iter().enumerate().take(N) {
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
                    return Ok(());
                }
            }
            Coord::Other(_) => {
                println!("undefined coord, use OCS as default");
            }
        }
        Ok(())
    }

    fn move_cartesian(&mut self, target: &Pose) -> RobotResult<()> {
        self.move_cartesian_async(target)?;
        loop {
            let state = self.robot_impl.state_read_cur_fsm(0)?;
            if state == RobotMode::StandBy {
                break;
            }
        }

        self.is_moving = false;
        Ok(())
    }

    fn move_cartesian_async(&mut self, target: &Pose) -> RobotResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;
        match self.coord.get() {
            Coord::OCS => {
                let move_config = WayPointEx {
                    pose: (*target).into(),
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
            Coord::Interial | Coord::Shot => {
                let pose: [f64; 6] = (*target).into();
                for (id, pose) in pose.iter().enumerate().take(N) {
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
                    return Ok(());
                }
            }
            Coord::Other(_) => {
                println!("undefined coord, use OCS as default");
            }
        }
        Ok(())
    }
}

impl<const T: HansType, const N: usize> ArmPreplannedMotion<N> for HansRobot<T, N>
where
    HansRobot<T, N>: ArmParam<N>,
{
    fn move_path(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        self.move_path_async(path)?;
        loop {
            let state = self.robot_impl.state_read_cur_fsm(0)?;
            if state == RobotMode::StandBy {
                break;
            }
        }

        self.is_moving = false;
        Ok(())
    }

    fn move_path_async(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
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
                    speed: self.max_vel.get()[0] / Self::JOINT_VEL_BOUND[0],
                    radius: 2.,
                };
                self.robot_impl.start_push_move_path_j((0, path_config))?;
                for point in path {
                    if let MotionType::Joint(joint) = point {
                        self.robot_impl
                            .push_move_path_j((0, path_name.into(), joint))?;
                    }
                }
                self.robot_impl.end_push_move_path((0, path_name.into()))?;
                loop {
                    let state = self
                        .robot_impl
                        .read_move_path_state((0, path_name.into()))?;
                    match state {
                        3 => break,
                        5 => {
                            return Err(RobotException::UnprocessableInstructionError(
                                "Connot calculate path, Check whether the points are appropriate"
                                    .into(),
                            ));
                        }
                        _ => sleep(Duration::from_millis(100)),
                    }
                }
                self.robot_impl.move_path_j((0, path_name.into()))?;
            }
            MotionType::Cartesian(_) => {
                let path_config = StartPushMovePathL {
                    path_name: path_name.into(),
                    vel: 100.,
                    acc: 2500.,
                    jeck: 1_000_000.,
                    ucs_name: "Base".into(),
                    tcp_name: "Tcp".into(),
                };
                self.robot_impl.start_push_move_path_l((0, path_config))?;
                for point in path {
                    if let MotionType::Cartesian(Pose::Euler(tran, rot)) = point {
                        let pose = [tran[0], tran[1], tran[2], rot[0], rot[1], rot[2]];
                        self.robot_impl.push_move_path_l((0, pose))?;
                    }
                }
                self.robot_impl.end_push_move_path((0, path_name.into()))?;
                loop {
                    let state = self
                        .robot_impl
                        .read_move_path_state((0, path_name.into()))?;
                    match state {
                        3 => break,
                        5 => {
                            return Err(RobotException::UnprocessableInstructionError(
                                "Connot calculate path, Check whether the points are appropriate"
                                    .into(),
                            ));
                        }
                        _ => sleep(Duration::from_millis(100)),
                    }
                }
                self.robot_impl.move_path_l((0, path_name.into()))?;
            }
            _ => {
                return Err(RobotException::UnprocessableInstructionError(
                    "Unsupported motion type".into(),
                ));
            }
        }
        Ok(())
    }

    fn move_path_prepare(&mut self, path: Vec<MotionType<N>>) -> robot_behavior::RobotResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }

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
            MotionType::Cartesian(_) => {
                let path_config = StartPushMovePathL {
                    path_name: path_name.into(),
                    vel: 100.,
                    acc: 2500.,
                    jeck: 1_000_000.,
                    ucs_name: "Base".into(),
                    tcp_name: "Tcp".into(),
                };
                self.robot_impl.start_push_move_path_l((0, path_config))?;
                for point in path {
                    if let MotionType::Cartesian(Pose::Euler(tran, rot)) = point {
                        let pose = [tran[0], tran[1], tran[2], rot[0], rot[1], rot[2]];
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
        self.robot_impl.end_push_move_path((0, path_name.into()))?;
        Ok(())
    }

    fn move_path_start(&mut self, start: MotionType<N>) -> RobotResult<()> {
        if self.is_moving() {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        let path_name = "hans_path";
        loop {
            let state = self
                .robot_impl
                .read_move_path_state((0, path_name.into()))?;
            match state {
                3 => break,
                5 => {
                    return Err(RobotException::UnprocessableInstructionError(
                        "Connot calculate path, Check whether the points are appropriate".into(),
                    ));
                }
                _ => sleep(Duration::from_millis(100)),
            }
        }
        self.is_moving = true;

        self.move_to(start)?;
        match start {
            MotionType::Joint(_) => {
                self.robot_impl.move_path_j((0, path_name.to_string()))?;
            }
            MotionType::Cartesian(_) => {
                self.robot_impl.move_path_l((0, path_name.to_string()))?;
            }
            _ => {
                return Err(RobotException::UnprocessableInstructionError(
                    "Unsupported motion type".into(),
                ));
            }
        }

        loop {
            let state = self.robot_impl.state_read_cur_fsm(0)?;
            if state == RobotMode::StandBy {
                break;
            }
        }
        self.is_moving = false;
        Ok(())
    }
}
