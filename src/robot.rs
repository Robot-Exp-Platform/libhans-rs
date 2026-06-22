use std::{marker::PhantomData, thread::sleep, time::Duration};

use robot_behavior::{
    ArmState, Coord, JointSample, LoadState, OverrideOnce, Pose, Robot, RobotException,
    RobotResult, SpatialSample, StateView, driver::*,
};

use crate::{RobotMode, robot_impl::RobotImpl, robot_param::*, robot_state::RobotState, types::*};

pub trait HansType {
    const N: usize;
}

pub struct HansRobot<T: HansType, const N: usize> {
    pub(crate) marker: PhantomData<T>,
    pub robot_impl: RobotImpl<N>,
    pub(crate) is_moving: bool,

    pub(crate) coord: OverrideOnce<Coord>,
    pub(crate) max_vel: OverrideOnce<[f64; N]>,
    pub(crate) max_acc: OverrideOnce<[f64; N]>,
    pub(crate) max_cartesian_vel: OverrideOnce<f64>,
    pub(crate) max_cartesian_acc: OverrideOnce<f64>,
}

impl<T: HansType, const N: usize> HansRobot<T, N> {
    pub fn connect(&mut self, ip: &str, port: u16) {
        self.robot_impl.connect(ip, port);
    }

    /// 鏂紑缃戠粶杩炴帴
    pub fn disconnect(&mut self) {
        self.robot_impl.disconnect();
    }
}

impl<T: HansType, const N: usize> Robot for HansRobot<T, N> {
    type State = RobotState;
    const CONTROL_PERIOD: f64 = 1e-3;

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

    fn reset(&mut self) -> RobotResult<()> {
        self.robot_impl.robot_reset(0)?;
        Ok(())
    }

    fn is_moving(&mut self) -> RobotResult<bool> {
        if !self.is_moving {
            return Ok(false);
        }
        self.is_moving = self.robot_impl.state_read_cur_fsm(0).unwrap() != RobotMode::StandBy;
        Ok(self.is_moving)
    }

    fn waiting_for_finish(&mut self) -> RobotResult<()> {
        loop {
            let state = self.robot_impl.state_read_cur_fsm(0)?;
            if state == RobotMode::StandBy {
                break;
            }
        }

        self.is_moving = false;
        Ok(())
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

impl<T: HansType, const N: usize> Arm<N> for HansRobot<T, N>
where
    HansRobot<T, N>: Joints<N> + EndPoint + MoveTo<JointSpace<N>> + MoveTo<FlangeSpace>,
{
    fn state(&mut self) -> RobotResult<ArmState<N>> {
        let act_pose = self.robot_impl.state_read_act_pos(0)?;
        let joint_vel = self.robot_impl.state_read_act_joint_vel(0)?;
        let pose_vel = self.robot_impl.state_read_act_tcp_vel(0)?;

        let state = ArmState {
            joint: StateView::from_meas(JointSample {
                q: Some(act_pose.joint),
                dq: Some(joint_vel),
                ddq: None,
                tau: None,
                dtau: None,
            }),
            flange: StateView::from_meas(SpatialSample {
                pose: Some(Pose::Euler(
                    act_pose.pose_o_to_ee[0..3].try_into().unwrap(),
                    act_pose.pose_o_to_ee[3..6].try_into().unwrap(),
                )),
                vel: Some(pose_vel),
                acc: None,
                wrench: None,
            }),
            load: None,
            ..Default::default()
        };
        Ok(state)
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        self.robot_impl
            .state_set_payload((0, Load { mass: load.m, centroid: load.x }))
    }
    fn get_joint(&self) -> [f64; N] {
        [0.; N]
    }

    fn get_endpoint(&self) -> Pose {
        Pose::default()
    }

    fn with_joint_vel(mut self, vel_bound: [f64; N]) -> Self {
        self.max_vel.once(vel_bound);
        self
    }

    fn with_joint_acc(mut self, acc_bound: [f64; N]) -> Self {
        self.max_acc.once(acc_bound);
        self
    }

    fn with_joint_jerk(self, _jerk_bound: [f64; N]) -> Self {
        self
    }

    fn with_torque(self, _torque_bound: [f64; N]) -> Self {
        self
    }

    fn with_torque_dot(self, _torque_dot_bound: [f64; N]) -> Self {
        self
    }

    fn with_cartesian_vel(mut self, vel_bound: f64) -> Self {
        self.max_cartesian_vel.once(vel_bound);
        self
    }

    fn with_cartesian_acc(mut self, acc_bound: f64) -> Self {
        self.max_cartesian_acc.once(acc_bound);
        self
    }

    fn with_cartesian_jerk(self, _jerk_bound: f64) -> Self {
        self
    }

    fn with_rotation_vel(self, _vel_bound: f64) -> Self {
        self
    }

    fn with_rotation_acc(self, _acc_bound: f64) -> Self {
        self
    }

    fn with_rotation_jerk(self, _jerk_bound: f64) -> Self {
        self
    }
}

impl<T: HansType, const N: usize> HansRobot<T, N>
where
    HansRobot<T, N>: Joints<N> + EndPoint,
{
    pub fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        self.coord.set(coord);
        Ok(())
    }

    pub fn set_scale(&mut self, scale: f64) -> RobotResult<()> {
        self.max_vel.set(Self::JOINT_VEL_BOUND.map(|v| v * scale));
        self.max_acc.set(Self::JOINT_ACC_BOUND.map(|v| v * scale));
        self.max_cartesian_vel
            .set(Self::CARTESIAN_VEL_BOUND * scale);
        self.max_cartesian_acc
            .set(Self::CARTESIAN_ACC_BOUND * scale);

        self.robot_impl.state_set_override((0, scale))?;
        Ok(())
    }

    pub fn with_coord(&mut self, coord: Coord) -> &mut Self {
        self.coord.once(coord);
        self
    }

    pub fn with_scale(&mut self, scale: f64) -> &mut Self {
        self.max_vel.once(Self::JOINT_VEL_BOUND.map(|v| v * scale));
        self.max_acc.once(Self::JOINT_ACC_BOUND.map(|v| v * scale));
        self.max_cartesian_vel
            .once(Self::CARTESIAN_VEL_BOUND * scale);
        self.max_cartesian_acc
            .once(Self::CARTESIAN_ACC_BOUND * scale);
        self
    }
}

impl<T: HansType, const N: usize> MoveTo<JointSpace<N>> for HansRobot<T, N>
where
    HansRobot<T, N>: Joints<N>,
{
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        if self.is_moving()? {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;
        match self.coord.get() {
            Coord::OCS => {
                let move_config = WayPointEx {
                    joint: target,
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
            Coord::Inertial | Coord::Relative => {
                for (id, joint) in target.iter().enumerate().take(N) {
                    if *joint == 0. {
                        continue;
                    }
                    let dir = *joint > 0.;
                    let move_config = RelJ { id: id as u8, dir, dis: joint.abs() };
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
}

impl<T: HansType, const N: usize> MoveTo<FlangeSpace> for HansRobot<T, N>
where
    HansRobot<T, N>: EndPoint,
{
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        if self.is_moving()? {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;
        match self.coord.get() {
            Coord::OCS => {
                let move_config = WayPointEx {
                    pose: target.into(),
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
            Coord::Inertial | Coord::Relative => {
                let pose: [f64; 6] = target.into();
                for (id, pose) in pose.iter().enumerate().take(N) {
                    if *pose == 0. {
                        continue;
                    }
                    let dir = *pose >= 0.;
                    let move_config = RelL { id: id as u8, dir, dis: pose.abs(), coord: 0 };
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

impl<T: HansType, const N: usize> MoveTraj<JointSpace<N>> for HansRobot<T, N>
where
    HansRobot<T, N>: Joints<N>,
{
    fn move_traj(&mut self, path: Vec<[f64; N]>) -> RobotResult<()> {
        <Self as MoveTraj<JointSpace<N>>>::move_waypoints(self, path)
    }

    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<[f64; N]>,
    {
        Err(RobotException::UnprocessableInstructionError(
            "Hans driver does not support continuous-path planning; use move_waypoints".into(),
        ))
    }

    fn move_waypoints(&mut self, path: Vec<[f64; N]>) -> RobotResult<()> {
        if path.is_empty() {
            return Ok(());
        }
        if self.is_moving()? {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;

        <Self as MoveTo<JointSpace<N>>>::move_to(self, path[0])?;

        let path_name = "my_path";
        let path_config = StartPushMovePathJ {
            path_name: path_name.into(),
            speed: self.max_vel.get()[0] / Self::JOINT_VEL_BOUND[0],
            radius: 2.,
        };
        self.robot_impl.start_push_move_path_j((0, path_config))?;
        for joint in path {
            self.robot_impl
                .push_move_path_j((0, path_name.into(), joint))?;
        }
        self.robot_impl.end_push_move_path((0, path_name.into()))?;
        wait_move_path_ready(&mut self.robot_impl, path_name)?;
        self.robot_impl.move_path_j((0, path_name.into()))?;
        Ok(())
    }
}

impl<T: HansType, const N: usize> MoveTraj<FlangeSpace> for HansRobot<T, N>
where
    HansRobot<T, N>: EndPoint,
{
    fn move_traj(&mut self, path: Vec<Pose>) -> RobotResult<()> {
        <Self as MoveTraj<FlangeSpace>>::move_waypoints(self, path)
    }

    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<Pose>,
    {
        Err(RobotException::UnprocessableInstructionError(
            "Hans driver does not support continuous-path planning; use move_waypoints".into(),
        ))
    }

    fn move_waypoints(&mut self, path: Vec<Pose>) -> RobotResult<()> {
        if path.is_empty() {
            return Ok(());
        }
        if self.is_moving()? {
            return Err(RobotException::UnprocessableInstructionError(
                "Robot is moving, you can not push new move command".into(),
            ));
        }
        self.is_moving = true;

        <Self as MoveTo<FlangeSpace>>::move_to(self, path[0])?;

        let path_name = "my_path";
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
            let (tran, rot) = point.euler();
            let pose = [tran[0], tran[1], tran[2], rot[0], rot[1], rot[2]];
            self.robot_impl.push_move_path_l((0, pose))?;
        }
        self.robot_impl.end_push_move_path((0, path_name.into()))?;
        wait_move_path_ready(&mut self.robot_impl, path_name)?;
        self.robot_impl.move_path_l((0, path_name.into()))?;
        Ok(())
    }
}

fn wait_move_path_ready<const N: usize>(
    robot_impl: &mut RobotImpl<N>,
    path_name: &str,
) -> RobotResult<()> {
    loop {
        let state = robot_impl.read_move_path_state((0, path_name.into()))?;
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
    Ok(())
}
