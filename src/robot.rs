use std::{marker::PhantomData, thread::sleep, time::Duration};

use roplat_behavior::{
    ArmState, JointSample, LoadState, MotionLimits, Pose, Robot, RobotException, RobotResult,
    SpatialSample, StateView, driver::*,
};

use crate::{RobotMode, robot_impl::RobotImpl, robot_param::*, robot_state::RobotState, types::*};

pub trait HansType {
    const N: usize;
}

pub struct HansRobot<T: HansType, const N: usize> {
    pub(crate) marker: PhantomData<T>,
    pub robot_impl: RobotImpl<N>,
    pub(crate) is_moving: bool,
    /// Its `scale` is the controller's speed override; moves use the
    /// controller's own velocity and acceleration settings.
    pub(crate) limits: MotionLimits<N>,
}

impl<T: HansType, const N: usize> HansRobot<T, N> {
    pub fn connect(&mut self, ip: &str, port: u16) {
        self.robot_impl.connect(ip, port);
    }

    /// Disconnects from the robot controller.
    pub fn disconnect(&mut self) {
        self.robot_impl.disconnect();
    }
}

impl<T: HansType, const N: usize> Robot for HansRobot<T, N> {
    type State = RobotState;
    const CONTROL_PERIOD: Duration = Duration::from_millis(1);

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
        self.is_moving = self.robot_impl.state_read_cur_fsm(0)? != RobotMode::StandBy;
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
            joint: StateView::from_meas(
                JointSample::EMPTY.with_q(act_pose.joint).with_dq(joint_vel),
            ),
            flange: StateView::from_meas(
                SpatialSample::EMPTY
                    .with_pose(Pose::from_euler(
                        act_pose.pose_o_to_ee[0..3].try_into().unwrap(),
                        act_pose.pose_o_to_ee[3..6].try_into().unwrap(),
                    ))
                    .with_twist((pose_vel).into()),
            ),
            load: None,
            ..Default::default()
        };
        Ok(state)
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        self.robot_impl
            .state_set_payload((0, Load { mass: load.mass, centroid: load.center_of_mass }))
    }
    fn get_joint(&self) -> [f64; N] {
        [0.; N]
    }

    fn motion_limits(&self) -> MotionLimits<N> {
        self.limits
    }

    /// Checks `limits` against the declared envelope and sends their `scale`
    /// to the controller as its speed override.
    fn set_motion_limits(&mut self, limits: MotionLimits<N>) -> RobotResult<()> {
        limits.within(&MotionLimits::envelope::<Self>())?;
        self.robot_impl.state_set_override((0, limits.scale))?;
        self.limits = limits;
        Ok(())
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
            speed: self.limits.scaled_joint().dq_max[0] / Self::JOINT_VEL_BOUND[0],
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
