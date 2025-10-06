use robot_behavior::{RobotException, RobotResult};

use super::command::{Command, CommandRequest, CommandResponse};
use super::command_serde::CommandSerde;
use crate::robot_error::RobotError;

pub type MoveRelJRequest = CommandRequest<{ Command::MoveRelJ }, (u8, RelJ)>;
pub type MoveRelLRequest = CommandRequest<{ Command::MoveRelL }, (u8, RelL)>;
pub type WayPointRelRequest<const N: usize> =
    CommandRequest<{ Command::WayPointRel }, (u8, WayPointRel<N>)>;
pub type WayPointExRequest<const N: usize> =
    CommandRequest<{ Command::WayPointEx }, (u8, WayPointEx<N>)>;
pub type WayPointRequest<const N: usize> = CommandRequest<{ Command::WayPoint }, (u8, WayPoint<N>)>;
pub type WayPoint2Request<const N: usize> =
    CommandRequest<{ Command::WayPoint2 }, (u8, WayPoint2<N>)>;
pub type MoveJRequest<const N: usize> = CommandRequest<{ Command::MoveJ }, (u8, MoveJ<N>)>;
pub type MoveLRequest<const N: usize> = CommandRequest<{ Command::MoveL }, (u8, MoveL<N>)>;
pub type MoveCRequest = CommandRequest<{ Command::MoveC }, (u8, MoveC)>;
pub type StartPushMovePathRequest =
    CommandRequest<{ Command::StartPushMovePath }, (u8, StartPushMovePathJ)>;
pub type PushMovePathJRequest<const N: usize> =
    CommandRequest<{ Command::PushMovePathJ }, (u8, String, [f64; N])>;
pub type EndPushMovePathRequest = CommandRequest<{ Command::EndPushMovePath }, (u8, String)>;
pub type MovePathRequest = CommandRequest<{ Command::MovePath }, (u8, String)>;
pub type ReadMovePathStateRequest = CommandRequest<{ Command::ReadMovePathState }, (u8, String)>;
pub type UpdateMovePathNameRequest =
    CommandRequest<{ Command::UpdateMovePathName }, (u8, String, String)>;
pub type DelMovePathRequest = CommandRequest<{ Command::DelMovePath }, (u8, String)>;
pub type ReadSoftMotionProcessRequest = CommandRequest<{ Command::ReadSoftMotionProcess }, u8>;
pub type InitMovePathLRequest =
    CommandRequest<{ Command::InitMovePathL }, (u8, StartPushMovePathL)>;
pub type PushMovePathLRequest = CommandRequest<{ Command::PushMovePathL }, (u8, [f64; 6])>;
pub type PushMovePathsRequest<const N: usize> =
    CommandRequest<{ Command::PushMovePaths }, (u8, MovePaths<N>)>;
pub type MovePathLRequest = CommandRequest<{ Command::MovePathL }, (u8, String)>;
pub type SetMovePathOverrideRequest = CommandRequest<{ Command::SetMovePathOverride }, (u8, f64)>;
pub type StartServoRequest = CommandRequest<{ Command::StartServo }, (u8, f64, f64)>;
pub type PushServoJRequest<const N: usize> =
    CommandRequest<{ Command::PushServoJ }, (u8, [f64; N])>;
pub type PushServoPRequest = CommandRequest<{ Command::PushServoP }, (u8, [[f64; 6]; 3])>;

pub type MoveRelJResponse = CommandResponse<{ Command::MoveRelJ }, ()>;
pub type MoveRelLResponse = CommandResponse<{ Command::MoveRelL }, ()>;
pub type WayPointRelResponse = CommandResponse<{ Command::WayPointRel }, ()>;
pub type WayPointExResponse = CommandResponse<{ Command::WayPointEx }, ()>;
pub type WayPointResponse = CommandResponse<{ Command::WayPoint }, ()>;
pub type WayPoint2Response = CommandResponse<{ Command::WayPoint2 }, ()>;
pub type MoveJResponse = CommandResponse<{ Command::MoveJ }, ()>;
pub type MoveLResponse = CommandResponse<{ Command::MoveL }, ()>;
pub type MoveCResponse = CommandResponse<{ Command::MoveC }, ()>;
pub type StartPushMovePathResponse = CommandResponse<{ Command::StartPushMovePath }, ()>;
pub type PushMovePathJResponse = CommandResponse<{ Command::PushMovePathJ }, ()>;
pub type EndPushMovePathResponse = CommandResponse<{ Command::EndPushMovePath }, ()>;
pub type MovePathResponse = CommandResponse<{ Command::MovePath }, ()>;
pub type ReadMovePathStateResponse = CommandResponse<{ Command::ReadMovePathState }, u8>;
pub type UpdateMovePathNameResponse = CommandResponse<{ Command::UpdateMovePathName }, ()>;
pub type DelMovePathResponse = CommandResponse<{ Command::DelMovePath }, ()>;
pub type ReadSoftMotionProcessResponse =
    CommandResponse<{ Command::ReadSoftMotionProcess }, (f64, u16)>;
pub type InitMovePathLResponse = CommandResponse<{ Command::InitMovePathL }, ()>;
pub type PushMovePathLResponse = CommandResponse<{ Command::PushMovePathL }, ()>;
pub type PushMovePathsResponse = CommandResponse<{ Command::PushMovePaths }, ()>;
pub type MovePathLResponse = CommandResponse<{ Command::MovePathL }, ()>;
pub type SetMovePathOverrideResponse = CommandResponse<{ Command::SetMovePathOverride }, ()>;
pub type StartServoResponse = CommandResponse<{ Command::StartServo }, ()>;
pub type PushServoJResponse = CommandResponse<{ Command::PushServoJ }, ()>;
pub type PushServoPResponse = CommandResponse<{ Command::PushServoP }, ()>;

#[derive(Default, libhans_derive::CommandSerde)]
pub struct RelJ {
    pub id: u8,
    pub dir: bool,
    pub dis: f64,
}

#[derive(Default, libhans_derive::CommandSerde)]
pub struct RelL {
    pub id: u8,
    pub dir: bool,
    pub dis: f64,
    pub coord: u8,
}

// TODO: 路点指令有两种指令形式，部分情况下部分内容不作为输入参数

#[derive(libhans_derive::CommandSerde)]
pub struct WayPointRel<const N: usize> {
    pub move_mode: u8,
    pub use_point_list: bool,
    pub pose: [f64; 6],
    pub joint: [f64; N],
    pub rel_move_mode: u8,
    pub is_move: [bool; N],
    pub dis: [f64; N],
    pub tcp_name: String,
    pub ucs_name: String,
    pub vel: f64,
    pub acc: f64,
    pub radius: f64,
    pub use_joint: bool,
    pub is_seek_di: bool,
    pub di_id: u8,
    pub di_value: bool,
    pub command_id: String,
}

#[derive(libhans_derive::CommandSerde)]
pub struct WayPointEx<const N: usize> {
    pub pose: [f64; 6],
    pub joint: [f64; N],
    pub ucs: [f64; 6],
    pub tcp: [f64; 6],
    pub vel: f64,
    pub acc: f64,
    pub radius: f64,
    pub move_mode: u8,
    pub use_joint: bool,
    pub is_seek_di: bool,
    pub di_id: u8,
    pub di_value: bool,
    pub command_id: String,
}

impl<const N: usize> Default for WayPointEx<N> {
    fn default() -> Self {
        WayPointEx {
            pose: [0.0; 6],
            joint: [0.0; N],
            ucs: [0.0; 6],
            tcp: [0.0; 6],
            vel: 0.1,
            acc: 0.1,
            radius: 0.0,
            move_mode: 1,
            use_joint: false,
            is_seek_di: false,
            di_id: 0,
            di_value: false,
            command_id: String::new(),
        }
    }
}

#[derive(libhans_derive::CommandSerde)]
pub struct WayPoint<const N: usize> {
    pub pose: [f64; 6],
    pub joint: [f64; N],
    pub tcp_name: String,
    pub ucs_name: String,
    pub vel: f64,
    pub acc: f64,
    pub radius: f64,
    pub move_mode: u8,
    pub use_joint: bool,
    pub is_seek_di: bool,
    pub di_id: u8,
    pub di_value: bool,
    pub command_id: String,
}

#[derive(libhans_derive::CommandSerde)]
pub struct WayPoint2<const N: usize> {
    pub pose1: [f64; 6],
    pub joint: [f64; N],
    pub tcp_name: String,
    pub ucs_name: String,
    pub vel: f64,
    pub acc: f64,
    pub radius: f64,
    pub move_mode: u8,
    pub use_joint: bool,
    pub is_seek_di: bool,
    pub di_id: u8,
    pub di_value: bool,
    pub pose2: [f64; 6],
    pub command_id: String,
}

#[derive(libhans_derive::CommandSerde)]
pub struct MoveJ<const N: usize> {
    pub pose: [f64; 6],
    pub joint: [f64; N],
    pub ucs_name: String,
    pub tcp_name: String,
    pub vel: f64,
    pub acc: f64,
    pub radius: f64,
    pub use_joint: bool,
    pub is_seek_di: bool,
    pub di_id: u8,
    pub di_value: bool,
    pub command_id: String,
}

#[derive(libhans_derive::CommandSerde)]
pub struct MoveL<const N: usize> {
    pub pose: [f64; 6],
    pub joint: [f64; N],
    pub ucs_name: String,
    pub tcp_name: String,
    pub vel: f64,
    pub acc: f64,
    pub radius: f64,
    pub use_joint: bool,
    pub is_seek_di: bool,
    pub di_id: u8,
    pub di_value: bool,
    pub command_id: String,
}

#[derive(Default, libhans_derive::CommandSerde)]
pub struct MoveC {
    pub pose_start: [f64; 6],
    pub pose_pass: [f64; 6],
    pub pose_end: [f64; 6],
    pub is_fixed_pose: bool,
    pub move_mode: u8,
    pub rad_len: f64,
    pub vel: f64,
    pub acc: f64,
    pub radius: f64,
    pub tcp_name: String,
    pub ucs_name: String,
    pub command_id: String,
}

#[derive(Default, libhans_derive::CommandSerde)]
pub struct StartPushMovePathJ {
    pub path_name: String,
    pub speed: f64,
    pub radius: f64,
}

#[derive(Default, libhans_derive::CommandSerde)]
pub struct StartPushMovePathL {
    pub path_name: String,
    pub vel: f64,
    pub acc: f64,
    pub jeck: f64,
    pub ucs_name: String,
    pub tcp_name: String,
}

pub struct MovePaths<const N: usize> {
    pub path_name: String,
    pub move_mode: u8,
    pub points: [[f64; 6]; N],
}

impl<const N: usize> CommandSerde for MovePaths<N> {
    fn to_string(&self) -> String {
        [
            CommandSerde::to_string(&self.path_name),
            CommandSerde::to_string(&self.move_mode),
            N.to_string(),
            self.points.to_string(),
        ]
        .join(",")
    }

    fn from_str(data: &str) -> RobotResult<Self> {
        let mut iter = data.split(',');
        let path_name = CommandSerde::from_str(iter.next().unwrap())?;
        let move_mode = CommandSerde::from_str(iter.next().unwrap())?;
        let _: u16 = CommandSerde::from_str(iter.next().unwrap())?;
        let mut points = [[0.0; 6]; N];
        for point in points.iter_mut().take(N) {
            *point = CommandSerde::from_str(iter.next().unwrap())?;
        }
        Ok(MovePaths {
            path_name,
            move_mode,
            points,
        })
    }

    fn try_default() -> Self {
        MovePaths {
            path_name: CommandSerde::try_default(),
            move_mode: CommandSerde::try_default(),
            points: CommandSerde::try_default(),
        }
    }
}
