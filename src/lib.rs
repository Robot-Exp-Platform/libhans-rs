#![feature(adt_const_params)]

mod exception;
mod network;
mod robot;
mod robot_error;
mod robot_impl;
mod robot_mode;
mod robot_param;
mod robot_state;
mod types;

pub use exception::HansResult;
pub use network::*;
pub use robot::HansRobot;
pub use robot_error::RobotError;
pub use robot_impl::{CommandSubmit, DispatchFn};
pub use robot_mode::RobotMode;
pub use robot_param::*;
pub use types::CommandSerde;
