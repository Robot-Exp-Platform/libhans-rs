use std::{thread::sleep, time::Duration};

use libhans::HansRobot;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = HansRobot::new("192.168.20.10");

    let _ = robot.disable().unwrap();
    let _ = robot.enable().unwrap();

    println!("{}", robot.version());

    robot.set_speed(0.2)?;

    robot.move_joint(&[
        0.716136684,
        -1.926529132,
        2.0074477,
        -0.080928808,
        0.71614034,
        4.71239228,
    ])?;
    // robot.move_joint_rel(&[0., 0., 0.2, 0., 0., 0.]).unwrap();

    sleep(Duration::from_secs(10));

    // robot.move_path_from_file("./low_traj.csv", 0.1).unwrap();
    Ok(())
}
