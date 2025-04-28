use libhans::HansRobot;
use robot_behavior::{ArmPreplannedMotionExt, RobotBehavior, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = HansRobot::new("192.168.20.10");

    let _ = robot.enable().unwrap();

    println!("{}", robot.version());

    robot.move_joint_rel(&[0.1, 0., 0., 0., 0., 0.]).unwrap();

    robot.move_path_from_file("./low_traj.csv", 0.1).unwrap();
    Ok(())
}
