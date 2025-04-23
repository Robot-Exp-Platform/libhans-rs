use clap::Parser;
use colored::Colorize;
use libhans::{HANS_ASCII, HANS_DOF, HANS_VERSION, HansRobot, PORT_IF, ROPLAT_ASCII};
use robot_behavior::{
    ArmBehavior, ArmPreplannedMotion, ArmPreplannedMotionExt, LoadState, MotionType, Pose,
    RobotBehavior, RobotResult,
};
use serde::{Deserialize, Serialize};
use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
};

#[derive(Parser, Debug)]
#[clap(author, version)]
struct CommandLineArguments {
    /// IP-Address or hostname of the robot
    pub port: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
enum RobotCommand {
    Connect { ip: String },
    Enable,
    Disable,
    Reset,
    IsMoving,
    Stop,
    Pause,
    Resume,
    ArmState,
    SetLoad { m: f64, x: [f64; 3] },
    SetSpeed { speed: f64 },
    MoveJoint { joint: [f64; HANS_DOF], speed: f64 },
    MoveJointRel { joint: [f64; HANS_DOF], speed: f64 },
    MoveLinearWithEuler { pose: [f64; 6], speed: f64 },
    MoveLinearWithEulerRel { pose: [f64; 6], speed: f64 },
    MovePathFromFile { path: String, speed: f64 },
}

fn handle_client(mut stream: TcpStream) -> RobotResult<()> {
    let mut buffer = [0; 1024];
    let mut robot = HansRobot::default();
    loop {
        let bytes_read = stream.read(&mut buffer)?;
        if bytes_read == 0 {
            break;
        }

        let command: RobotCommand =
            if let Ok(command) = serde_json::from_slice(&buffer[..bytes_read]) {
                command
            } else {
                println!(
                    "Error parsing command: {}",
                    String::from_utf8_lossy(&buffer[..bytes_read])
                );
                continue;
            };

        let map = |()| "".to_string();

        let result = match command {
            RobotCommand::Connect { ip } => {
                robot.connect(&ip, PORT_IF);
                Ok(String::new())
            }
            RobotCommand::Enable => robot.enable().map(map),
            RobotCommand::Disable => robot.disable().map(map),
            RobotCommand::Reset => robot.reset().map(map),
            RobotCommand::IsMoving => Ok(robot.is_moving().to_string()),
            RobotCommand::Stop => robot.stop().map(map),
            RobotCommand::ArmState => robot.state().map(|s| s.to_string()),
            RobotCommand::Pause => robot.pause().map(map),
            RobotCommand::Resume => robot.resume().map(map),
            RobotCommand::SetLoad { m, x } => {
                robot.set_load(LoadState { m, x, i: [0.0; 9] }).map(map)
            }
            RobotCommand::SetSpeed { speed } => robot.set_speed(speed).map(map),
            RobotCommand::MoveJoint { joint, speed } => robot.move_joint(&&joint, speed).map(map),
            RobotCommand::MoveJointRel { joint, speed } => {
                robot.move_joint_rel(&joint, speed).map(map)
            }
            RobotCommand::MoveLinearWithEuler { pose, speed } => {
                let (tran, rot) = pose.split_at(3);
                robot
                    .move_linear_with_euler(
                        tran.try_into().unwrap(),
                        rot.try_into().unwrap(),
                        speed,
                    )
                    .map(map)
            }
            RobotCommand::MoveLinearWithEulerRel { pose, speed } => {
                let (tran, rot) = pose.split_at(3);
                robot
                    .move_rel(
                        MotionType::Cartesian(Pose::Euler(
                            tran.try_into().unwrap(),
                            rot.try_into().unwrap(),
                        )),
                        speed,
                    )
                    .map(map)
            }
            RobotCommand::MovePathFromFile { path, speed } => {
                robot.move_path_from_file(&path, speed).map(map)
            }
        }
        .map_err(|e| format!("[Error:{}]", e));

        stream.write_all(serde_json::to_string(&result).unwrap().as_bytes())?;
        stream.flush()?;
    }
    Ok(())
}

fn main() -> RobotResult<()> {
    let args = CommandLineArguments::parse();

    println!("{}", ROPLAT_ASCII.blue());
    println!("<<<<<<Welcome to the robot control interface, we are roplat>>>>>>");
    println!("Version: {}", HANS_VERSION.yellow());
    println!("{}", HANS_ASCII.green());

    let port = args.port.parse::<u16>().unwrap_or(PORT_IF);
    let tcp_listener = TcpListener::bind(format!("0.0.0.0:{}", port))?;
    println!("Listening on port {}", port);

    loop {
        let (stream, _) = tcp_listener.accept()?;
        println!("Client connected: {:?}", stream.peer_addr()?);
        std::thread::spawn(move || {
            if let Err(e) = handle_client(stream) {
                eprintln!("Error handling client: {}", e);
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use robot_behavior::ArmState;

    use super::*;

    #[test]
    fn test_robot_command_serde_json() {
        let command = RobotCommand::Connect {
            ip: "192.168.0.2".to_string(),
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "Connect", r#"{"Connect":{"ip":"192.168.0.2"}}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"Connect":{"ip":"192.168.0.2"}}"#
        );

        let command = RobotCommand::Enable;
        println!("指令 {} | 发送 {} | 返回 {}", "Enable", "\"Enable\"", "");
        assert_eq!(serde_json::to_string(&command).unwrap(), "\"Enable\"");

        let command = RobotCommand::Disable;
        println!("指令 {} | 发送 {} | 返回 {}", "Disable", "\"Disable\"", "");
        assert_eq!(serde_json::to_string(&command).unwrap(), "\"Disable\"");

        let command = RobotCommand::Reset;
        println!("指令 {} | 发送 {} | 返回 {}", "Reset", "\"Reset\"", "");
        assert_eq!(serde_json::to_string(&command).unwrap(), "\"Reset\"");

        let command = RobotCommand::IsMoving;
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "IsMoving", "\"IsMoving\"", "true"
        );
        assert_eq!(serde_json::to_string(&command).unwrap(), "\"IsMoving\"");

        let command = RobotCommand::Stop;
        println!("指令 {} | 发送 {} | 返回 {}", "Stop", "\"Stop\"", "");
        assert_eq!(serde_json::to_string(&command).unwrap(), "\"Stop\"");

        let command = RobotCommand::Pause;
        println!("指令 {} | 发送 {} | 返回 {}", "Pause", "\"Pause\"", "");
        assert_eq!(serde_json::to_string(&command).unwrap(), "\"Pause\"");

        let command = RobotCommand::Resume;
        println!("指令 {} | 发送 {} | 返回 {}", "Resume", "\"Resume\"", "");
        assert_eq!(serde_json::to_string(&command).unwrap(), "\"Resume\"");

        let command = RobotCommand::ArmState;
        println!(
            "指令 {} | 发送 {} | 返回 {:?}",
            "ArmState",
            "\"ArmState\"",
            ArmState::<HANS_DOF>::default()
        );
        assert_eq!(serde_json::to_string(&command).unwrap(), "\"ArmState\"");

        let command = RobotCommand::SetLoad {
            m: 1.0,
            x: [0.0, 0.0, 0.0],
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "SetLoad", r#"{"SetLoad":{"m":1.0,"x":[0.0,0.0,0.0]}}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"SetLoad":{"m":1.0,"x":[0.0,0.0,0.0]}}"#
        );

        let command = RobotCommand::SetSpeed { speed: 1.0 };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "SetSpeed", r#"{"SetSpeed":{"speed":1.0}}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"SetSpeed":{"speed":1.0}}"#
        );

        let command = RobotCommand::MoveJoint {
            joint: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            speed: 1.0,
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MoveJoint", r#"{"MoveJoint":{"joint":[0.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MoveJoint":{"joint":[0.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#
        );

        let command = RobotCommand::MoveJointRel {
            joint: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            speed: 1.0,
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MoveJointRel", r#"{"MoveJointRel":{"joint":[1.0,0.0,0.0,0.0,0.0,0.0]}}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MoveJointRel":{"joint":[1.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#
        );

        let command = RobotCommand::MoveLinearWithEuler {
            pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            speed: 1.0,
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MoveLinearWithEuler",
            r#"{"MoveLinearWithEuler":{"pose":[0.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#,
            ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MoveLinearWithEuler":{"pose":[0.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#
        );

        let command = RobotCommand::MoveLinearWithEulerRel {
            pose: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            speed: 1.0,
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MoveLinearWithEulerRel",
            r#"{"MoveLinearWithEulerRel":{"pose":[1.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#,
            ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MoveLinearWithEulerRel":{"pose":[1.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#
        );

        let command = RobotCommand::MovePathFromFile {
            path: String::from("path/to/file"),
            speed: 1.0,
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MovePathFromFile", r#"{"MovePathFromFile":{"path":"path/to/file","speed":1.0}}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MovePathFromFile":{"path":"path/to/file","speed":1.0}}"#
        );

        let command = RobotCommand::MovePathFromFile {
            path: "low_traj.csv".into(),
            speed: 1.0,
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MovePathFromFile", r#"{"MovePathFromFile":{"path":"low_traj.csv","speed":1.0}}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MovePathFromFile":{"path":"low_traj.csv","speed":1.0}}"#
        );
    }

    #[test]
    fn test_move_path() {
        let motion = [MotionType::Joint([0.; 6]); 10];
        let motion_json = serde_json::to_string(&motion).unwrap();
        let file_path = "traj_joint.json";
        let mut file = std::fs::File::create(file_path).unwrap();
        file.write_all(motion_json.as_bytes()).unwrap();
        file.flush().unwrap();

        let motion = [MotionType::<6>::Cartesian(Pose::Euler([0.; 3], [0.; 3])); 10];
        let motion_json = serde_json::to_string(&motion).unwrap();
        let file_path = "traj_cartesian.json";
        let mut file = std::fs::File::create(file_path).unwrap();
        file.write_all(motion_json.as_bytes()).unwrap();
        file.flush().unwrap();
    }
}
