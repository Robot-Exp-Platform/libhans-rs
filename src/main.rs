use clap::Parser;
use libhans::{HANS_DOF, HansRobot, PORT_IF};
use robot_behavior::{
    ArmBehavior, ArmPreplannedMotion, ArmPreplannedMotionExt, LoadState, MotionType, RobotBehavior,
    RobotException, RobotResult,
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
    MoveJointRel { joint: [f64; HANS_DOF] },
    MoveLinearWithEuler { pose: [f64; 6], speed: f64 },
    MoveLinearWithEulerRel { pose: [f64; 6] },
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

        let result = match command {
            RobotCommand::Connect { ip } => {
                robot.connect(&ip, PORT_IF);
                Ok(())
            }
            RobotCommand::Enable => robot.enable(),
            RobotCommand::Disable => robot.disable(),
            RobotCommand::Reset => robot.reset(),
            RobotCommand::IsMoving => {
                let is_moving = robot.is_moving();
                stream
                    .write_all(format!("{:?}", is_moving).as_bytes())
                    .map_err(|e| RobotException::NetworkError(e.to_string()))
            }
            RobotCommand::Stop => robot.stop(),
            RobotCommand::ArmState => {
                let state = robot.state();
                if let Ok(state) = state {
                    stream
                        .write_all(format!("{:?}", state).as_bytes())
                        .map_err(|e| RobotException::NetworkError(e.to_string()))
                } else {
                    state.map(|_| ())
                }
            }
            RobotCommand::Pause => robot.pause(),
            RobotCommand::Resume => robot.resume(),
            RobotCommand::SetLoad { m, x } => robot.set_load(LoadState { m, x, i: [0.0; 9] }),
            RobotCommand::SetSpeed { speed } => robot.set_speed(speed),
            RobotCommand::MoveJoint { joint, speed } => robot.move_joint(&&joint, speed),
            RobotCommand::MoveJointRel { joint } => robot.move_joint_rel(&joint),
            RobotCommand::MoveLinearWithEuler { pose, speed } => {
                robot.move_linear_with_euler(&pose, speed)
            }
            RobotCommand::MoveLinearWithEulerRel { pose } => {
                robot.move_rel(MotionType::CartesianEuler(pose))
            }
        };

        if let Err(e) = result {
            stream
                .write_all(format!("Error: {}", e).as_bytes())
                .map_err(|e| RobotException::NetworkError(e.to_string()))?;
            println!("Error handling command: {}", e);
            continue;
        }
    }
    Ok(())
}

fn main() -> RobotResult<()> {
    let args = CommandLineArguments::parse();

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
            "SetLoad", r#"{"m":1.0,"x":[0.0,0.0,0.0]}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"SetLoad":{"m":1.0,"x":[0.0,0.0,0.0]}}"#
        );

        let command = RobotCommand::SetSpeed { speed: 1.0 };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "SetSpeed", r#"{"speed":1.0}"#, ""
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
            "MoveJoint", r#"{"joint":[0.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MoveJoint":{"joint":[0.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#
        );

        let command = RobotCommand::MoveJointRel {
            joint: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MoveJointRel", r#"{"joint":[1.0,0.0,0.0,0.0,0.0,0.0]}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MoveJointRel":{"joint":[1.0,0.0,0.0,0.0,0.0,0.0]}}"#
        );

        let command = RobotCommand::MoveLinearWithEuler {
            pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            speed: 1.0,
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MoveLinearWithEuler", r#"{"pose":[0.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MoveLinearWithEuler":{"pose":[0.0,0.0,0.0,0.0,0.0,0.0],"speed":1.0}}"#
        );

        let command = RobotCommand::MoveLinearWithEulerRel {
            pose: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        };
        println!(
            "指令 {} | 发送 {} | 返回 {}",
            "MoveLinearWithEulerRel", r#"{"pose":[1.0,0.0,0.0,0.0,0.0,0.0]}"#, ""
        );
        assert_eq!(
            serde_json::to_string(&command).unwrap(),
            r#"{"MoveLinearWithEulerRel":{"pose":[1.0,0.0,0.0,0.0,0.0,0.0]}}"#
        );
    }
}
