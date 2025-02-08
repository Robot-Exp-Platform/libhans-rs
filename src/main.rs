use colored::Colorize;
use libhans_rs::{CommandSerde, CommandSubmit, HANS_ASCII, HANS_DOF, HansRobot, ROPLAT_ASCII};
use robot_behavior::{RobotBehavior, RobotException};
use std::collections::HashMap;
use std::io::{self, Write};

fn main() {
    let mut command_map = HashMap::new();

    for cmd in inventory::iter::<CommandSubmit> {
        command_map.insert(cmd.fn_name, cmd.dispatch);
    }

    let mut robot = HansRobot::new("127.0.0.1");

    println!("{}", ROPLAT_ASCII.blue());
    println!("<<<<<<Welcome to the robot control interface, we are roplat>>>>>>");
    println!("Version: {}", robot.version().yellow());
    println!("{}", HANS_ASCII.green());

    loop {
        print!("{} ", "robot>".blue());
        io::stdout().flush().unwrap();

        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();
        let input = input.trim();

        let args: Vec<&str> = input.split_whitespace().collect();
        if args.is_empty() {
            continue;
        }

        match args[0] {
            "connect" => {
                if args.len() != 3 {
                    println!("Usage: connect <IP> <PORT>");
                    continue;
                }
                let ip = args[1];
                let port: u16 = args[2].parse().unwrap();
                robot.connect(ip, port);
                println!("Connected to {}:{}", ip, port);
            }
            "disconnect" => {
                robot.disconnect();
                println!("Disconnected");
            }
            "robot_impl" => {
                let result = if args.len() == 2 {
                    command_map.get(args[1]).unwrap()(&mut robot.robot_impl, "")
                } else if args.len() == 3 {
                    command_map.get(args[1]).unwrap()(&mut robot.robot_impl, args[2])
                } else {
                    Err(RobotException::InvalidInstruction(
                        "Usage: robot_impl <COMMAND> <ARGS>".into(),
                    ))
                };

                println!("{} {:?}", "robot>robot_impl>".blue(), result);
            }
            "move_joints" => {
                if args.len() != 2 {
                    println!("Usage: move_joints <JOINTS>");
                    continue;
                }
                let joints: [f64; HANS_DOF] = CommandSerde::from_str(args[1]).unwrap();
                robot.move_joints(joints).unwrap();
                println!("Moved joints to {:?}", joints);
            }
            "version" => {
                println!("{}", robot.version());
            }
            "exit" => {
                break;
            }
            _ => {
                println!("Unknown command: {}", args[0]);
            }
        }
    }
}
