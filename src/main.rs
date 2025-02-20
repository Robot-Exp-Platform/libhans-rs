use clap::{ArgGroup, Args, Parser};
use colored::Colorize;
use crossterm::event::{self, Event, KeyCode, poll};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use libhans::{
    CommandSerde, CommandSubmit, DispatchFn, HANS_ASCII, HANS_DOF, HansRobot, PORT_IF, ROPLAT_ASCII,
};
use robot_behavior::{RobotBehavior, RobotException};
use std::collections::HashMap;
use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

enum CliState {
    Root,
    RobotImpl,
    KeyBoard,
}

fn main() {
    let mut cli_state = CliState::Root;

    let mut command_map = HashMap::new();
    for cmd in inventory::iter::<CommandSubmit> {
        command_map.insert(cmd.fn_name, cmd.dispatch);
    }

    let mut robot = HansRobot::default();

    println!("{}", ROPLAT_ASCII.blue());
    println!("<<<<<<Welcome to the robot control interface, we are roplat>>>>>>");
    println!("Version: {}", robot.version().yellow());
    println!("{}", HANS_ASCII.green());

    loop {
        match cli_state {
            CliState::Root => print!("{} ", "robot>".blue()),
            CliState::RobotImpl => print!("{} ", "robot>robot_impl>".blue()),
            CliState::KeyBoard => print!("{} ", "robot>keyboard>".blue()),
        }
        io::stdout().flush().unwrap();

        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();

        let result = match cli_state {
            CliState::Root => cli_root(&mut cli_state, &mut robot, &input),
            CliState::RobotImpl => cli_robot_impl(&mut cli_state, &mut robot, &input, &command_map),
            CliState::KeyBoard => cli_key_board(&mut cli_state, &mut robot, &input),
        };

        if let Err(e) = result {
            println!("Error: {:?}", e);
        }
    }
}

fn cli_root(
    cli_state: &mut CliState,
    robot: &mut HansRobot,
    input: &str,
) -> Result<(), RobotException> {
    let root_command = RootCommand::try_parse_from(input.split_whitespace());
    if let Err(e) = root_command {
        println!("Error: {}", e);
        return Ok(());
    }
    match root_command.unwrap() {
        RootCommand::Connect(args) => {
            robot.connect(&args.ip, args.port);
            println!("Connected to {}:{}", args.ip, args.port);
        }
        RootCommand::Disconnect => {
            robot.disconnect();
            println!("Disconnected");
        }
        RootCommand::Enable => {
            let _ = robot.enable();
            println!("Enabled");
        }
        RootCommand::Disable => {
            let _ = robot.disable();
            println!("Disabled");
        }
        RootCommand::RobotImpl => {
            *cli_state = CliState::RobotImpl;
        }
        RootCommand::Move(args) => match (args.relative, args.joints, args.linear) {
            (true, Some(joints), None) => {
                robot.move_joint_rel(joints)?;
                println!("Moved joints for {:?}", joints);
            }
            (true, None, Some(linear)) => {
                robot.move_linear_with_euler_rel(linear)?;
                println!("Moved linear for {:?}", linear);
            }
            (false, Some(joints), None) => {
                robot.move_joint(joints)?;
                println!("Moved joints to {:?}", joints);
            }
            (false, None, Some(linear)) => {
                robot.move_linear_with_euler(linear)?;
                println!("Moved linear to {:?}", linear);
            }
            _ => unimplemented!("Move command is not implemented"),
        },
        RootCommand::MoveFromTcp => {
            let mut tcp = TcpStream::connect("192.168.10.7:25532").unwrap();
            let mut buffer = [0; 48]; // 6 f64 values, each 8 bytes
            tcp.read_exact(&mut buffer).unwrap();
            let linear: [f64; 6] = unsafe { std::ptr::read(buffer.as_ptr() as *const [f64; 6]) };
            robot.move_linear_with_euler(linear)?;
            tcp.write_all(b"ok").unwrap();
            println!("Moved linear to {:?}", linear);
        }
        RootCommand::KeyBoardControl => {
            *cli_state = CliState::KeyBoard;
        }
        RootCommand::Version => {
            println!("{}", robot.version());
        }
        RootCommand::Exit => {
            std::process::exit(0);
        }
    }

    Ok(())
}

fn cli_robot_impl(
    cli_state: &mut CliState,
    robot: &mut HansRobot,
    input: &str,
    command_map: &HashMap<&str, DispatchFn>,
) -> Result<(), RobotException> {
    let robot_impl_command = RobotImplCommand::try_parse_from(input.split_whitespace());
    if let Err(e) = robot_impl_command {
        return Err(RobotException::UnWarpError(e.to_string()));
    }
    let robot_impl_command = robot_impl_command.unwrap();
    if robot_impl_command.command == "exit" {
        *cli_state = CliState::Root;
        return Ok(());
    }
    let func = command_map.get(&robot_impl_command.command[..]);
    if func.is_none() {
        println!("Unknown command: {}", robot_impl_command.command);
        return Ok(());
    }

    let result = if let Some(args) = &robot_impl_command.args {
        func.unwrap()(&mut robot.robot_impl, args)
    } else {
        func.unwrap()(&mut robot.robot_impl, "")
    };

    println!("{} {:?}", "robot>robot_impl>".blue(), result);
    Ok(())
}

fn cli_key_board(
    cli_state: &mut CliState,
    robot: &mut HansRobot,
    input: &str,
) -> Result<(), RobotException> {
    let input_value = f64::from_str(input)?;
    robot.robot_impl.state_set_override((0, input_value))?;
    enable_raw_mode().unwrap();
    let mut stdout = io::stdout();
    println!("Press ESC to exit.");
    loop {
        let mut joints = [0.; 6];
        let mut linear = [0.; 6];
        if poll(Duration::from_millis(100)).unwrap() {
            let mut key = event::read().unwrap();
            while poll(Duration::from_millis(0)).unwrap() {
                if let Ok(e) = event::read() {
                    key = e;
                } else {
                    break;
                }
            }
            if let Event::Key(key_event) = key {
                match key_event.code {
                    KeyCode::Char('1') => joints[0] = 1.0,
                    KeyCode::Char('2') => joints[1] = 1.0,
                    KeyCode::Char('3') => joints[2] = 1.0,
                    KeyCode::Char('4') => joints[3] = 1.0,
                    KeyCode::Char('5') => joints[4] = 1.0,
                    KeyCode::Char('6') => joints[5] = 1.0,
                    KeyCode::Char('q') => joints[0] = -1.0,
                    KeyCode::Char('w') => joints[1] = -1.0,
                    KeyCode::Char('e') => joints[2] = -1.0,
                    KeyCode::Char('r') => joints[3] = -1.0,
                    KeyCode::Char('t') => joints[4] = -1.0,
                    KeyCode::Char('y') => joints[5] = -1.0,
                    KeyCode::Char('a') => linear[0] = 1.0,
                    KeyCode::Char('s') => linear[1] = 1.0,
                    KeyCode::Char('d') => linear[2] = 1.0,
                    KeyCode::Char('f') => linear[3] = 1.0,
                    KeyCode::Char('g') => linear[4] = 1.0,
                    KeyCode::Char('h') => linear[5] = 1.0,
                    KeyCode::Char('z') => linear[0] = -1.0,
                    KeyCode::Char('x') => linear[1] = -1.0,
                    KeyCode::Char('c') => linear[2] = -1.0,
                    KeyCode::Char('v') => linear[3] = -1.0,
                    KeyCode::Char('b') => linear[4] = -1.0,
                    KeyCode::Char('n') => linear[5] = -1.0,
                    KeyCode::Esc => {
                        *cli_state = CliState::Root;
                        break;
                    }
                    _ => {}
                }
            }
        }
        if joints.iter().sum::<f64>() == 0. && linear.iter().sum::<f64>() == 0. {
            continue;
        }
        println!("joints: {:?}, linear: {:?}", joints, linear);
        robot.move_joint_rel(joints)?;
        robot.move_linear_with_euler_rel(linear)?;
        println!("Move over");
        stdout.flush().unwrap();
    }
    disable_raw_mode().unwrap();
    Ok(())
}

#[derive(Debug, Parser, Clone)]
#[command(no_binary_name = true)]
enum RootCommand {
    #[command(arg_required_else_help = true)]
    Connect(ConnectArgs),
    Disconnect,
    Enable,
    Disable,
    RobotImpl,
    #[command(arg_required_else_help = true)]
    #[command(group(
        ArgGroup::new("move_type")
            .required(true)
            .args(&["joints", "linear"]),
    ))]
    Move(MoveArgs),
    MoveFromTcp,
    KeyBoardControl,
    Version,
    Exit,
}

#[derive(Debug, Parser, Clone)]
struct ConnectArgs {
    #[arg(short, long, default_value = "192.168.10.2")]
    ip: String,

    #[arg(short, long, default_value_t = PORT_IF)]
    port: u16,
}

#[derive(Debug, Args, Clone)]
struct MoveArgs {
    #[arg(short, long)]
    relative: bool,
    #[arg(
        short,
        long,
        value_name = "JOINTS",
        value_parser = <[f64; HANS_DOF]>::from_str,
        conflicts_with = "linear"
    )]
    joints: Option<[f64; HANS_DOF]>,
    #[arg(
        short,
        long,
        value_name = "COORDINATES",
        value_parser =  <[f64; 6]>::from_str,
        conflicts_with = "joints"
    )]
    linear: Option<[f64; 6]>,
}

#[derive(Debug, Parser)]
#[command(no_binary_name = true)]
struct RobotImplCommand {
    command: String,
    args: Option<String>,
}
