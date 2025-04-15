#[cfg(feature = "to_cxx")]
pub mod to_cxx {
    use crate::HansRobot;
    use crate::RobotResult;
    use robot_behavior::{ArmBehavior, RobotBehavior};

    #[cxx::bridge]
    mod ffi {
        extern "Rust" {
            type HansRobot;

            fn hans_new(ip: &str) -> Box<HansRobot>;
            fn connect(self: &mut HansRobot, ip: &str, port: u16);
            fn disconnect(self: &mut HansRobot);
            fn move_joint(self: &mut HansRobot, joint: [f64; 6]) -> Result<()>;
            fn move_joint_rel(self: &mut HansRobot, joint: [f64; 6]) -> Result<()>;
            fn move_linear_with_euler(self: &mut HansRobot, pose: [f64; 6]) -> Result<()>;
            fn move_linear_with_euler_rel(self: &mut HansRobot, pose: [f64; 6]) -> Result<()>;
            fn version(se: &HansRobot) -> String;
            fn init(se: &mut HansRobot) -> Result<()>;
            fn shutdown(se: &mut HansRobot) -> Result<()>;
            fn enable(se: &mut HansRobot) -> Result<()>;
            fn disable(se: &mut HansRobot) -> Result<()>;
            fn reset(se: &mut HansRobot) -> Result<()>;
            fn is_moving(se: &mut HansRobot) -> bool;
            fn stop(se: &mut HansRobot) -> Result<()>;
            fn resume(se: &mut HansRobot) -> Result<()>;
            fn emergency_stop(se: &mut HansRobot) -> Result<()>;
            fn clear_emergency_stop(se: &mut HansRobot) -> Result<()>;
            fn move_path_from_file(se: &mut HansRobot, path: &str) -> Result<()>;
        }
    }

    fn hans_new(ip: &str) -> Box<HansRobot> {
        Box::new(HansRobot::new(ip))
    }
    fn version(se: &HansRobot) -> String {
        se.version()
    }

    macro_rules! impl_behavior {
    (fn $name:ident) => {
        fn $name(se: &mut HansRobot) -> RobotResult<()> {
            se.$name()
        }
    };
    (fn $name:ident -> $ret:ty) => {
        fn $name(se: &mut HansRobot) -> $ret {
            se.$name()
        }
    };
    (fn $name:ident($($arg:ident: $ty:ty),*)) => {
        fn $name(se: &mut HansRobot, $($arg: $ty),*) -> RobotResult<()> {
            se.$name($($arg),*)
        }
    };
    (fn $name:ident($($arg:ident: $ty:ty),*) -> $ret:ty) => {
        fn $name(se: &mut HansRobot, $($arg: $ty),*) -> $ret {
            se.$name($($arg),*)
        }
    };
}

    impl_behavior!(fn init);
    impl_behavior!(fn shutdown);
    impl_behavior!(fn enable);
    impl_behavior!(fn disable);
    impl_behavior!(fn reset);
    impl_behavior!(fn is_moving -> bool);
    impl_behavior!(fn stop);
    impl_behavior!(fn resume);
    impl_behavior!(fn emergency_stop);
    impl_behavior!(fn clear_emergency_stop);
    impl_behavior!(fn move_path_from_file(path: &str));
}
