#[cfg(test)]
mod tests {
    use libhans::{CommandSubmit, HansRobot};

    #[test]
    fn test_all() {
        let mut robot = HansRobot::new("127.0.0.1");

        for cmd in inventory::iter::<CommandSubmit> {
            let command = cmd.dispatch;
            let result = command(&mut robot.robot_impl, "");
            println!("{:?} {:?}", cmd.fn_name, result);
        }
    }
}
