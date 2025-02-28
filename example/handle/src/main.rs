use gilrs::{Button, Event, Gilrs};
use libhans::HansRobot;

fn main() {
    let mut gilrs = Gilrs::new().unwrap();
    let mut mode = Mode::Idle;
    let mut robot = HansRobot::new("192.168.10.2");
    let mut active_gamepad = None;

    // Iterate over all connected gamepads
    for (_id, gamepad) in gilrs.gamepads() {
        println!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    loop {
        // Examine new events
        while let Some(Event {
            id, event, time, ..
        }) = gilrs.next_event()
        {
            println!("{:?} New event from {}: {:?}", time, id, event);
            active_gamepad = Some(id);
        }

        // // You can also use cached gamepad state
        // if let Some(gamepad) = active_gamepad.map(|id| gilrs.gamepad(id)) {
        //     if gamepad.is_pressed(Button::South) {
        //         println!("Button South is pressed (XBox - A, PS - X)");
        //     }
        // }
    }
}

#[derive(Default)]
enum Mode {
    #[default]
    Idle,
    JointControl,
    CartesianControl,
}
