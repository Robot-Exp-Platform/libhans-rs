use std::ffi::{CStr, CString, c_char};

use robot_behavior::{
    ArmBehavior, ControlType, MotionType, RobotBehavior,
    ffi::{CControlType, CMotionType},
};

use crate::{HANS_DOF, HansRobot};

#[repr(C)]
pub struct CHansRobot(*mut HansRobot);

#[unsafe(no_mangle)]
extern "C" fn hans_robot_new() -> *mut CHansRobot {
    Box::into_raw(Box::new(CHansRobot(Box::into_raw(Box::new(
        HansRobot::default(),
    )))))
}

#[unsafe(no_mangle)]
extern "C" fn hans_robot_free(robot: *mut CHansRobot) {
    unsafe {
        let _ = Box::from_raw(robot as *mut HansRobot);
    }
}

#[unsafe(no_mangle)]
extern "C" fn hans_connect(robot: *mut CHansRobot, ip: *const i8, port: u16) {
    let robot = unsafe { &mut *robot }.0;
    let ip = unsafe { CStr::from_ptr(ip).to_str().unwrap() };
    unsafe { robot.as_mut().unwrap() }.connect(ip, port);
}

#[unsafe(no_mangle)]
extern "C" fn hans_disconnect(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.disconnect();
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_joint(robot: *mut CHansRobot, joint: *const f64) {
    let robot = unsafe { &mut *robot }.0;
    let joint = unsafe { std::slice::from_raw_parts(joint, HANS_DOF) };
    let joint_array: [f64; HANS_DOF] = joint.try_into().expect("slice with incorrect length");
    unsafe { robot.as_mut().unwrap() }
        .move_joint(joint_array)
        .unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_joint_rel(robot: *mut CHansRobot, joint: *const f64) {
    let robot = unsafe { &mut *robot }.0;
    let joint = unsafe { std::slice::from_raw_parts(joint, HANS_DOF) };
    let joint_array: [f64; HANS_DOF] = joint.try_into().expect("slice with incorrect length");
    unsafe { robot.as_mut().unwrap() }
        .move_joint_rel(joint_array)
        .unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_linear_with_euler(robot: *mut CHansRobot, pose: *const f64) {
    let robot = unsafe { &mut *robot }.0;
    let pose = unsafe { std::slice::from_raw_parts(pose, 6) };
    let pose_array: [f64; 6] = pose.try_into().expect("slice with incorrect length");
    unsafe { robot.as_mut().unwrap() }
        .move_linear_with_euler(pose_array)
        .unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_linear_with_euler_rel(robot: *mut CHansRobot, pose: *const f64) {
    let robot = unsafe { &mut *robot }.0;
    let pose = unsafe { std::slice::from_raw_parts(pose, 6) };
    let pose_array: [f64; 6] = pose.try_into().expect("slice with incorrect length");
    unsafe { robot.as_mut().unwrap() }
        .move_linear_rel_with_euler(pose_array)
        .unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_version(robot: *mut CHansRobot) -> *const c_char {
    let robot = unsafe { &mut *robot }.0;
    let version_cstring = CString::new(unsafe { robot.as_mut().unwrap() }.version()).unwrap();
    version_cstring.into_raw()
}

#[unsafe(no_mangle)]
extern "C" fn hans_init(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.init().unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_shutdown(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.shutdown().unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_enable(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.enable().unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_disable(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.disable().unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_reset(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.reset().unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_stop(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.stop().unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_resume(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.resume().unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_emergency_stop(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.emergency_stop().unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_clear_emergency_stop(robot: *mut CHansRobot) {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }
        .clear_emergency_stop()
        .unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_is_moving(robot: *mut CHansRobot) -> bool {
    let robot = unsafe { &mut *robot }.0;
    unsafe { robot.as_mut().unwrap() }.is_moving()
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_to(robot: *mut CHansRobot, motion_type: CMotionType, data: *const f64) {
    let robot = unsafe { &mut *robot }.0;
    let data = unsafe { std::slice::from_raw_parts(data, 6) };
    let data_array: [f64; 6] = data.try_into().expect("slice with incorrect length");
    let motion: MotionType<HANS_DOF> = (motion_type, data_array).into();
    unsafe { robot.as_mut().unwrap() }.move_to(motion).unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_to_async(
    robot: *mut CHansRobot,
    motion_type: CMotionType,
    data: *const f64,
) {
    let robot = unsafe { &mut *robot }.0;
    let data = unsafe { std::slice::from_raw_parts(data, 6) };
    let data_array: [f64; 6] = data.try_into().expect("slice with incorrect length");
    let motion: MotionType<HANS_DOF> = (motion_type, data_array).into();
    unsafe { robot.as_mut().unwrap() }
        .move_to_async(motion)
        .unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_to_rel(robot: *mut CHansRobot, motion_type: CMotionType, data: *const f64) {
    let robot = unsafe { &mut *robot }.0;
    let data = unsafe { std::slice::from_raw_parts(data, 6) };
    let data_array: [f64; 6] = data.try_into().expect("slice with incorrect length");
    let motion: MotionType<HANS_DOF> = (motion_type, data_array).into();
    unsafe { robot.as_mut().unwrap() }.move_rel(motion).unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_to_rel_async(
    robot: *mut CHansRobot,
    motion_type: CMotionType,
    data: *const f64,
) {
    let robot = unsafe { &mut *robot }.0;
    let data = unsafe { std::slice::from_raw_parts(data, 6) };
    let data_array: [f64; 6] = data.try_into().expect("slice with incorrect length");
    let motion: MotionType<HANS_DOF> = (motion_type, data_array).into();
    unsafe { robot.as_mut().unwrap() }
        .move_rel_async(motion)
        .unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_move_path_from_file(robot: *mut CHansRobot, path: *const i8) {
    let robot = unsafe { &mut *robot }.0;
    let path = unsafe { CStr::from_ptr(path).to_str().unwrap() };
    unsafe { robot.as_mut().unwrap() }
        .move_path_from_file(path)
        .unwrap();
}

#[unsafe(no_mangle)]
extern "C" fn hans_control_with(
    robot: *mut CHansRobot,
    control_type: CControlType,
    data: *const f64,
) {
    let robot = unsafe { &mut *robot }.0;
    let data = unsafe { std::slice::from_raw_parts(data, HANS_DOF) };
    let data_array: [f64; HANS_DOF] = data.try_into().expect("slice with incorrect length");
    let control: ControlType<HANS_DOF> = (control_type, data_array).into();
    unsafe { robot.as_mut().unwrap() }
        .control_with(control)
        .unwrap();
}
