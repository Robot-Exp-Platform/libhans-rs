use pyo3::{
    Bound, PyResult,
    exceptions::PyException,
    pyclass, pymethods, pymodule,
    types::{PyModule, PyModuleMethods},
};
use robot_behavior::{
    ArmBehavior, ArmPreplannedMotion, ArmPreplannedMotionExt, MotionType, Pose, RobotBehavior,
    RobotException,
};

use crate::{HANS_DOF, HansRobot};

/// # HansRobot
/// new(ip: str) -> PyHansRobot
#[pyclass(name = "HansRobot")]
pub struct PyHansRobot(HansRobot);

#[pymethods]
impl PyHansRobot {
    /// 初始化一个机器人对象，此时的机器人对象未连接到机器人
    #[new]
    fn new() -> Self {
        PyHansRobot(HansRobot::default())
    }

    fn __repr__(&self) -> String {
        "HansRobot".to_string()
    }

    fn connect(&mut self, ip: &str, port: u16) {
        self.0.connect(ip, port)
    }

    fn disconnect(&mut self) {
        self.0.disconnect()
    }

    fn move_joint(&mut self, joint: [f64; HANS_DOF], speed: f64) -> PyResult<()> {
        self.0.move_joint(&joint, speed).map_err(map_err)
    }

    #[pyo3(text_signature = "(joint)")]
    fn move_joint_async(&mut self, joint: [f64; HANS_DOF], speed: f64) -> PyResult<()> {
        self.0.move_joint_async(&joint, speed).map_err(map_err)
    }

    fn move_joint_rel(&mut self, joint: [f64; HANS_DOF], speed: f64) -> PyResult<()> {
        self.0.move_joint_rel(&joint, speed).map_err(map_err)
    }

    fn move_joint_rel_async(&mut self, joint: [f64; HANS_DOF], speed: f64) -> PyResult<()> {
        self.0.move_joint_rel_async(&joint, speed).map_err(map_err)
    }

    fn move_cartesian(&mut self, pose: Pose, speed: f64) -> PyResult<()> {
        self.0.move_cartesian(&pose, speed).map_err(map_err)
    }

    fn move_cartesian_async(&mut self, pose: Pose, speed: f64) -> PyResult<()> {
        self.0.move_cartesian_async(&pose, speed).map_err(map_err)
    }

    fn move_cartesian_rel(&mut self, pose: Pose, speed: f64) -> PyResult<()> {
        self.0.move_cartesian_rel(&pose, speed).map_err(map_err)
    }

    fn move_cartesian_rel_async(&mut self, pose: Pose, speed: f64) -> PyResult<()> {
        self.0
            .move_cartesian_rel_async(&pose, speed)
            .map_err(map_err)
    }

    fn move_linear_with_euler(
        &mut self,
        tran: [f64; 3],
        rot: [f64; 3],
        speed: f64,
    ) -> PyResult<()> {
        self.0
            .move_linear_with_euler(tran, rot, speed)
            .map_err(map_err)
    }

    fn move_linear_with_euler_async(
        &mut self,
        tran: [f64; 3],
        rot: [f64; 3],
        speed: f64,
    ) -> PyResult<()> {
        self.0
            .move_linear_with_euler_async(tran, rot, speed)
            .map_err(map_err)
    }

    fn move_joint_path(&mut self, joints: Vec<[f64; HANS_DOF]>, speed: f64) -> PyResult<()> {
        let joints = joints.into_iter().map(MotionType::Joint).collect();
        self.0.move_path(joints, speed).map_err(map_err)
    }

    fn move_cartesian_path(&mut self, poses: Vec<Pose>, speed: f64) -> PyResult<()> {
        let poses = poses.into_iter().map(MotionType::Cartesian).collect();
        self.0.move_path(poses, speed).map_err(map_err)
    }

    fn move_linear_path_with_euler(&mut self, poses: Vec<[f64; 6]>, speed: f64) -> PyResult<()> {
        let poses = poses
            .into_iter()
            .map(|p| {
                MotionType::Cartesian(Pose::Euler(
                    p[0..3].try_into().unwrap(),
                    p[3..6].try_into().unwrap(),
                ))
            })
            .collect();
        self.0.move_path(poses, speed).map_err(map_err)
    }

    fn move_path_from_file(&mut self, path: &str, speed: f64) -> PyResult<()> {
        self.0.move_path_from_file(path, speed).map_err(map_err)
    }

    /// 获取机器人的关节角度
    /// return:
    ///   List[float], 机器人的关节角度
    fn read_joint(&mut self) -> PyResult<[f64; HANS_DOF]> {
        self.0
            .state()
            .map(|s| s.joint.unwrap_or_default())
            .map_err(map_err)
    }

    /// 获取机器人的关节速度
    /// return:
    ///  List[float], 机器人的关节速度
    fn read_joint_vel(&mut self) -> PyResult<[f64; HANS_DOF]> {
        self.0
            .state()
            .map(|s| s.joint_vel.unwrap_or_default())
            .map_err(map_err)
    }

    /// 获取机器人的笛卡尔坐标
    /// return:
    ///  List[float], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    fn read_cartesian_euler(&mut self) -> PyResult<[f64; 6]> {
        self.0
            .state()
            .map(|s| {
                let (tran, rot) = s.pose_o_to_ee.unwrap_or_default().euler();
                [tran[0], tran[1], tran[2], rot[0], rot[1], rot[2]]
            })
            .map_err(map_err)
    }

    /// 获取机器人的笛卡尔速度
    /// return:
    ///  List[float], 机器人的笛卡尔速度
    fn read_cartesian_vel(&mut self) -> PyResult<[f64; 6]> {
        self.0
            .state()
            .map(|s| s.cartesian_vel.unwrap_or_default())
            .map_err(map_err)
    }

    /// 设置运动速度
    /// args:
    ///   speed: float, 速度
    fn set_speed(&mut self, speed: f64) -> PyResult<()> {
        self.0.set_speed(speed).map_err(map_err)
    }

    /// 以笛卡尔坐标的方式移动机器人
    fn version(&self) -> String {
        self.0.version()
    }

    /// 机器人对象初始化
    fn init(&mut self) -> PyResult<()> {
        self.0.init().map_err(map_err)
    }

    /// 机器人对象关闭
    fn shutdown(&mut self) -> PyResult<()> {
        self.0.shutdown().map_err(map_err)
    }

    /// 机器人使能
    fn enable(&mut self) -> PyResult<()> {
        self.0.enable().map_err(map_err)
    }

    /// 机器人去使能
    fn disable(&mut self) -> PyResult<()> {
        self.0.disable().map_err(map_err)
    }

    fn reset(&mut self) -> PyResult<()> {
        self.0.reset().map_err(map_err)
    }

    fn is_moving(&mut self) -> bool {
        self.0.is_moving()
    }

    fn stop(&mut self) -> PyResult<()> {
        self.0.stop().map_err(map_err)
    }

    /// 机器人恢复
    fn resume(&mut self) -> PyResult<()> {
        self.0.resume().map_err(map_err)
    }

    /// 机器人急停
    fn emergency_stop(&mut self) -> PyResult<()> {
        self.0.emergency_stop().map_err(map_err)
    }

    /// 清除机器人急停
    fn clear_emergency_stop(&mut self) -> PyResult<()> {
        self.0.clear_emergency_stop().map_err(map_err)
    }
}

#[pymodule]
fn libhans(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyHansRobot>()?;
    Ok(())
}

fn map_err(e: RobotException) -> pyo3::PyErr {
    PyException::new_err(e.to_string())
}
