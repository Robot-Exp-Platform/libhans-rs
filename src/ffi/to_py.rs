use pyo3::{
    Bound, PyResult,
    exceptions::PyException,
    pyclass, pymethods, pymodule,
    types::{PyModule, PyModuleMethods},
};
use robot_behavior::{ArmBehavior, MotionType, RobotBehavior, RobotException};

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
        format!("HansRobot")
    }

    /// 连接到机器人
    /// args:
    ///    ip: str, 机器人的ip地址
    ///    port: int, 机器人的端口号
    #[pyo3(text_signature = "(ip, port = 10003)")]
    fn connect(&mut self, ip: &str, port: u16) {
        self.0.connect(ip, port)
    }

    /// 断开与机器人的连接
    fn disconnect(&mut self) {
        self.0.disconnect()
    }

    /// 以关节角度的方式移动机器人
    /// args:
    ///     joint: List[float], 机器人的关节角度
    #[pyo3(text_signature = "(joint)")]
    fn move_joint(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.0.move_joint(joint).map_err(map_err)
    }

    /// 以关节角度的方式异步移动机器人
    /// args:
    ///   joint: List[float], 机器人的关节角度
    #[pyo3(text_signature = "(joint)")]
    fn move_joint_async(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.0.move_joint_async(joint).map_err(map_err)
    }

    /// 以关节角度的方式相对移动机器人
    /// args:
    ///    joint: List[float], 机器人的关节角度
    #[pyo3(text_signature = "(joint_rel)")]
    fn move_joint_rel(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.0.move_joint_rel(joint).map_err(map_err)
    }

    /// 以关节角度的方式异步相对移动机器人
    /// args:
    ///  joint: List[float], 机器人的关节角度
    #[pyo3(text_signature = "(joint_rel)")]
    fn move_joint_rel_async(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.0.move_joint_rel_async(joint).map_err(map_err)
    }

    /// 以笛卡尔坐标的方式移动机器人
    /// args:
    ///   pose: List[float], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    #[pyo3(text_signature = "(pose)")]
    fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.0.move_linear_with_euler(pose).map_err(map_err)
    }

    /// 以笛卡尔坐标的方式异步移动机器人
    /// args:
    ///  pose: List[float], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    #[pyo3(text_signature = "(pose)")]
    fn move_linear_with_euler_async(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.0.move_linear_with_euler_async(pose).map_err(map_err)
    }

    /// 以笛卡尔坐标的方式相对移动机器人
    /// args:
    ///  pose: List[float], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    #[pyo3(text_signature = "(pose_rel)")]
    fn move_linear_rel_with_euler(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.0.move_linear_rel_with_euler(pose).map_err(map_err)
    }

    /// 以笛卡尔坐标的方式异步相对移动机器人
    /// args:
    ///  pose: List[float], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    #[pyo3(text_signature = "(pose_rel)")]
    fn move_linear_rel_with_euler_async(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.0
            .move_linear_rel_with_euler_async(pose)
            .map_err(map_err)
    }

    /// 以关节坐标移动连续轨迹
    /// args:
    ///  joints: List[List[float]], 机器人的关节角度
    fn move_joint_path(&mut self, joints: Vec<[f64; HANS_DOF]>) -> PyResult<()> {
        let joints = joints.into_iter().map(|j| MotionType::Joint(j)).collect();
        self.0.move_path(joints).map_err(map_err)
    }

    /// 从文件中读取，并以关节坐标移动连续轨迹
    /// args:n
    ///   path: str, 文件路径
    fn move_joint_path_from_file(&mut self, path: &str) -> PyResult<()> {
        self.0.move_path_from_file(path).map_err(map_err)
    }

    /// 以笛卡尔坐标移动连续轨迹
    /// args:
    ///   poses: List[List[float]], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    fn move_linear_path_with_euler(&mut self, poses: Vec<[f64; 6]>) -> PyResult<()> {
        let poses = poses
            .into_iter()
            .map(|p| MotionType::CartesianEuler(p))
            .collect();
        self.0.move_path(poses).map_err(map_err)
    }

    /// 从文件中读取，并以笛卡尔坐标移动连续轨迹
    /// args:
    ///   path: str, 文件路径
    fn move_linear_path_with_euler_from_file(&mut self, path: &str) -> PyResult<()> {
        self.0.move_path_from_file(path).map_err(map_err)
    }

    /// 获取机器人的关节角度
    /// return:
    ///   List[float], 机器人的关节角度
    fn read_joint(&mut self) -> PyResult<[f64; HANS_DOF]> {
        self.0.read_joint().map_err(map_err)
    }

    /// 获取机器人的关节速度
    /// return:
    ///  List[float], 机器人的关节速度
    fn read_joint_vel(&mut self) -> PyResult<[f64; HANS_DOF]> {
        self.0.read_joint_vel().map_err(map_err)
    }

    /// 获取机器人的笛卡尔坐标
    /// return:
    ///  List[float], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    fn read_cartesian_euler(&mut self) -> PyResult<[f64; 6]> {
        self.0.read_cartesian_euler().map_err(map_err)
    }

    /// 获取机器人的笛卡尔速度
    /// return:
    ///  List[float], 机器人的笛卡尔速度
    fn read_cartesian_vel(&mut self) -> PyResult<[f64; 6]> {
        self.0.read_cartesian_vel().map_err(map_err)
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
        self.robot.reset().map_err(map_err)
    }

    fn is_moving(&mut self) -> bool {
        self.robot.is_moving()
    }

    fn move_joint(&mut self, joint: [f64; HANS_DOF], speed: f64) -> PyResult<()> {
        self.robot.move_joint(&joint, speed).map_err(map_err)
    }

    fn move_joint_rel(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.robot.move_joint_rel(&joint).map_err(map_err)
    }

    fn move_linear_with_euler(&mut self, pose: [f64; 6], speed: f64) -> PyResult<()> {
        self.robot
            .move_linear_with_euler(&pose, speed)
            .map_err(map_err)
    }

    fn reset(&mut self) -> PyResult<()> {
        self.robot.reset().map_err(map_err)
    }

    fn is_moving(&mut self) -> bool {
        self.robot.is_moving()
    }

    fn move_joint(&mut self, joint: [f64; HANS_DOF], speed: f64) -> PyResult<()> {
        self.robot.move_joint(&joint, speed).map_err(map_err)
    }

    fn move_joint_rel(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.robot.move_joint_rel(&joint).map_err(map_err)
    }

    fn move_linear_with_euler(&mut self, pose: [f64; 6], speed: f64) -> PyResult<()> {
        self.robot
            .move_linear_with_euler(&pose, speed)
            .map_err(map_err)
    }

    fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.robot.move_linear_with_euler_rel(pose).map_err(map_err)
    }

    fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.robot.move_linear_with_euler_rel(pose).map_err(map_err)
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

    /// 获取机器人的关节角度
    /// return:
    ///    List[float], 机器人的关节角度
    fn is_moving(&mut self) -> bool {
        self.0.is_moving()
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
