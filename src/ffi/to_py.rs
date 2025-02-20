use pyo3::{
    Bound, PyResult,
    exceptions::PyException,
    pyclass, pymethods, pymodule,
    types::{PyModule, PyModuleMethods},
};
use robot_behavior::{RobotBehavior, RobotException};

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

    /// 以关节角度的方式相对移动机器人
    /// args:
    ///    joint: List[float], 机器人的关节角度
    #[pyo3(text_signature = "(joint_rel)")]
    fn move_joint_rel(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.0.move_joint_rel(joint).map_err(map_err)
    }

    /// 以笛卡尔坐标的方式移动机器人
    /// args:
    ///   pose: List[float], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    #[pyo3(text_signature = "(pose)")]
    fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.0.move_linear_with_euler(pose).map_err(map_err)
    }

    /// 以笛卡尔坐标的方式相对移动机器人
    /// args:
    ///  pose: List[float], 机器人的笛卡尔坐标，其中前三个元素为位置，后三个元素为欧拉角
    #[pyo3(text_signature = "(pose_rel)")]
    fn move_linear_rel_with_euler(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.0.move_linear_rel_with_euler(pose).map_err(map_err)
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

    /// 机器人停止
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
