use pyo3::{PyResult, exceptions::PyException, pyclass, pymethods};
use robot_behavior::{RobotBehavior, RobotException};

use crate::{HANS_DOF, HansRobot};

#[pyclass]
pub struct PyHansRobot {
    robot: HansRobot,
}

#[pymethods]
impl PyHansRobot {
    #[new]
    fn new(ip: &str) -> Self {
        PyHansRobot {
            robot: HansRobot::new(ip),
        }
    }

    fn connect(&mut self, ip: &str, port: u16) {
        self.robot.connect(ip, port)
    }

    fn disconnect(&mut self) {
        self.robot.disconnect()
    }

    fn move_joint(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.robot.move_joint(joint).map_err(map_err)
    }

    fn move_joint_rel(&mut self, joint: [f64; HANS_DOF]) -> PyResult<()> {
        self.robot.move_joint_rel(joint).map_err(map_err)
    }

    fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.robot.move_linear_with_euler(pose).map_err(map_err)
    }

    fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> PyResult<()> {
        self.robot.move_linear_with_euler_rel(pose).map_err(map_err)
    }

    fn version(&self) -> String {
        self.robot.version()
    }

    fn init(&mut self) -> PyResult<()> {
        self.robot.init().map_err(map_err)
    }

    fn shutdown(&mut self) -> PyResult<()> {
        self.robot.shutdown().map_err(map_err)
    }

    fn enable(&mut self) -> PyResult<()> {
        self.robot.enable().map_err(map_err)
    }

    fn disable(&mut self) -> PyResult<()> {
        self.robot.disable().map_err(map_err)
    }

    fn stop(&mut self) -> PyResult<()> {
        self.robot.stop().map_err(map_err)
    }

    fn resume(&mut self) -> PyResult<()> {
        self.robot.resume().map_err(map_err)
    }

    fn emergency_stop(&mut self) -> PyResult<()> {
        self.robot.emergency_stop().map_err(map_err)
    }

    fn clear_emergency_stop(&mut self) -> PyResult<()> {
        self.robot.clear_emergency_stop().map_err(map_err)
    }

    fn is_moving(&mut self) -> bool {
        self.robot.is_moving()
    }
}

fn map_err(e: RobotException) -> pyo3::PyErr {
    PyException::new_err(e.to_string())
}
