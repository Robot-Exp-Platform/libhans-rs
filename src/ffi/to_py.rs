use pyo3::{PyResult, pyclass, pymethods};
use robot_behavior::{
    behavior::*, py_arm_behavior, py_arm_param, py_arm_preplanned_motion,
    py_arm_preplanned_motion_ext, py_arm_preplanned_motion_impl, py_robot_behavior,
};

use crate::{HANS_DOF, HansS30};

#[pyclass(name = "HansS30")]
pub struct PyHansS30(HansS30);

#[pymethods]
impl PyHansS30 {
    #[new]
    fn new(ip: &str) -> Self {
        PyHansS30(HansS30::new(ip))
    }

    fn __repr__(&self) -> String {
        "HansS30".to_string()
    }

    fn connect(&mut self, ip: &str, port: u16) {
        self.0.connect(ip, port)
    }

    fn disconnect(&mut self) {
        self.0.disconnect()
    }

    fn read_joint(&mut self) -> PyResult<[f64; HANS_DOF]> {
        self.0
            .state()
            .map(|s| s.joint.unwrap_or_default())
            .map_err(Into::into)
    }

    fn read_joint_vel(&mut self) -> PyResult<[f64; HANS_DOF]> {
        self.0
            .state()
            .map(|s| s.joint_vel.unwrap_or_default())
            .map_err(Into::into)
    }

    fn read_cartesian_euler(&mut self) -> PyResult<[f64; 6]> {
        self.0
            .state()
            .map(|s| s.pose_o_to_ee.unwrap_or_default().into())
            .map_err(Into::into)
    }

    fn read_cartesian_vel(&mut self) -> PyResult<[f64; 6]> {
        self.0
            .state()
            .map(|s| s.cartesian_vel.unwrap_or_default())
            .map_err(Into::into)
    }
}

py_robot_behavior!(PyHansS30(HansS30));
py_arm_behavior!(PyHansS30<{6}>(HansS30));
py_arm_param!(PyHansS30<{6}>(HansS30));
py_arm_preplanned_motion!(PyHansS30<{6}>(HansS30));
py_arm_preplanned_motion_ext!(PyHansS30<{6}>(HansS30));
py_arm_preplanned_motion_impl!(PyHansS30<{6}>(HansS30));
