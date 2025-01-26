use serde_repr::{Deserialize_repr, Serialize_repr};

#[derive(Default, Debug, Serialize_repr, Deserialize_repr, PartialEq)]
#[repr(u16)]
pub enum RobotError {
    #[default]
    NoError,
    ControllerNotInit = 40000,
}
