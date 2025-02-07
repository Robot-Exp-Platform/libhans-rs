use crate::robot_error::RobotError;

pub trait CommandSerde: Sized {
    fn to_string(&self) -> String;
    fn from_str(data: &str) -> Result<Self, RobotError>;
}

impl CommandSerde for RobotError {
    fn to_string(&self) -> String {
        serde_json::to_string(self).unwrap()
    }
    fn from_str(data: &str) -> Result<Self, RobotError> {
        serde_json::from_str(data).map_err(|_| RobotError::DeserializeError)
    }
}

impl CommandSerde for () {
    fn to_string(&self) -> String {
        String::new()
    }
    fn from_str(_: &str) -> Result<Self, RobotError> {
        Ok(())
    }
}

impl CommandSerde for bool {
    fn to_string(&self) -> String {
        format!("{}", if *self { 1 } else { 0 })
    }

    fn from_str(data: &str) -> Result<Self, RobotError> {
        match data {
            "0" => Ok(false),
            "1" => Ok(true),
            _ => Err(RobotError::DeserializeError),
        }
    }
}

impl CommandSerde for u8 {
    fn to_string(&self) -> String {
        format!("{}", self)
    }
    fn from_str(data: &str) -> Result<Self, RobotError> {
        data.parse().map_err(|_| RobotError::DeserializeError)
    }
}

impl CommandSerde for u16 {
    fn to_string(&self) -> String {
        format!("{}", self)
    }
    fn from_str(data: &str) -> Result<Self, RobotError> {
        data.parse().map_err(|_| RobotError::DeserializeError)
    }
}

impl CommandSerde for f64 {
    fn to_string(&self) -> String {
        format!("{}", self)
    }
    fn from_str(data: &str) -> Result<Self, RobotError> {
        data.parse().map_err(|_| RobotError::DeserializeError)
    }
}

impl<T1, T2> CommandSerde for (T1, T2)
where
    T1: CommandSerde,
    T2: CommandSerde,
{
    fn to_string(&self) -> String {
        format!("{},{}", self.0.to_string(), self.1.to_string())
    }
    fn from_str(data: &str) -> Result<Self, RobotError> {
        let mut data = data.split(',');
        Ok((
            T1::from_str(data.next().unwrap())?,
            T2::from_str(data.next().unwrap())?,
        ))
    }
}

impl<T1, T2, T3> CommandSerde for (T1, T2, T3)
where
    T1: CommandSerde,
    T2: CommandSerde,
    T3: CommandSerde,
{
    fn to_string(&self) -> String {
        format!(
            "{},{},{}",
            self.0.to_string(),
            self.1.to_string(),
            self.2.to_string()
        )
    }
    fn from_str(data: &str) -> Result<Self, RobotError> {
        let mut data = data.split(',');
        Ok((
            T1::from_str(data.next().unwrap())?,
            T2::from_str(data.next().unwrap())?,
            T3::from_str(data.next().unwrap())?,
        ))
    }
}

impl<const N: usize, T> CommandSerde for [T; N]
where
    T: CommandSerde + Copy,
{
    fn to_string(&self) -> String {
        self.iter()
            .map(|x| x.to_string())
            .collect::<Vec<_>>()
            .join(",")
    }
    fn from_str(data: &str) -> Result<Self, RobotError> {
        let mut data = data.split(',');
        assert_eq!(data.size_hint().0, N);
        Ok([T::from_str(data.next().unwrap()).unwrap(); N])
    }
}

impl CommandSerde for String {
    fn to_string(&self) -> String {
        self.clone()
    }
    fn from_str(data: &str) -> Result<Self, RobotError> {
        Ok(data.to_string())
    }
}
