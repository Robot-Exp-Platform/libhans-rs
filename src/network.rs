use std::io::{Read, Write};
use std::net::{Shutdown, TcpStream};
use std::time::Duration;

use robot_behavior::RobotException;

use crate::exception::HansResult;
use crate::types::CommandSerde;

pub const PORT_IF: u16 = 10003;
pub const PORT_DATASHEET_JSON_1: u16 = 10004;
pub const PORT_DATASHEET_JSON_2: u16 = 10005;
pub const PORT_DATASHEET_JSON_3: u16 = 10006;
pub const PORT_DATASHEET_STRUCT_1: u16 = 10014;
pub const PORT_DATASHEET_STRUCT_2: u16 = 10015;
pub const PORT_DATASHEET_STRUCT_3: u16 = 10016;
pub const PORT_MODBUSTCP: u16 = 10502;

#[derive(Default)]
pub struct Network {
    socket: Option<TcpStream>,
    connected: bool,
}

impl Network {
    pub fn from_ip(host: &str, port: u16) -> Self {
        let mut network = Network::default();
        network.connect_to_cps(host, port).unwrap();
        network
    }

    pub fn from_defult_port(host: &str) -> Self {
        Network::from_ip(host, PORT_IF)
    }

    /// 连接到指定 IP 与端口
    pub fn connect_to_cps(&mut self, host: &str, port: u16) -> HansResult<()> {
        let addr = format!("{}:{}", host, port);
        let stream = TcpStream::connect(&addr)?;

        stream.set_read_timeout(Some(Duration::from_secs(3)))?;
        stream.set_write_timeout(Some(Duration::from_secs(3)))?;

        self.socket = Some(stream);
        self.connected = true;
        Ok(())
    }

    /// 断开 TCP 连接
    pub fn disconnect_from_cps(&mut self) -> HansResult<()> {
        if let Some(stream) = &self.socket {
            stream.shutdown(Shutdown::Both)?;
        }
        self.socket = None;
        self.connected = false;
        Ok(())
    }

    /// 判断是否已连接
    pub fn is_connected(&self) -> bool {
        self.connected
    }

    /// 发送命令并等待返回
    pub fn send_and_recv<R, S>(&mut self, cmd: &R) -> HansResult<S>
    where
        R: CommandSerde,
        S: CommandSerde,
    {
        if let Some(stream) = &mut self.socket {
            stream.write_all(cmd.to_string().as_bytes())?;
            let mut buffer = [0_u8; 1024];
            let n = stream.read(&mut buffer)?;
            Ok(S::from_str(&String::from_utf8_lossy(&buffer[..n])).unwrap())
        } else {
            Err(RobotException::NetworkError(
                "No active TCP connection.".into(),
            ))
        }
    }
}
