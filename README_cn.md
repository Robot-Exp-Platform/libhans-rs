# libhans

[English](README.md) | [简体中文](README_cn.md)

`libhans` 将大族控制器的文本指令协议映射到 Rust 和统一的 [`robot_behavior`](https://github.com/Robot-Exp-Platform/robot_behavior) 接口。已发布的 **`libhans 0.2.0`** 提供 `HansS30`、状态查询、目标运动及控制器专有指令。

行为层尚不完整：**查询观测请使用 `Arm::state()`，`Robot::read_state()` 未实现**。另外，`HansS30::new()` 建立连接后会尝试把控制器速度倍率设为 `0.1`，因此构造对象不是严格只读操作。

## 设计与流程

驱动将请求/响应细节与应用能力分开。`HansRobot<T, N>` 表示机型与关节数，`Arm`、`MoveTo<JointSpace<N>>`、`MoveTo<FlangeSpace>` 提供通用表达，公开的 `robot_impl` 字段保留统一 trait 尚未覆盖的控制器操作。

典型顺序是 **连接 → 查询机械臂状态 → 配置并使能 → 发目标运动 → 等待完成**。本后端使用阻塞 TCP 请求。`state()` 分别查询位置、关节速度和 TCP 速度，因此不是同一控制周期的原子快照。trait 名称相同不意味着其他驱动的所有控制模式在这里都已实现。

## 安装与前置条件

先运行 `cargo new hans-read-state` 创建二进制项目，再加入：

```toml
[dependencies]
libhans = "0.2.0"
robot_behavior = "0.6.1"
```

当前 crate 需要 Rust nightly。新应用应使用行为库已发布的 0.6.1 修复。`libhans_derive 0.1.3` 是实现依赖，普通使用无需单独添加。

默认连接使用 TCP **10003**，需要可达且提供匹配大族指令接口的控制器。默认 Rust 构建直接实现协议，不链接另行安装的厂商 SDK。本版本没有已验证的主机/控制器兼容矩阵或实时周期保证。

| Feature | 含义 |
|---|---|
| 默认，无 feature | 实际 TCP 请求/响应后端。 |
| `no_robot` | 跳过 socket，生成默认协议响应，供开发使用，不是物理仿真。 |
| `to_py` / `to_cxx` | 基于行为库 FFI 的 Python/C++ 绑定代码，不是独立语言发行包。 |

manifest 还声明了 `ffi`、`to_c`，但这不代表存在完整 C SDK。Python 集成需要匹配的开发环境，C++ 集成需要相应工具链。本驱动自身没有 `roplat` feature 或原生异步控制会话。

## 第一个程序：查询机械臂状态

将代码放入 `src/main.rs`。决定实际运行前替换地址，并确认允许把控制器倍率设为 `0.1`：**构造器会执行这一设置**。其余代码只查询状态，不发送运动指令。

```rust,no_run
use libhans::HansS30;
use robot_behavior::{Arm, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = HansS30::new("192.168.0.10");
    let state = robot.state()?;
    println!("joint position: {:?}", state.joint.meas.q);
    println!("joint velocity: {:?}", state.joint.meas.dq);
    println!("flange pose: {:?}", state.flange.meas.pose);
    Ok(())
}
```

只检查编译，不打开 socket：

```sh
cargo +nightly check
```

`new()` 连接失败时可能 panic，并丢弃首次倍率设置调用的错误；后续 `state()` 请求失败通过 `RobotResult` 返回。状态实现将控制器响应放入统一表示，应用到其他后端算法前，应确认控制器的单位和坐标约定。

需要离线观察协议时，可在依赖声明中启用 `no_robot`。它不建立连接，而是打印指令并生成响应。mock 成功不代表控制器接受指令或获得了真实观测。

## 下一步与实现边界

[机器人实现](src/robot.rs)包含关节/法兰目标运动、负载设置及暂停、停止、继续操作。这些入口会改变设备状态或使机器人运动，应在控制器工作配置明确后使用。[指令层](src/robot_impl.rs)组织状态、I/O、运动和配置请求，[机型定义](src/hans/hans_s.rs)列出当前 S30 参数。

0.2.0 的重要边界：

- `Arm::state()` 已实现；`Robot::read_state()`、`emergency_stop()`、`clear_emergency_stop()` 仍执行 `unimplemented!()` 并 panic。
- `get_joint()`、`get_endpoint()` 返回占位值；查询控制器请用 `state()`。
- 后端未实现通用的 `ControlWith`/`AsyncControlWith` servo 会话；底层 servo 指令不等于实现该会话契约。
- 响应解析及部分底层方法仍使用 `unwrap`，并非所有失败都转换为返回错误。

本仓库没有独立 examples 目录。以上方完整程序为起点，再查阅[已发布 crate](https://crates.io/crates/libhans/0.2.0)、[指令类型](src/types)和[行为库指南](https://github.com/Robot-Exp-Platform/robot_behavior)。生成文档不可用时，可将上述源码模块作为 API 参考。

## 源码与许可

registry 用户不需要父级 `drives` 工作区。checkout manifest 将开发依赖固定到 Git revision，需要访问对应源码，与上面的 registry 安装不同。

驱动与派生宏由 Robot-Exp-Platform 维护，使用 [Apache-2.0](LICENSE)；宏 crate 也附带[许可证](src/libhans_derive/LICENSE)。Hans/大族是设备厂商名称，本库不分发或替代控制器固件及厂商操作文档。
