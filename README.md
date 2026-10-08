# libhans

[English](README.md) | [简体中文](README_cn.md)

`libhans` maps the Hans controller's text command protocol to Rust and the common [`robot_behavior`](https://github.com/Robot-Exp-Platform/robot_behavior) interfaces. The published **`libhans 0.2.0`** provides `HansS30`, state queries, target motion, and controller-specific commands.

The behavior layer is incomplete: **query measurements through `Arm::state()`; `Robot::read_state()` is unimplemented**. Also, `HansS30::new()` connects and attempts to set the controller speed override to `0.1`. Constructing the object is not a strictly read-only operation.

## Design and workflow

The driver separates request/response details from application capabilities. `HansRobot<T, N>` represents a model and joint count; `Arm`, `MoveTo<JointSpace<N>>`, and `MoveTo<FlangeSpace>` provide the common vocabulary. The public `robot_impl` field retains controller operations not covered by those traits.

The normal sequence is **connect → query arm state → configure and enable → send a target motion → wait for completion**. This backend uses blocking TCP requests. `state()` queries position, joint velocity, and TCP velocity separately, so its result is not an atomic snapshot of one controller cycle. Shared trait names do not imply that every other driver's control mode is implemented here.

## Install and prerequisites

Create a binary project with `cargo new hans-read-state`, then add:

```toml
[dependencies]
libhans = "0.2.0"
robot_behavior = "0.6.1"
```

The current crates require Rust nightly. New applications should use the published behavior 0.6.1 fixes. `libhans_derive 0.1.3` is an implementation dependency; ordinary driver users need not add it separately.

The default connection uses TCP **10003** and requires a reachable controller with the matching Hans command interface. Default Rust builds speak the protocol directly and do not link a separately installed vendor SDK. This release has no verified host/controller compatibility matrix or real-time timing guarantee.

| Feature | Meaning |
|---|---|
| Default, no features | Actual TCP request/response backend. |
| `no_robot` | Skip sockets and generate default protocol replies for development; not physical simulation. |
| `to_py` / `to_cxx` | Python/C++ binding code built on behavior FFI; not standalone language packages. |

`ffi` and `to_c` are declared too, but their presence does not establish a complete C SDK. Python integration needs a compatible development environment; C++ integration needs its toolchain. The driver itself has no `roplat` feature or native asynchronous control session.

## First program: query arm state

Put this in `src/main.rs`. Before deliberately running it, replace the address and confirm that setting the controller override to `0.1` is appropriate: **the constructor performs that setting**. The remaining code only queries state and sends no motion command.

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

Check without opening a socket:

```sh
cargo +nightly check
```

`new()` can panic on connection failure and discards the error from the initial override-setting call. Subsequent `state()` request failures propagate as `RobotResult`. State values are forwarded from controller replies into the common representation; confirm the controller's units and coordinates before using them in another backend's algorithm.

For an offline protocol exercise, enable `no_robot` in the dependency declaration. It prints commands and generates responses without connecting. Successful mock replies are not evidence of controller acceptance or measured robot state.

## Next steps and implemented boundaries

[The robot implementation](src/robot.rs) includes joint/flange target motion, load settings, and pause/stop/resume operations. These change device state or move the robot; use them after establishing the controller's operating configuration. [The command layer](src/robot_impl.rs) groups state, I/O, motion, and configuration requests. [The model definition](src/hans/hans_s.rs) contains current S30 parameters.

Important boundaries in 0.2.0:

- `Arm::state()` is implemented; `Robot::read_state()`, `emergency_stop()`, and `clear_emergency_stop()` still call `unimplemented!()` and panic.
- `get_joint()` and `get_endpoint()` return placeholders; use `state()` for controller queries.
- The backend does not implement common `ControlWith`/`AsyncControlWith` servo sessions. Low-level servo commands do not provide that session contract.
- Response parsing and some lower-level methods still use `unwrap`; not every failure becomes a returned error.

There is no standalone examples directory. Start with the complete program above, then the [published crate](https://crates.io/crates/libhans/0.2.0), [command types](src/types), and [behavior guide](https://github.com/Robot-Exp-Platform/robot_behavior). The linked source modules are the API reference when generated documentation is unavailable.

## Source builds and license

Registry users need no parent `drives` workspace. Checkout manifests pin development dependencies to Git revisions and require access to those sources, unlike the registry installation above.

The driver and derive helper are maintained by Robot-Exp-Platform under [Apache-2.0](LICENSE); the helper includes [its license](src/libhans_derive/LICENSE). Hans is the manufacturer's name. This library does not distribute or replace controller firmware or the manufacturer's operation documentation.
