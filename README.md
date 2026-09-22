# Readme

[English](README.md) | [简体中文](README_cn.md)

This library is part of the [Universal Robot Driver Project](https://github.com/Robot-Exp-Platform/robot_behavior)! We are committed to providing Rust driver support for more robotic platforms! **Unifying driver interfaces across different robot models, reducing the learning curve for robotics, and delivering more efficient robot control solutions!**

## Independent source checkout

This internal development baseline pins `robot_behavior` to a specific GitHub
commit because its current 0.6 API has not been published on crates.io. The
manifest is complete and does not inherit dependencies from a parent drives
workspace. Use an authorized SSH key/agent and
`CARGO_NET_GIT_FETCH_WITH_CLI=true` when building from a standalone checkout;
no sibling `robot_behavior` or `roplat` directory is required. Native driver
builds do not enable the optional roplat framework adapter.
