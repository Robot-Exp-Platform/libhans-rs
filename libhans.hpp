#pragma once

// Public C++ facade for Hans robot bindings.
// The current Rust crate exposes Python FFI; this header mirrors the shared
// robot_behavior shape for C++ consumers and future cxx bridge work.

#include "robot_behavior.hpp"

#include <array>
#include <memory>
#include <string>

namespace libhans {

using PoseData = robot_behavior::PoseData;
using LoadState = robot_behavior::LoadState;

class HansS30 : public robot_behavior::Arm<6>,
                public robot_behavior::JointMotion<6>,
                public robot_behavior::FlangeMotion {
public:
  std::string version() const override;
  void init() override;
  void shutdown() override;
  void enable() override;
  void disable() override;
  void reset() override;
  void stop() override;
  void emergency_stop() override;
  void clear_emergency_stop() override;
  bool is_moving() override;

  void connect(const std::string &ip, int port);
  void disconnect();

  std::string state() override;
  void set_load(const LoadState &load) override;
  std::array<double, 6> get_joint() const override;
  PoseData get_endpoint() const override;

  void move_joint(std::array<double, 6> target) override;
  void move_joint_sync(std::array<double, 6> target) override;
  void move_flange(PoseData target) override;
  void move_flange_sync(PoseData target) override;
};

std::unique_ptr<HansS30> hans_s30_attach(const std::string &ip);

} // namespace libhans
