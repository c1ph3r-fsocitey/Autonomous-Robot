// Copyright 2026
// Licensed under the Apache License, Version 2.0

#ifndef BOT_HARDWARE__DIFFDRIVE_MICROROS_HPP_
#define BOT_HARDWARE__DIFFDRIVE_MICROROS_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace bot_hardware
{

/**
 * ros2_control SystemInterface for a differential drive base whose low-level
 * motor control lives on an ESP32 running micro-ROS.
 *
 * This class does no serial I/O of its own. The micro-ROS agent already turns
 * the ESP32 into a normal ROS 2 participant, so all we have to do is bridge
 * two topics into ros2_control's command/state interfaces:
 *
 *   write() -> publishes [left_rad_s, right_rad_s] on `cmd_topic`
 *   read()  <- consumes  [l_pos, r_pos, l_vel, r_vel] from `state_topic`
 *
 * The subscription is serviced by a dedicated executor on its own thread so
 * that read() never blocks the controller manager's update loop.
 */
class DiffDriveMicroRos : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveMicroRos)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publish_zero_command();

  // --- configuration (from the <hardware> block in the URDF) ---------------
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  std::string cmd_topic_;
  std::string state_topic_;
  double state_timeout_sec_{1.0};

  // --- ros2_control storage ------------------------------------------------
  // Index 0 = left, index 1 = right, matching the order in export_*().
  //
  // std::array, not std::vector, on purpose: export_state_interfaces() hands
  // out raw pointers into these, and they have to stay valid for the whole
  // life of the component. A vector that ever reallocated would leave the
  // controller manager reading freed memory.
  std::array<double, 2> hw_commands_vel_{{0.0, 0.0}};
  std::array<double, 2> hw_states_pos_{{0.0, 0.0}};
  std::array<double, 2> hw_states_vel_{{0.0, 0.0}};

  // --- latest feedback, written by the executor thread ---------------------
  std::mutex state_mutex_;
  double last_pos_[2]{0.0, 0.0};
  double last_vel_[2]{0.0, 0.0};
  std::atomic<bool> got_first_state_{false};
  rclcpp::Time last_state_stamp_;

  // Only complain about a stale/missing ESP32 once per outage.
  bool timeout_reported_{false};

  // --- ROS plumbing --------------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<std::thread> executor_thread_;
  std::atomic<bool> executor_running_{false};

  // Reused so write() does not allocate on every controller cycle.
  std_msgs::msg::Float64MultiArray cmd_msg_;
};

}  // namespace bot_hardware

#endif  // BOT_HARDWARE__DIFFDRIVE_MICROROS_HPP_
