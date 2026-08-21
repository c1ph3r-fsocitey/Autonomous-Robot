// Copyright 2026
// Licensed under the Apache License, Version 2.0

#include "bot_hardware/diffdrive_microros.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr const char * kLogger = "DiffDriveMicroRos";
constexpr size_t kLeft = 0;
constexpr size_t kRight = 1;
}  // namespace

namespace bot_hardware
{

// =============================================================================
//  on_init
// =============================================================================
hardware_interface::CallbackReturn DiffDriveMicroRos::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [this](const std::string & key, const std::string & fallback) {
    auto it = info_.hardware_parameters.find(key);
    return (it != info_.hardware_parameters.end()) ? it->second : fallback;
  };

  left_wheel_name_  = get_param("left_wheel_name", "left_wheel_joint");
  right_wheel_name_ = get_param("right_wheel_name", "right_wheel_joint");
  cmd_topic_        = get_param("cmd_topic", "/wheel_cmd");
  state_topic_      = get_param("state_topic", "/wheel_state");

  try {
    state_timeout_sec_ = std::stod(get_param("state_timeout_sec", "1.0"));
  } catch (const std::exception &) {
    state_timeout_sec_ = 1.0;
  }

  // --- validate the joints declared in the URDF ----------------------------
  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kLogger),
      "Expected exactly 2 joints in the <ros2_control> block, got %zu.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "Joint '%s' must have exactly one command interface, of type '%s'.",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "Joint '%s' must expose state interfaces '%s' then '%s'.",
        joint.name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // The order of info_.joints must match our left/right indexing.
  if (info_.joints[kLeft].name != left_wheel_name_ ||
      info_.joints[kRight].name != right_wheel_name_)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger(kLogger),
      "Joint order mismatch. Expected '%s' then '%s' in the <ros2_control> "
      "block, but found '%s' then '%s'. List the left wheel first.",
      left_wheel_name_.c_str(), right_wheel_name_.c_str(),
      info_.joints[kLeft].name.c_str(), info_.joints[kRight].name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "Configured: cmd_topic='%s', state_topic='%s', timeout=%.2fs",
    cmd_topic_.c_str(), state_topic_.c_str(), state_timeout_sec_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
//  on_configure  --  create the bridge node and start its executor thread
// =============================================================================
hardware_interface::CallbackReturn DiffDriveMicroRos::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = rclcpp::Node::make_shared("bot_hardware_bridge");
  last_state_stamp_ = node_->now();

  // Best-effort, depth 1: the ESP32 publishes best-effort, and a stale wheel
  // state is worse than no wheel state.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic_, qos);

  state_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    state_topic_, qos,
    std::bind(&DiffDriveMicroRos::state_callback, this, std::placeholders::_1));

  cmd_msg_.data.assign(2, 0.0);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_running_ = true;
  executor_thread_ = std::make_unique<std::thread>([this]() {
    while (rclcpp::ok() && executor_running_) {
      executor_->spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Bridge node up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
//  on_cleanup
// =============================================================================
hardware_interface::CallbackReturn DiffDriveMicroRos::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  executor_running_ = false;
  if (executor_thread_ && executor_thread_->joinable()) {
    executor_thread_->join();
  }
  executor_thread_.reset();

  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
  state_sub_.reset();
  cmd_pub_.reset();
  executor_.reset();
  node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
//  Interface export
// =============================================================================
std::vector<hardware_interface::StateInterface>
DiffDriveMicroRos::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < 2; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffDriveMicroRos::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < 2; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]));
  }
  return command_interfaces;
}

// =============================================================================
//  Activate / deactivate
// =============================================================================
hardware_interface::CallbackReturn DiffDriveMicroRos::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_commands_vel_.fill(0.0);
  hw_states_vel_.fill(0.0);
  timeout_reported_ = false;

  // Seed the ros2_control position state with whatever the ESP32 last told us,
  // so activating mid-session does not produce a huge fake odometry jump.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hw_states_pos_[kLeft]  = last_pos_[kLeft];
    hw_states_pos_[kRight] = last_pos_[kRight];
  }

  publish_zero_command();

  if (!got_first_state_) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger),
      "Activated but no message on '%s' yet. Is micro_ros_agent running and is "
      "the ESP32's status LED solid?", state_topic_.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveMicroRos::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_commands_vel_.fill(0.0);
  publish_zero_command();
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Deactivated, motors commanded to zero.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
//  read / write
// =============================================================================
hardware_interface::return_type DiffDriveMicroRos::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!node_) {
    return hardware_interface::return_type::ERROR;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!got_first_state_) {
    // Nothing has arrived yet. Hold the last (zero) state rather than erroring
    // out, so the controller manager can still start up while the ESP32 boots.
    return hardware_interface::return_type::OK;
  }

  // Deliberately using the bridge node's clock for BOTH sides of this
  // subtraction. The `time` argument comes from the controller manager and can
  // be on a different clock type, which makes rclcpp::Time throw.
  const double age = (node_->now() - last_state_stamp_).seconds();
  if (age > state_timeout_sec_) {
    if (!timeout_reported_) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger),
        "No wheel state on '%s' for %.2f s - is the ESP32 still alive? "
        "Holding last position, reporting zero velocity.",
        state_topic_.c_str(), age);
      timeout_reported_ = true;
    }
    // Report zero velocity so odometry does not keep integrating a stale value.
    // We deliberately do NOT return ERROR: that would put the whole hardware
    // component into an unrecoverable error state and require a manual
    // `ros2 control set_hardware_component_state` to get moving again. The
    // ESP32's own 600 ms command watchdog is what actually stops the motors.
    hw_states_vel_[kLeft]  = 0.0;
    hw_states_vel_[kRight] = 0.0;
    return hardware_interface::return_type::OK;
  }

  if (timeout_reported_) {
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "Wheel state recovered.");
    timeout_reported_ = false;
  }

  hw_states_pos_[kLeft]  = last_pos_[kLeft];
  hw_states_pos_[kRight] = last_pos_[kRight];
  hw_states_vel_[kLeft]  = last_vel_[kLeft];
  hw_states_vel_[kRight] = last_vel_[kRight];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveMicroRos::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!cmd_pub_) {
    return hardware_interface::return_type::ERROR;
  }

  // NaN shows up when a controller is loaded but not yet receiving commands.
  const double left  = std::isfinite(hw_commands_vel_[kLeft])
                         ? hw_commands_vel_[kLeft] : 0.0;
  const double right = std::isfinite(hw_commands_vel_[kRight])
                         ? hw_commands_vel_[kRight] : 0.0;

  cmd_msg_.data[0] = left;
  cmd_msg_.data[1] = right;
  cmd_pub_->publish(cmd_msg_);

  return hardware_interface::return_type::OK;
}

// =============================================================================
//  Helpers
// =============================================================================
void DiffDriveMicroRos::state_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(kLogger), *node_->get_clock(), 5000,
      "Ignoring wheel state with %zu elements, expected 4.", msg->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  last_pos_[kLeft]  = msg->data[0];
  last_pos_[kRight] = msg->data[1];
  last_vel_[kLeft]  = msg->data[2];
  last_vel_[kRight] = msg->data[3];
  last_state_stamp_ = node_->now();
  got_first_state_  = true;
}

void DiffDriveMicroRos::publish_zero_command()
{
  if (!cmd_pub_) {
    return;
  }
  cmd_msg_.data[0] = 0.0;
  cmd_msg_.data[1] = 0.0;
  cmd_pub_->publish(cmd_msg_);
}

}  // namespace bot_hardware

PLUGINLIB_EXPORT_CLASS(bot_hardware::DiffDriveMicroRos, hardware_interface::SystemInterface)
