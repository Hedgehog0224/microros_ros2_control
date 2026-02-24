#include "micro_ros_plugin.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace microros_ros2_control {
hardware_interface::CallbackReturn MicroRos2SystemHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
      rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));

  hw_start_sec_ = hardware_interface::stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = hardware_interface::stod(info_.hardware_parameters["hw_stop_duration_sec"]);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!rclcpp::ok()) {
    RCLCPP_DEBUG(get_logger(), "✓ Create default context");
    std::vector<const char*> argv;
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
  }
  std::string ns = "/";
  std::string node_name = "mini_node";
  this->node_ = rclcpp::Node::make_shared(node_name, ns);
  this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->executor_->add_node(this->node_);
  auto spin = [this]() { this->executor_->spin(); };
  this->thread_executor_spin_ = std::thread(spin);

  RCLCPP_DEBUG(get_logger(), "✓ Create node %s", node_name);

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
  qos_profile.best_effort();

  subscription_micro_ros2_ = node_->create_subscription<std_msgs::msg::Int16MultiArray>(
      this->robot_state, qos_profile,
      std::bind(&MicroRos2SystemHardware::micro_ros2_callback, this, std::placeholders::_1));
  publisher_micro_ros2_ =
      node_->create_publisher<std_msgs::msg::Int16MultiArray>(this->wheel_speeds, 1);

  std::vector<int16_t> placeholder(info_.joints.size(), 0);

  message_microros_.data = placeholder;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MicroRos2SystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MicroRos2SystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MicroRos2SystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

const void MicroRos2SystemHardware::micro_ros2_callback(const std_msgs::msg::Int16MultiArray& msg) {
  for (int i = 0; i < this->hw_velocities_.size(); i++) {
    this->hw_velocities_[i] = msg.data[i + 1] / MAX_SPEED;
  }
}

hardware_interface::CallbackReturn MicroRos2SystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MicroRos2SystemHardware::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& period) {
  for (std::size_t i = 0; i < hw_velocities_.size(); i++) {
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type microros_ros2_control ::MicroRos2SystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    this->message_microros_.data[i] = int(hw_commands_[i] * 2900);
  }
  this->publisher_micro_ros2_->publish(this->message_microros_);

  return hardware_interface::return_type::OK;
}

}  // namespace microros_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(microros_ros2_control::MicroRos2SystemHardware,
                       hardware_interface::SystemInterface)
