// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/clock.hpp"
// #include "rclcpp/duration.hpp"
// #include "rclcpp/logger.hpp"
// #include "rclcpp/macros.hpp"
// #include "rclcpp/publisher.hpp"
// #include "rclcpp/qos.hpp"
// #include "rclcpp/subscription.hpp"
// #include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/int16_multi_array.hpp"

namespace microros_ros2_control {
class MicroRos2SystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MicroRos2SystemHardware);

  //// Блок данных для контроллера
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  //// Блок данных для ROS2
  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  /// \brief Node Handles
  std::shared_ptr<rclcpp::Node> node_{nullptr};

  /// \brief Thread where the executor will spin
  std::thread thread_executor_spin_;

  /// \brief Executor to spin the controller
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  //// Блок данных о роботе
  // Подписчик на состояние робота
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_micro_ros2_;
  // Название топика с данными от робота
  std::string robot_state = "/robot/robot_state";
  // Колбэк для подписки
  const void micro_ros2_callback(const std_msgs::msg::Int16MultiArray& msg);

  // Издатель сообщения для робота
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_micro_ros2_;
  // Название топика с данными для робота
  std::string wheel_speeds = "/robot/wheel_speeds";
  // Сообщение для робота
  std_msgs::msg::Int16MultiArray message_microros_;
  // std::vector<double> velocities_from_microros_;
  const int MAX_SPEED = 2900;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace microros_ros2_control

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
