/***********************************************************************************************************************************
Плагин для ros2_control.

Аппаратные компоненты обеспечивают связь с физическим оборудованием и представляют его абстракцию в
фреймворке ros2_control. Компоненты должны быть экспортированы в виде плагинов с использованием
pluginlib-библиотеки. Диспетчер ресурсов динамически загружает эти плагины и управляет их жизненным
циклом.

Разработчик: Лебедева Екатерина
e.lebedeva@rtc.ru
***********************************************************************************************************************************/

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
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/int16_multi_array.hpp"

namespace microros_ros2_control {
/**
 * @class MicroRos2SystemHardware.
 * @brief Класс для связи ros-control и hardware реального робота.
 *
 * В классе переопределены базовые функции @see hardware_interface::SystemInterface для
 * реализации подписки и публикации в необходимые топики.
 */
class MicroRos2SystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MicroRos2SystemHardware);

  /**
   * @brief Инициализация.
   * @see hardware_interface::StateInterface
   *
   * Проверка состояния шарниров.
   * Инициализация ноды, подписчика и издателя.
   *
   * @param info Информация о шарнирах.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  // Documentation inherited
  // Унаследованная документация
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation inherited
  // Унаследованная документация
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation inherited
  // Унаследованная документация
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  // Documentation inherited
  // Унаследованная документация
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // Documentation inherited
  // Унаследованная документация
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  // Documentation inherited
  // Унаследованная документация
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  //// Блок данных для ROS2
  /// Get the logger of the SystemInterface.

  /**
   * @brief Логгирование.
   * @see rclcpp::Node
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /**
   * @brief Время системы.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  /**
   * @brief Нода.
   */
  std::shared_ptr<rclcpp::Node> node_{nullptr};

  /**
   * @brief Новый поток с ROS'ом.
   */
  std::thread thread_executor_spin_;

  /**
   * @brief Для работы ROS'а в многопотоке.
   */
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  /**
   * @brief Подписчик на состояние робота.
   */
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_micro_ros2_;
  // Название топика с данными от робота
  std::string robot_state = "/robot/robot_state";

  /**
   * @brief Подписка на состояние робота.
   */
  const void micro_ros2_callback(const std_msgs::msg::Int16MultiArray& msg);

  /**
   * @brief Издатель для движения робота.
   */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_micro_ros2_;
  // Название топика с данными для робота
  std::string wheel_speeds = "/robot/wheel_speeds";

  /**
   * @brief Сообщение для робота.
   */
  std_msgs::msg::Int16MultiArray message_microros_;

  /**
   * @brief Максимальная скорость.
   */
  const int MAX_SPEED = 2900;

private:
  double hw_start_sec_;
  double hw_stop_sec_;

  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};

}  // namespace microros_ros2_control

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
