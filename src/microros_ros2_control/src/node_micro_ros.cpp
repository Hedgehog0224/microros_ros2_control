// Нода для создания подписчика и издателя в плагине micro_ros
// Created by Ekaterina Lebedeva on 2026/02/24.

#ifndef _MICROROS2NODE_H_
#define _MICROROS2NODE_H_

#include <cstring>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace microros_ros2_control {

class MicroRos2Node : public rclcpp::Node {
public:
  MicroRos2Node(const std::string& node_name, size_t size) : Node(node_name) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    subscription_micro_ros2_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        this->robot_state, qos_profile,
        std::bind(&MicroRos2Node::micro_ros2_callback, this, std::placeholders::_1));
    publisher_micro_ros2_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(this->wheel_speeds, 1);
    velocities_from_microros_.resize(size, std::numeric_limits<double>::quiet_NaN());
  }

  ~MicroRos2Node() { rclcpp::shutdown(); }

  const void micro_ros2_callback(const std_msgs::msg::Int16MultiArray& msg) {
    for (int i = 0; i < this->velocities_from_microros_.size(); i++) {
      this->velocities_from_microros_[i] = double(msg.data[i + 1]) / MAX_SPEED;
    }
  }

  void micro_ros2_publisher(std::vector<int16_t> comands_for_weels) {
    for (int i = 0; i < this->velocities_from_microros_.size(); i++) {
      this->message_microros_.data[i] = comands_for_weels[i];
    }
    this->publisher_micro_ros2_->publish(this->message_microros_);
  }

  std::vector<double> get_velocities_from_microros_() { return this->velocities_from_microros_; }

private:
  //// Блок данных о роботе
  // Подписчик на состояние робота
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_micro_ros2_;
  // Название топика с данными от робота
  std::string robot_state = "/robot/robot_state";
  // Колбэк для подписки
  //   const void micro_ros2_callback(const std_msgs::msg::Int16MultiArray& msg);

  // Издатель сообщения для робота
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_micro_ros2_;
  // Название топика с данными для робота
  std::string wheel_speeds = "/robot/wheel_speeds";
  // Сообщение для робота
  std_msgs::msg::Int16MultiArray message_microros_;
  std::vector<double> velocities_from_microros_;
  const int MAX_SPEED = 2900;
};

}  // namespace microros_ros2_control

#endif  // _MICROROS2NODE_H_
