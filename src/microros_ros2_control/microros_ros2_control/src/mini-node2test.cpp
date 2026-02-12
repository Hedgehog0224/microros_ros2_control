#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    right_wheel = 0;
    left_wheel = 0;

    publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/robot/robot_state", 20);

    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::Int16MultiArray();
      std::vector<int16_t> placeholder(5, 0);
      message.data = placeholder;
      message.data[0] = 179;
      message.data[1] = right_wheel;
      message.data[2] = left_wheel;
      message.data[3] = 50;
      message.data[4] = 50;
      this->publisher_->publish(message);
    };

    auto topic_callback = [this](std_msgs::msg::Int16MultiArray::UniquePtr msg) -> void {
      this->right_wheel = msg->data[0];
      this->left_wheel = msg->data[1];
    };

    subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>("/robot/wheel_speeds",
                                                                              10, topic_callback);

    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
  int right_wheel;
  int left_wheel;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}