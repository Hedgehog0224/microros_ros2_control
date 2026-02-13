#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher") {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    right_wheel = 0;
    left_wheel = 0;
    auto message_twist = geometry_msgs::msg::Twist();
    auto message = std_msgs::msg::Int16MultiArray();
    std::vector<int16_t> placeholder(5, 0);
    this->message.data = placeholder;

    publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_base_controller/cmd_vel_unstamped", 1);
    publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/robot/robot_state", 1);
    subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/robot/wheel_speeds", qos_profile, std::bind(&MinimalPublisher::topic_callback, this, _1));
    timer_ = this->create_wall_timer(50ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

  void timer_callback() {
    this->count_++;
    // == "/robot/robot_state" ==
    this->message.data[0] = 179;
    this->message.data[1] = right_wheel;
    this->message.data[2] = left_wheel;
    this->message.data[3] = 50;
    this->message.data[4] = 50;
    this->publisher_->publish(this->message);

    // == "/diff_drive_base_controller/cmd_vel_unstamped" ==
    this->message_twist.linear.x = 0.5;
    this->message_twist.linear.y = 0;
    this->message_twist.linear.z = 0;
    this->message_twist.angular.x = 0;
    this->message_twist.angular.y = 0;
    this->message_twist.angular.z = 0.5;
    this->publisher_twist_->publish(this->message_twist);

    if (this->count_ > 5) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 120000, "OZHIDAJU...");
    }
  }

  const void topic_callback(const std_msgs::msg::Int16MultiArray& msg) {
    this->count_ = 0;
    this->right_wheel = msg.data[0];
    this->left_wheel = msg.data[1];
    if ((this->right_wheel != 543) || (this->left_wheel != 2356)) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PYPYPY NE WORCKAET..");
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
                           "OLL WORCKAET KAK CHAS'S");
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
  int right_wheel;
  int left_wheel;
  std_msgs::msg::Int16MultiArray message;
  geometry_msgs::msg::Twist message_twist;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}