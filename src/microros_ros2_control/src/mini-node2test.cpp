/***********************************************************************************************************************************
Нода для эмулирования поведения stm'ки

Разработчик: Лебедева Екатерина
e.lebedeva@rtc.ru
***********************************************************************************************************************************/

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher") {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    qos_profile.best_effort();

    // right_wheel = 0;
    // left_wheel = 0;
    auto message_twist = geometry_msgs::msg::Twist();
    auto message = std_msgs::msg::Int16MultiArray();
    std::vector<int16_t> placeholder(5, 0);
    this->message.data = placeholder;
    this->config();

    if (mode_ == 1){
      publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "/diff_drive_base_controller/cmd_vel_unstamped", 1);}

    if ((mode_ == 2) || (mode_ == 3)){
      publisher_twist_stam_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
          "/articulated_frame_controller/cmd_vel", 1);}

    if (mode_ == 4){
      publisher_twist_stam_acer_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ackermann_steering_controller/reference", 1);}

    if ((mode_ == 1) || (mode_ == 2)){
      publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/robot/robot_state", 1);}

    timer_ = this->create_wall_timer(50ms, std::bind(&MinimalPublisher::timer_callback, this));
    count_[0] = 0.0;
    count_[1] = 0.0;
    count_[2] = 0.0;
  }
  int config() {
    std::cout << "REZHIM (1 - dif (x,z); 2 - artic-micro (Vf, Vt, w); 3 - aric-gz (V, w)); 4 - ackermann: ";
    std::cin >> mode_;
  }
  void check() {
    // std::string command;
    std::cout << "ZHDU COMANDU... ";
    if (mode_ != 2)
    {
      std::cin >> this->count_[0] >> this->count_[1];
      this->count_[2] = 42;
    }
    else
    {
      std::cin >> this->count_[0] >> this->count_[1] >> this->count_[2];
    }
  }
  void timer_callback() {
    if (mode_ != 1){
      auto message = geometry_msgs::msg::TwistStamped();
      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "base_link";
      message.twist.linear.x = this->count_[0];
      message.twist.linear.y = 0.0;
      message.twist.linear.z = 0.0;

      message.twist.angular.x = 0.0;
      message.twist.angular.y = 0.0;
      message.twist.angular.z = this->count_[1];

      if (mode_ == 4) {
      publisher_twist_stam_acer_->publish(message);}
      else{
      publisher_twist_stam_->publish(message);}
    }

    // == "/robot/robot_state" ==
    if ((mode_ == 1) || (mode_ == 2)){
      this->message.data[0] = 179;
      this->message.data[1] = this->count_[0];
      this->message.data[2] = this->count_[1];
      this->message.data[3] = this->count_[2];
      this->message.data[4] = 42;
      this->publisher_->publish(this->message);
    }

    // == "/diff_drive_base_controller/cmd_vel_unstamped" ==
    if (mode_ == 1){
      this->message_twist.linear.x = count_[0];
      this->message_twist.linear.y = 0;
      this->message_twist.linear.z = 0;
      this->message_twist.angular.x = 0;
      this->message_twist.angular.y = 0;
      this->message_twist.angular.z = count_[1];
      this->publisher_twist_->publish(this->message_twist);
    }
  }

  // const void topic_callback(const std_msgs::msg::Int16MultiArray& msg) {
  //   this->right_wheel = msg.data[0];
  //   this->left_wheel = msg.data[1];
  // }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_twist_stam_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_twist_stam_acer_;
  // rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;
  // int right_wheel;
  // int left_wheel;
  // int angle;
  std_msgs::msg::Int16MultiArray message;
  geometry_msgs::msg::Twist message_twist;
  int mode_;
  double count_[3];
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto Node = std::make_shared<MinimalPublisher>();

  std::thread spinner([Node]() { rclcpp::spin(Node); });

  while (rclcpp::ok()) {
    Node->check();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}