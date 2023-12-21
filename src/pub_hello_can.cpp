#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <can_plugins2/msg/frame.hpp>
#include <can_utils.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace can_utils;

can_plugins2::msg::Frame generate_shirasu_mode(const uint16_t id, const uint8_t mode)
{
  const int byte_size = 1;  // Mode is 1 byte.
  
  can_plugins2::msg::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;

  frame.dlc = byte_size;

  frame.data[0] = mode;

  return frame;
}

can_plugins2::msg::Frame generate_shirasu_target(const uint16_t id, const float data)
{
  const int float_size = 4;  // float is 4 bytes.
  
  can_plugins2::msg::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;

  frame.dlc = float_size;

  can_pack<float>(frame.data, data);

  const auto tmp1 = frame.data[0];
  const auto tmp2 = frame.data[1];
  const auto tmp3 = frame.data[2];
  const auto tmp4 = frame.data[3];

  frame.data[0] = tmp4;
  frame.data[1] = tmp3;
  frame.data[2] = tmp2;
  frame.data[3] = tmp1;
  
  return frame;
}

const uint16_t shirasu_id = 0x720;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);

    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MinimalPublisher::joy_callback, this, _1)
    );

    ///@todo シラスのモードを変える
    timer_target_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_target_callback, this));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_msg->buttons[0] == 1)
    {
      can_plugins2::msg::Frame mode_msg = generate_shirasu_mode(shirasu_id, 5);
      publisher_->publish(mode_msg);
      RCLCPP_INFO(this->get_logger(), "Published Shirasu Mode: '%d'", mode_msg.data[0]);
    }
  }

  void timer_target_callback()
  {
    can_plugins2::msg::Frame target_msg = generate_shirasu_target(shirasu_id + 1, static_cast<float>(5.0));
    publisher_->publish(target_msg);
    RCLCPP_INFO(this->get_logger(),"any");
    // RCLCPP_INFO(this->get_logger(),"Published Shirasu Target: '%f'", target_msg.data);
  }

  rclcpp::TimerBase::SharedPtr timer_target_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
