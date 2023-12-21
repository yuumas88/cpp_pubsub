#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <can_plugins2/msg/frame.hpp>
#include <can_utils.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace can_utils;

std::unique_ptr<can_plugins2::msg::Frame> generate_shirasu_frame(const uint16_t id, const float data)
{
  const int float_size = 4;  // float is 4 bytes.

  auto frame = std::make_unique<can_plugins2::msg::Frame>();
  frame->id = id;
  frame->is_rtr = false;
  frame->is_extended = false;
  frame->is_error = false;

  frame->dlc = float_size;

  can_pack<float>(frame->data, data);

  const auto tmp1 = frame->data[0];
  const auto tmp2 = frame->data[1];
  const auto tmp3 = frame->data[2];
  const auto tmp4 = frame->data[3];

  frame->data[0] = tmp4;
  frame->data[1] = tmp3;
  frame->data[2] = tmp2;
  frame->data[3] = tmp1;

  return frame;
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = can_plugins2::msg::Frame();
    auto msg1 = generate_shirasu_frame(0x720, 5);
    auto msg2 = generate_shirasu_frame(0x721, static_cast<float>(3.14));
    auto msg3 = generate_shirasu_frame(0x720, 3.14);

    publisher_->publish(*msg1);
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    // Additional variables (not used in this example)
    int count = 0;
    float maxSpeed = 0.0f;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
