#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pedalboard_msgs/msg/audio_sample.hpp"

namespace pedalboard {

class Delay : public rclcpp::Node {
public:
  Delay(const rclcpp::NodeOptions &options) : Delay("", options) {}

  Delay(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("delay", name_space, options) {
    declare_parameter("time", 200);
    int time = get_parameter("time").as_int();

    out_ = create_publisher<pedalboard_msgs::msg::AudioSample>("~/out", rclcpp::QoS(10));
    in_ = create_subscription<pedalboard_msgs::msg::AudioSample>(
        "~/in", rclcpp::QoS(10), [this, time](pedalboard_msgs::msg::AudioSample::UniquePtr msg) {
          int delay_nframes = msg->sample_rate * time / 1000;
          for (size_t i = 0; i < msg->nframes; ++i) {
            delay_buf_.push(msg->data[i]);
            if (delay_buf_.size() >= delay_nframes) {
              msg->data[i] += delay_buf_.front() * 0.8f;
              delay_buf_.pop();
            }
          }
          out_->publish(std::move(msg));
        });
  }

private:
  rclcpp::Publisher<pedalboard_msgs::msg::AudioSample>::SharedPtr out_;
  rclcpp::Subscription<pedalboard_msgs::msg::AudioSample>::SharedPtr in_;

  std::queue<float> delay_buf_;
};

} // namespace pedalboard

RCLCPP_COMPONENTS_REGISTER_NODE(pedalboard::Delay)
