#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pedalboard_msgs/msg/audio_sample.hpp"

namespace pedalboard {

class Distortion : public rclcpp::Node {
public:
  Distortion(const rclcpp::NodeOptions &options) : Distortion("", options) {}

  Distortion(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("distortion", name_space, options) {
    declare_parameter("gain", 100.0);
    float gain = get_parameter("gain").as_double();
    declare_parameter("volume", 0.5);
    float volume = get_parameter("volume").as_double();

    out_ = create_publisher<pedalboard_msgs::msg::AudioSample>("~/out", rclcpp::QoS(10));
    in_ = create_subscription<pedalboard_msgs::msg::AudioSample>(
        "~/in", rclcpp::QoS(10), [this, gain, volume](pedalboard_msgs::msg::AudioSample::UniquePtr msg) {
          for (size_t i = 0; i < msg->nframes; ++i) {
            msg->data[i] = std::clamp(msg->data[i] * gain, -1.0f, 1.0f) * volume;
          }
          out_->publish(std::move(msg));
        });
  }

private:
  rclcpp::Publisher<pedalboard_msgs::msg::AudioSample>::SharedPtr out_;
  rclcpp::Subscription<pedalboard_msgs::msg::AudioSample>::SharedPtr in_;
};

} // namespace pedalboard

RCLCPP_COMPONENTS_REGISTER_NODE(pedalboard::Distortion)
