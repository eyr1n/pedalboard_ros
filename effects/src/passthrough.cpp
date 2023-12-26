#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pedalboard_msgs/msg/audio_sample.hpp"

namespace pedalboard {

class Passthrough : public rclcpp::Node {
public:
  Passthrough(const rclcpp::NodeOptions &options) : Passthrough("", options) {}

  Passthrough(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("passthrough", name_space, options) {
    out_ = create_publisher<pedalboard_msgs::msg::AudioSample>("~/out", rclcpp::QoS(10));
    in_ = create_subscription<pedalboard_msgs::msg::AudioSample>(
        "~/in", rclcpp::QoS(10),
        [this](pedalboard_msgs::msg::AudioSample::UniquePtr msg) { out_->publish(std::move(msg)); });
  }

private:
  rclcpp::Publisher<pedalboard_msgs::msg::AudioSample>::SharedPtr out_;
  rclcpp::Subscription<pedalboard_msgs::msg::AudioSample>::SharedPtr in_;
};

} // namespace pedalboard

RCLCPP_COMPONENTS_REGISTER_NODE(pedalboard::Passthrough)
