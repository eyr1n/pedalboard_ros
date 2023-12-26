#include <algorithm>
#include <memory>

#include <boost/lockfree/spsc_queue.hpp>
#include <jack/jack.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pedalboard_msgs/msg/audio_sample.hpp"

namespace pedalboard {

rclcpp::Publisher<pedalboard_msgs::msg::AudioSample>::WeakPtr node_in_pub;
boost::lockfree::spsc_queue<pedalboard_msgs::msg::AudioSample::SharedPtr> out_queue(64);
jack_nframes_t jack_sample_rate;
jack_port_t *jack_in_port;
jack_port_t *jack_out_port;

class JunctionBox : public rclcpp::Node {
public:
  JunctionBox(const rclcpp::NodeOptions &options) : JunctionBox("", options) {}

  JunctionBox(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("junction_box", name_space, options) {
    in_pub_ = create_publisher<pedalboard_msgs::msg::AudioSample>("~/in", rclcpp::QoS(10));
    out_sub_ = create_subscription<pedalboard_msgs::msg::AudioSample>(
        "~/out", rclcpp::QoS(10),
        [](pedalboard_msgs::msg::AudioSample::UniquePtr msg) { out_queue.push(std::move(msg)); });
    node_in_pub = in_pub_;
    jack_initialize();
  }

private:
  rclcpp::Publisher<pedalboard_msgs::msg::AudioSample>::SharedPtr in_pub_;
  rclcpp::Subscription<pedalboard_msgs::msg::AudioSample>::SharedPtr out_sub_;
  std::shared_ptr<jack_client_t> jack_client_;

  void jack_initialize() {
    jack_status_t status;
    jack_client_ =
        std::shared_ptr<jack_client_t>(jack_client_open("junction_box", JackNullOption, &status), jack_client_close);
    if (!jack_client_) {
      RCLCPP_ERROR(get_logger(), "jack_client_open() failed, status = 0x%2.0x", status);
      if (status & JackServerFailed) {
        RCLCPP_ERROR(get_logger(), "Unable to connect to JACK server");
      }
      return;
    }
    if (status & JackServerStarted) {
      RCLCPP_INFO(get_logger(), "JACK server started");
    }
    if (status & JackNameNotUnique) {
      char *name = jack_get_client_name(jack_client_.get());
      RCLCPP_INFO(get_logger(), "unique name \"%s\" assigned", name);
    }

    jack_set_process_callback(jack_client_.get(), jack_process, 0);

    jack_sample_rate = jack_get_sample_rate(jack_client_.get());

    jack_in_port = jack_port_register(jack_client_.get(), "in", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
    jack_out_port = jack_port_register(jack_client_.get(), "out", JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);

    if (!jack_in_port || !jack_out_port) {
      RCLCPP_ERROR(get_logger(), "no more JACK ports available");
      return;
    }

    if (jack_activate(jack_client_.get())) {
      RCLCPP_ERROR(get_logger(), "cannot activate client");
      return;
    }

    {
      auto ports = std::shared_ptr<const char *>(
          jack_get_ports(jack_client_.get(), nullptr, nullptr, JackPortIsPhysical | JackPortIsOutput), jack_free);
      if (!ports) {
        RCLCPP_ERROR(get_logger(), "no physical capture ports");
        return;
      }
      if (jack_connect(jack_client_.get(), ports.get()[0], jack_port_name(jack_in_port))) {
        RCLCPP_ERROR(get_logger(), "cannot connect input ports");
      }
    }

    {
      auto ports = std::shared_ptr<const char *>(
          jack_get_ports(jack_client_.get(), nullptr, nullptr, JackPortIsPhysical | JackPortIsInput), jack_free);
      if (!ports) {
        RCLCPP_ERROR(get_logger(), "no physical playback ports");
        return;
      }
      if (jack_connect(jack_client_.get(), jack_port_name(jack_out_port), ports.get()[0])) {
        RCLCPP_ERROR(get_logger(), "cannot connect output ports");
      }
    }
  }

  static inline int jack_process(jack_nframes_t nframes, void *) {
    {
      jack_default_audio_sample_t *in =
          static_cast<jack_default_audio_sample_t *>(jack_port_get_buffer(jack_in_port, nframes));
      auto msg = std::make_unique<pedalboard_msgs::msg::AudioSample>();
      msg->sample_rate = jack_sample_rate;
      msg->nframes = nframes;
      msg->data.resize(nframes);
      std::copy(in, in + nframes, msg->data.begin());
      if (rclcpp::Publisher<pedalboard_msgs::msg::AudioSample>::SharedPtr node_in_pub_ = node_in_pub.lock()) {
        node_in_pub_->publish(std::move(msg));
      }
    }

    {
      jack_default_audio_sample_t *out =
          static_cast<jack_default_audio_sample_t *>(jack_port_get_buffer(jack_out_port, nframes));
      pedalboard_msgs::msg::AudioSample::SharedPtr msg;
      if (out_queue.pop(msg)) {
        std::copy(msg->data.begin(), msg->data.end(), out);
      } else {
        std::fill(out, out + nframes, 0.0f);
      }
    }

    return 0;
  }
};

} // namespace pedalboard

RCLCPP_COMPONENTS_REGISTER_NODE(pedalboard::JunctionBox)
