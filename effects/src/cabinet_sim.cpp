#include <fftw3.h>
#include <sndfile.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pedalboard_msgs/msg/audio_sample.hpp"

namespace pedalboard {

class CabinetSim : public rclcpp::Node {
public:
  CabinetSim(const rclcpp::NodeOptions &options) : CabinetSim("", options) {}

  CabinetSim(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("cabinet_sim", name_space, options) {
    declare_parameter("ir_wav", "");
    std::string ir_wav_path = get_parameter("ir_wav").as_string();

    SndfileHandle ir_wav(ir_wav_path);
    ir_data_.resize(ir_wav.frames());
    ir_wav.readf(ir_data_.data(), ir_data_.size());

    out_ = create_publisher<pedalboard_msgs::msg::AudioSample>("~/out", rclcpp::QoS(10));
    in_ = create_subscription<pedalboard_msgs::msg::AudioSample>(
        "~/in", rclcpp::QoS(10), [this](pedalboard_msgs::msg::AudioSample::UniquePtr msg) {
          size_t n = near_pow2(prev_data_.size() + msg->nframes + ir_data_.size() - 1);

          if (n > prev_n_) {
            prev_n_ = n;

            std::vector<float> ir_time(n, 0.0f);
            std::copy(ir_data_.begin(), ir_data_.end(), ir_time.begin());
            ir_freq_ = fftwf_alloc_complex(n / 2 + 1);

            fftwf_plan ir_plan = fftwf_plan_dft_r2c_1d(n, ir_time.data(), ir_freq_, FFTW_ESTIMATE);
            fftwf_execute(ir_plan);
            fftwf_destroy_plan(ir_plan);

            msg_time_.resize(n);
            msg_freq_ = fftwf_alloc_complex(n / 2 + 1);
          }

          // msg_time
          auto itr = std::copy(prev_data_.begin(), prev_data_.end(), msg_time_.begin());
          itr = std::copy(msg->data.begin(), msg->data.end(), itr);
          std::fill(itr, msg_time_.end(), 0.0f);
          prev_data_ = msg->data;

          // fft
          fftwf_plan msg_plan = fftwf_plan_dft_r2c_1d(n, msg_time_.data(), msg_freq_, FFTW_ESTIMATE);
          fftwf_execute(msg_plan);
          fftwf_destroy_plan(msg_plan);

          // convolution
          for (size_t i = 0; i < n / 2 + 1; ++i) {
            float real = msg_freq_[i][0] * ir_freq_[i][0] - msg_freq_[i][1] * ir_freq_[i][1];
            float imag = msg_freq_[i][0] * ir_freq_[i][1] + msg_freq_[i][1] * ir_freq_[i][0];
            msg_freq_[i][0] = real;
            msg_freq_[i][1] = imag;
          }

          // ifft
          fftwf_plan res_plan = fftwf_plan_dft_c2r_1d(n, msg_freq_, msg_time_.data(), FFTW_ESTIMATE);
          fftwf_execute(res_plan);
          fftwf_destroy_plan(res_plan);

          // normalize
          for (size_t i = 0; i < msg->nframes; ++i) {
            msg->data[i] = msg_time_[prev_data_.size() + i] / n;
          }

          out_->publish(std::move(msg));
        });
  }

  ~CabinetSim() {
    fftwf_free(ir_freq_);
    fftwf_free(msg_freq_);
  }

private:
  rclcpp::Publisher<pedalboard_msgs::msg::AudioSample>::SharedPtr out_;
  rclcpp::Subscription<pedalboard_msgs::msg::AudioSample>::SharedPtr in_;

  size_t prev_n_ = 0;
  std::vector<float> ir_data_;
  std::vector<float> prev_data_;
  std::vector<float> msg_time_;
  fftwf_complex *ir_freq_;
  fftwf_complex *msg_freq_;

  size_t near_pow2(size_t n) {
    size_t res = 1;
    while (res < n) {
      res <<= 1;
    }
    return res;
  }
};

} // namespace pedalboard

RCLCPP_COMPONENTS_REGISTER_NODE(pedalboard::CabinetSim)
