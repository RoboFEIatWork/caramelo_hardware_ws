#ifndef CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_
#define CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mobile_base_hardware
{

struct MaxonMotorConfig
{
  int pwm_gpio = -1;
  int dir_gpio = -1;  // Mantido por compatibilidade, mas o controle principal e PWM centrado.
  int enc_a_gpio = -1;
  int enc_b_gpio = -1;
  double command_sign = 1.0;
  double feedback_sign = 1.0;
};

struct MaxonDriverConfig
{
  std::string pigpio_host;
  std::string pigpio_port;
  int pwm_frequency_hz = 500;
  int pwm_range = 255;
  int encoder_glitch_filter_us = 1;
  double encoder_counts_per_wheel_rev = 28672.0;
  double max_wheel_rad_per_sec = 10.0;
  int update_period_ms = 10;
};

class MaxonMotorsNode : public rclcpp::Node
{
public:
  MaxonMotorsNode();
  ~MaxonMotorsNode() override;

  bool initialize(
    const MaxonDriverConfig & driver_config,
    const std::vector<MaxonMotorConfig> & motor_configs);

  void shutdown_hardware();
  bool is_initialized() const;

  void set_command_velocity(std::size_t motor_index, double wheel_velocity_rad_s);
  bool get_feedback(std::size_t motor_index, double & position_rad, double & velocity_rad_s) const;

  void stop_all_motors();

private:
  struct MotorRuntime
  {
    MaxonMotorConfig config;
    std::atomic<int64_t> encoder_count{0};
    int64_t previous_count = 0;
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double command_rad_s = 0.0;
    int last_state = 0;
    int callback_a = -1;
    int callback_b = -1;
  };

  struct CallbackContext
  {
    MaxonMotorsNode * self = nullptr;
    std::size_t motor_index = 0;
  };

  static void encoder_callback(
    int pi, unsigned gpio, unsigned level, uint32_t tick, void * userdata);
  void handle_encoder_edge(std::size_t motor_index);
  void update_cycle();
  int velocity_to_duty(double wheel_velocity_rad_s) const;
  int neutral_duty() const;

  MaxonDriverConfig driver_config_;
  std::vector<MotorRuntime> motors_;
  std::vector<CallbackContext> cb_ctx_a_;
  std::vector<CallbackContext> cb_ctx_b_;
  mutable std::mutex state_mutex_;

  double rad_per_count_ = 0.0;
  int pi_handle_ = -1;
  bool initialized_ = false;
  rclcpp::Time last_update_time_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
};

}  // namespace mobile_base_hardware

#endif  // CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_

