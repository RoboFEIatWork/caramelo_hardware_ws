#ifndef CARAMELO_HARDWARE__MAXON_GPIO_DRIVER_HPP_
#define CARAMELO_HARDWARE__MAXON_GPIO_DRIVER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

namespace mobile_base_hardware
{

struct MaxonMotorConfig
{
  int pwm_gpio = -1;
  int dir_gpio = -1;  // Nao usado no modo PWM centrado (50% = parado).
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
  double encoder_counts_per_wheel_rev = 28672.0;  // 1024 * 28
  double max_wheel_rad_per_sec = 10.0;
};

class MaxonGpioDriver
{
public:
  MaxonGpioDriver();
  ~MaxonGpioDriver();

  bool initialize(
    const MaxonDriverConfig & driver_config,
    const std::vector<MaxonMotorConfig> & motor_configs,
    const rclcpp::Logger & logger);

  void shutdown();
  bool is_initialized() const;

  void stop_all_motors();

  bool set_motor_velocity(std::size_t motor_index, double wheel_velocity_rad_s);

  bool read_motor_state(
    std::size_t motor_index,
    double period_sec,
    double & position_rad,
    double & velocity_rad_s);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  bool initialized_ = false;
};

}  // namespace mobile_base_hardware

#endif  // CARAMELO_HARDWARE__MAXON_GPIO_DRIVER_HPP_
