#include "caramelo_hardware/maxon_gpio_driver.hpp"

#include <atomic>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
#include <pigpiod_if2.h>
#include <pigpio.h>
#endif

namespace mobile_base_hardware
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr int kInvalidCallbackId = -1;

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
int neutral_pwm_duty(int pwm_range)
{
  return static_cast<int>(std::lround(static_cast<double>(pwm_range) * 0.5));
}
#endif
}  // namespace

struct MaxonGpioDriver::Impl
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  struct CallbackContext
  {
    Impl * self = nullptr;
    std::size_t motor_index = 0;
  };

  struct MotorRuntime
  {
    MaxonMotorConfig config;
    std::atomic<int64_t> encoder_count{0};
    int64_t previous_count = 0;
    double position_rad = 0.0;
    int last_state = 0;
    bool warned_negative_without_dir = false;
    int callback_a = kInvalidCallbackId;
    int callback_b = kInvalidCallbackId;
    CallbackContext cb_ctx_a;
    CallbackContext cb_ctx_b;
  };

  static void encoder_callback(
    int /*pi*/, unsigned int /*gpio*/, unsigned int /*level*/, uint32_t /*tick*/, void * userdata)
  {
    auto * ctx = static_cast<CallbackContext *>(userdata);
    if (ctx == nullptr || ctx->self == nullptr) {
      return;
    }
    ctx->self->handle_encoder_edge(ctx->motor_index);
  }

  void handle_encoder_edge(std::size_t motor_index)
  {
    if (motor_index >= motors.size()) {
      return;
    }

    auto & motor = motors[motor_index];
    const int a = gpio_read(pi_handle, motor.config.enc_a_gpio);
    const int b = gpio_read(pi_handle, motor.config.enc_b_gpio);
    if (a < 0 || b < 0) {
      return;
    }

    const int new_state = ((a != 0) << 1) | (b != 0);
    const int transition = (motor.last_state << 2) | new_state;
    static constexpr std::array<int8_t, 16> kQuadratureDelta{
      0, -1, +1, 0,
      +1, 0, 0, -1,
      -1, 0, 0, +1,
      0, +1, -1, 0};

    const int8_t delta = kQuadratureDelta[transition];
    if (delta != 0) {
      motor.encoder_count.fetch_add(delta, std::memory_order_relaxed);
    }
    motor.last_state = new_state;
  }

  void cancel_callbacks()
  {
    for (auto & motor : motors) {
      if (motor.callback_a >= 0) {
        callback_cancel(motor.callback_a);
        motor.callback_a = kInvalidCallbackId;
      }
      if (motor.callback_b >= 0) {
        callback_cancel(motor.callback_b);
        motor.callback_b = kInvalidCallbackId;
      }
    }
  }
#endif

  rclcpp::Logger logger = rclcpp::get_logger("maxon_gpio_driver");
  MaxonDriverConfig driver_config;
  double rad_per_count = 0.0;

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  int pi_handle = -1;
  std::vector<MotorRuntime> motors;
#endif
};

MaxonGpioDriver::MaxonGpioDriver() : impl_(std::make_unique<Impl>()) {}

MaxonGpioDriver::~MaxonGpioDriver()
{
  shutdown();
}

bool MaxonGpioDriver::initialize(
  const MaxonDriverConfig & driver_config,
  const std::vector<MaxonMotorConfig> & motor_configs,
  const rclcpp::Logger & logger)
{
  shutdown();
  impl_->logger = logger;
  impl_->driver_config = driver_config;

  if (motor_configs.empty()) {
    RCLCPP_ERROR(impl_->logger, "No motors configured for Maxon GPIO driver.");
    return false;
  }

  const double counts_per_rev = std::max(1.0, impl_->driver_config.encoder_counts_per_wheel_rev);
  impl_->rad_per_count = kTwoPi / counts_per_rev;

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  impl_->pi_handle = pigpio_start(
    impl_->driver_config.pigpio_host.empty() ? nullptr : impl_->driver_config.pigpio_host.c_str(),
    impl_->driver_config.pigpio_port.empty() ? nullptr : impl_->driver_config.pigpio_port.c_str());
  if (impl_->pi_handle < 0) {
    RCLCPP_ERROR(
      impl_->logger,
      "Failed to connect to pigpiod (host='%s', port='%s').",
      impl_->driver_config.pigpio_host.empty() ? "localhost" : impl_->driver_config.pigpio_host.c_str(),
      impl_->driver_config.pigpio_port.empty() ? "8888" : impl_->driver_config.pigpio_port.c_str());
    return false;
  }

  impl_->motors.clear();
  impl_->motors.reserve(motor_configs.size());
  for (std::size_t i = 0; i < motor_configs.size(); ++i) {
    impl_->motors.emplace_back();
    auto & runtime = impl_->motors.back();
    runtime.config = motor_configs[i];
    runtime.cb_ctx_a.self = impl_.get();
    runtime.cb_ctx_a.motor_index = i;
    runtime.cb_ctx_b.self = impl_.get();
    runtime.cb_ctx_b.motor_index = i;

    if (
      runtime.config.pwm_gpio < 0 || runtime.config.enc_a_gpio < 0 ||
      runtime.config.enc_b_gpio < 0)
    {
      RCLCPP_ERROR(
        impl_->logger,
        "Invalid GPIO config for motor %zu (pwm=%d, enc_a=%d, enc_b=%d).",
        i, runtime.config.pwm_gpio, runtime.config.enc_a_gpio, runtime.config.enc_b_gpio);
      shutdown();
      return false;
    }

    if (set_mode(impl_->pi_handle, runtime.config.pwm_gpio, PI_OUTPUT) < 0) {
      RCLCPP_ERROR(impl_->logger, "Failed to set PWM pin mode for motor %zu.", i);
      shutdown();
      return false;
    }
    if (set_PWM_frequency(
      impl_->pi_handle, runtime.config.pwm_gpio,
      static_cast<unsigned>(impl_->driver_config.pwm_frequency_hz)) < 0)
    {
      RCLCPP_ERROR(impl_->logger, "Failed to set PWM frequency for motor %zu.", i);
      shutdown();
      return false;
    }
    if (set_PWM_range(
      impl_->pi_handle, runtime.config.pwm_gpio,
      static_cast<unsigned>(impl_->driver_config.pwm_range)) < 0)
    {
      RCLCPP_ERROR(impl_->logger, "Failed to set PWM range for motor %zu.", i);
      shutdown();
      return false;
    }
    if (set_PWM_dutycycle(
      impl_->pi_handle, runtime.config.pwm_gpio,
      neutral_pwm_duty(impl_->driver_config.pwm_range)) < 0)
    {
      RCLCPP_ERROR(impl_->logger, "Failed to set initial PWM dutycycle for motor %zu.", i);
      shutdown();
      return false;
    }

    if (runtime.config.dir_gpio >= 0) {
      if (set_mode(impl_->pi_handle, runtime.config.dir_gpio, PI_OUTPUT) < 0) {
        RCLCPP_ERROR(impl_->logger, "Failed to set direction pin mode for motor %zu.", i);
        shutdown();
        return false;
      }
      if (gpio_write(impl_->pi_handle, runtime.config.dir_gpio, 0) < 0) {
        RCLCPP_ERROR(impl_->logger, "Failed to write initial direction level for motor %zu.", i);
        shutdown();
        return false;
      }
    }

    if (
      set_mode(impl_->pi_handle, runtime.config.enc_a_gpio, PI_INPUT) < 0 ||
      set_mode(impl_->pi_handle, runtime.config.enc_b_gpio, PI_INPUT) < 0)
    {
      RCLCPP_ERROR(impl_->logger, "Failed to set encoder pin mode for motor %zu.", i);
      shutdown();
      return false;
    }
    if (
      set_pull_up_down(impl_->pi_handle, runtime.config.enc_a_gpio, PI_PUD_OFF) < 0 ||
      set_pull_up_down(impl_->pi_handle, runtime.config.enc_b_gpio, PI_PUD_OFF) < 0)
    {
      RCLCPP_ERROR(impl_->logger, "Failed to set encoder pull-up/down for motor %zu.", i);
      shutdown();
      return false;
    }
    if (set_glitch_filter(
      impl_->pi_handle, runtime.config.enc_a_gpio,
      static_cast<unsigned>(std::max(0, impl_->driver_config.encoder_glitch_filter_us))) < 0 ||
      set_glitch_filter(
      impl_->pi_handle, runtime.config.enc_b_gpio,
      static_cast<unsigned>(std::max(0, impl_->driver_config.encoder_glitch_filter_us))) < 0)
    {
      RCLCPP_ERROR(impl_->logger, "Failed to set encoder glitch filter for motor %zu.", i);
      shutdown();
      return false;
    }

    const int a0 = gpio_read(impl_->pi_handle, runtime.config.enc_a_gpio);
    const int b0 = gpio_read(impl_->pi_handle, runtime.config.enc_b_gpio);
    runtime.last_state = ((a0 != 0) << 1) | (b0 != 0);

    runtime.callback_a = callback_ex(
      impl_->pi_handle,
      static_cast<unsigned>(runtime.config.enc_a_gpio),
      PI_EITHER_EDGE,
      &Impl::encoder_callback,
      &runtime.cb_ctx_a);
    runtime.callback_b = callback_ex(
      impl_->pi_handle,
      static_cast<unsigned>(runtime.config.enc_b_gpio),
      PI_EITHER_EDGE,
      &Impl::encoder_callback,
      &runtime.cb_ctx_b);

    if (runtime.callback_a < 0 || runtime.callback_b < 0) {
      RCLCPP_ERROR(impl_->logger, "Failed to register encoder callbacks for motor %zu.", i);
      shutdown();
      return false;
    }

  }

  initialized_ = true;
  RCLCPP_INFO(impl_->logger, "Maxon GPIO driver initialized for %zu motors.", impl_->motors.size());
  return true;
#else
  (void)motor_configs;
  RCLCPP_ERROR(
    impl_->logger,
    "This build was compiled without pigpio support. Install pigpio dev headers/libs and rebuild.");
  return false;
#endif
}

void MaxonGpioDriver::shutdown()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  if (impl_->pi_handle >= 0) {
    stop_all_motors();
    impl_->cancel_callbacks();
    pigpio_stop(impl_->pi_handle);
    impl_->pi_handle = -1;
  }
  impl_->motors.clear();
#endif
  initialized_ = false;
}

bool MaxonGpioDriver::is_initialized() const
{
  return initialized_;
}

void MaxonGpioDriver::stop_all_motors()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  if (!initialized_ || impl_->pi_handle < 0) {
    return;
  }

  for (const auto & motor : impl_->motors) {
    set_PWM_dutycycle(
      impl_->pi_handle, motor.config.pwm_gpio,
      neutral_pwm_duty(impl_->driver_config.pwm_range));
    if (motor.config.dir_gpio >= 0) {
      gpio_write(impl_->pi_handle, motor.config.dir_gpio, 0);
    }
  }
#endif
}

bool MaxonGpioDriver::set_motor_velocity(std::size_t motor_index, double wheel_velocity_rad_s)
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  if (!initialized_ || impl_->pi_handle < 0 || motor_index >= impl_->motors.size()) {
    return false;
  }

  auto & motor = impl_->motors[motor_index];
  double signed_velocity = wheel_velocity_rad_s * motor.config.command_sign;
  const double max_rad = std::max(1e-6, impl_->driver_config.max_wheel_rad_per_sec);
  const double norm = std::clamp(signed_velocity / max_rad, -1.0, 1.0);
  const double center = static_cast<double>(impl_->driver_config.pwm_range) * 0.5;
  const double half_span = center;
  int duty = static_cast<int>(std::lround(center + norm * half_span));
  duty = std::clamp(duty, 0, impl_->driver_config.pwm_range);

  // Mantido apenas por compatibilidade de hardware antigo.
  if (motor.config.dir_gpio >= 0) {
    const int dir = signed_velocity >= 0.0 ? 1 : 0;
    (void)gpio_write(impl_->pi_handle, motor.config.dir_gpio, dir);
  }

  return set_PWM_dutycycle(impl_->pi_handle, motor.config.pwm_gpio, duty) == 0;
#else
  (void)motor_index;
  (void)wheel_velocity_rad_s;
  return false;
#endif
}

bool MaxonGpioDriver::read_motor_state(
  std::size_t motor_index,
  double period_sec,
  double & position_rad,
  double & velocity_rad_s)
{
  position_rad = 0.0;
  velocity_rad_s = 0.0;

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  if (!initialized_ || motor_index >= impl_->motors.size()) {
    return false;
  }

  auto & motor = impl_->motors[motor_index];
  const int64_t count_now = motor.encoder_count.load(std::memory_order_relaxed);
  const int64_t delta_count = count_now - motor.previous_count;
  motor.previous_count = count_now;

  const double delta_rad = static_cast<double>(delta_count) * impl_->rad_per_count * motor.config.feedback_sign;
  motor.position_rad += delta_rad;
  const double dt = std::max(1e-6, period_sec);
  velocity_rad_s = delta_rad / dt;
  position_rad = motor.position_rad;
  return true;
#else
  (void)motor_index;
  (void)period_sec;
  return false;
#endif
}

}  // namespace mobile_base_hardware
