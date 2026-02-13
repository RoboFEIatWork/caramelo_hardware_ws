#include "caramelo_hardware/mobile_base_hw_parser.hpp"

#include <array>
#include <exception>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_base_hardware
{

namespace
{
constexpr std::array<int, 4> kDefaultPwmPins{18, 23, 24, 25};
constexpr std::array<int, 4> kDefaultEncAPins{5, 12, 16, 20};
constexpr std::array<int, 4> kDefaultEncBPins{6, 13, 19, 21};

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & interface_name)
{
  for (const auto & interface : interfaces) {
    if (interface.name == interface_name) {
      return true;
    }
  }
  return false;
}

bool read_int_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  int & output)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }
  try {
    output = std::stoi(it->second);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

bool read_double_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  double & output)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return false;
  }
  try {
    output = std::stod(it->second);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

std::string read_string_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return fallback;
  }
  return it->second;
}

bool validate_joint(
  const hardware_interface::ComponentInfo & joint,
  const rclcpp::Logger & logger)
{
  const bool has_velocity_cmd = has_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  const bool has_position_state = has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION);
  const bool has_velocity_state = has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  if (!has_velocity_cmd || !has_position_state || !has_velocity_state) {
    RCLCPP_ERROR(
      logger,
      "Joint '%s' must expose command interface 'velocity' and state interfaces 'position' + 'velocity'.",
      joint.name.c_str());
    return false;
  }
  return true;
}
}  // namespace

bool parse_mobile_base_hw_info(
  const hardware_interface::HardwareInfo & info,
  MaxonDriverConfig & driver_config,
  std::vector<WheelData> & wheels,
  const rclcpp::Logger & logger)
{
  if (info.joints.empty()) {
    RCLCPP_ERROR(logger, "No joints declared in ros2_control block.");
    return false;
  }

  driver_config = MaxonDriverConfig{};
  driver_config.pigpio_host = read_string_param(info.hardware_parameters, "pigpio_host", "");
  driver_config.pigpio_port = read_string_param(info.hardware_parameters, "pigpio_port", "");
  read_int_param(info.hardware_parameters, "pwm_frequency_hz", driver_config.pwm_frequency_hz);
  read_int_param(info.hardware_parameters, "pwm_range", driver_config.pwm_range);
  read_int_param(info.hardware_parameters, "encoder_glitch_filter_us", driver_config.encoder_glitch_filter_us);
  read_double_param(
    info.hardware_parameters, "encoder_counts_per_wheel_rev",
    driver_config.encoder_counts_per_wheel_rev);
  read_double_param(info.hardware_parameters, "max_wheel_rad_per_sec", driver_config.max_wheel_rad_per_sec);

  wheels.clear();
  wheels.reserve(info.joints.size());

  for (std::size_t i = 0; i < info.joints.size(); ++i) {
    const auto & joint = info.joints[i];
    if (!validate_joint(joint, logger)) {
      return false;
    }

    WheelData wheel;
    wheel.joint_name = joint.name;

    if (i < kDefaultPwmPins.size()) {
      wheel.motor_config.pwm_gpio = kDefaultPwmPins[i];
      wheel.motor_config.enc_a_gpio = kDefaultEncAPins[i];
      wheel.motor_config.enc_b_gpio = kDefaultEncBPins[i];
    }

    read_int_param(joint.parameters, "pwm_gpio", wheel.motor_config.pwm_gpio);
    read_int_param(joint.parameters, "dir_gpio", wheel.motor_config.dir_gpio);
    read_int_param(joint.parameters, "enc_a_gpio", wheel.motor_config.enc_a_gpio);
    read_int_param(joint.parameters, "enc_b_gpio", wheel.motor_config.enc_b_gpio);
    read_double_param(joint.parameters, "command_sign", wheel.motor_config.command_sign);
    read_double_param(joint.parameters, "feedback_sign", wheel.motor_config.feedback_sign);

    if (
      wheel.motor_config.pwm_gpio < 0 || wheel.motor_config.enc_a_gpio < 0 ||
      wheel.motor_config.enc_b_gpio < 0)
    {
      RCLCPP_ERROR(
        logger,
        "Joint '%s' has invalid pin configuration (pwm=%d, enc_a=%d, enc_b=%d).",
        joint.name.c_str(),
        wheel.motor_config.pwm_gpio,
        wheel.motor_config.enc_a_gpio,
        wheel.motor_config.enc_b_gpio);
      return false;
    }

    wheels.push_back(wheel);
  }

  RCLCPP_INFO(
    logger, "Parsed %zu joints for Maxon GPIO hardware interface.", wheels.size());
  return true;
}

}  // namespace mobile_base_hardware

