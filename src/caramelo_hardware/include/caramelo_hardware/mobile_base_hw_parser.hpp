#ifndef CARAMELO_HARDWARE__MOBILE_BASE_HW_PARSER_HPP_
#define CARAMELO_HARDWARE__MOBILE_BASE_HW_PARSER_HPP_

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/logger.hpp"

#include "caramelo_hardware/maxon_gpio_driver.hpp"

namespace mobile_base_hardware
{

    struct WheelData
    {
        std::string joint_name;
        MaxonMotorConfig motor_config;
    };

    bool parse_mobile_base_hw_info(
        const hardware_interface::HardwareInfo & info,
        MaxonDriverConfig & driver_config,
        std::vector<WheelData> & wheels,
        const rclcpp::Logger & logger);

}  // namespace mobile_base_hardware

#endif  // CARAMELO_HARDWARE__MOBILE_BASE_HW_PARSER_HPP_

