#ifndef MOBILE_BASE_HW_INTERFACE_HPP
#define MOBILE_BASE_HW_INTERFACE_HPP

#include <memory>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"

#include "caramelo_hardware/maxon_gpio_driver.hpp"
#include "caramelo_hardware/mobile_base_hw_parser.hpp"

namespace mobile_base_hardware {

    class MobileBaseHWInterface : public hardware_interface::SystemInterface {
        public:
            hardware_interface::CallbackReturn
                on_configure(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_activate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_deactivate(const rclcpp_lifecycle::State & previous_state) override;


            hardware_interface::CallbackReturn
                on_init(const hardware_interface::HardwareInfo & info) override;

            hardware_interface::return_type
                read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            hardware_interface::return_type
                write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::vector<WheelData> wheels_;
            MaxonDriverConfig driver_config_;
            std::shared_ptr<MaxonGpioDriver> driver_;


    }; // class MobileBaseHWInterface

} //namespace mobile_base_hardware

#endif
