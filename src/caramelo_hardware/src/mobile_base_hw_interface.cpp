#include "caramelo_hardware/mobile_base_hw_interface.hpp"

#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_base_hardware{

    // Esse seria o construtor da Classe.
    // Ele é necessário para criar o objeto do tipo MobileBaseHWInterface, que é o hardware interface do ros2_control.
    hardware_interface::CallbackReturn MobileBaseHWInterface::on_init
        (const hardware_interface::HardwareInfo & info)
        {
            if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (!parse_mobile_base_hw_info(info_, driver_config_, wheels_, get_logger())) {
                return hardware_interface::CallbackReturn::ERROR;
            }

            info_ = info;

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_configure
        (const rclcpp_lifecycle::State & previous_state)
        {
            
            (void)previous_state;

            std::vector<MaxonMotorConfig> motor_configs;
            motor_configs.reserve(wheels_.size());
            for (const auto & wheel : wheels_) {
                motor_configs.push_back(wheel.motor_config);
            }

            driver_ = std::make_shared<MaxonGpioDriver>();
            if (!driver_->initialize(driver_config_, motor_configs, get_logger())) {
                RCLCPP_ERROR(get_logger(), "Failed to configure Maxon GPIO driver.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_activate
        (const rclcpp_lifecycle::State & previous_state)
        {
        
        (void)previous_state;
        if (!driver_ || !driver_->is_initialized()) {
            RCLCPP_ERROR(get_logger(), "Cannot activate hardware: driver not initialized.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (const auto & wheel : wheels_) {
            set_state(wheel.joint_name + "/" + hardware_interface::HW_IF_POSITION, 0.0);
            set_state(wheel.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
            set_command(wheel.joint_name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
        }

        driver_->stop_all_motors();
        return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_deactivate
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state;

            if (driver_) {
                driver_->stop_all_motors();
            }

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::return_type MobileBaseHWInterface::read
        (const rclcpp::Time & time, const rclcpp::Duration & period)
        {
            (void)time;

            if (!driver_ || !driver_->is_initialized()) {
                return hardware_interface::return_type::ERROR;
            }

            for (std::size_t i = 0; i < wheels_.size(); ++i) {
                double position = 0.0;
                double velocity = 0.0;
                if (!driver_->read_motor_state(i, period.seconds(), position, velocity)) {
                    RCLCPP_ERROR(get_logger(), "Failed to read state from motor index %zu.", i);
                    return hardware_interface::return_type::ERROR;
                }

                const auto & joint_name = wheels_[i].joint_name;
                set_state(joint_name + "/" + hardware_interface::HW_IF_POSITION, position);
                set_state(joint_name + "/" + hardware_interface::HW_IF_VELOCITY, velocity);
            }

            return hardware_interface::return_type::OK;
        }

    hardware_interface::return_type MobileBaseHWInterface::write
        (const rclcpp::Time & time, const rclcpp::Duration & period)
        {
            (void)time;
            (void)period;

            if (!driver_ || !driver_->is_initialized()) {
                return hardware_interface::return_type::ERROR;
            }

            for (std::size_t i = 0; i < wheels_.size(); ++i) {
                const auto command = get_command(wheels_[i].joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
                if (!driver_->set_motor_velocity(i, command)) {
                    RCLCPP_ERROR(get_logger(), "Failed to write command to motor index %zu.", i);
                    return hardware_interface::return_type::ERROR;
                }
            }

            return hardware_interface::return_type::OK;
        }

}  // namespace mobile_base_hardware

// Essa macro e necessaria para registrar a classe MobileBaseHWInterface como um plugin
// do tipo hardware_interface::SystemInterface, para que o ros2_control possa carrega-la
// dinamicamente.
#include "pluginlib/class_list_macros.hpp"

// Essa macro e necessaria para registrar a classe MobileBaseHWInterface como um plugin
// do tipo hardware_interface::SystemInterface, para que o ros2_control possa carrega-la
// dinamicamente.
// provide -> namespace :: nome da classe, nome da classe pai (interface) :: tipo do plugin
// (hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHWInterface, hardware_interface::SystemInterface)
