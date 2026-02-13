#ifndef MOBILE_BASE_HW_INTERFACE_HPP
#define MOBILE_BASE_HW_INTERFACE_HPP

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp" // inportado do ros2_control para cria um hw do tipo system interface
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "caramelo_hardware/maxon_motors_node.hpp" // importado o node/driver do motor

namespace mobile_base_hardware {

    class MobileBaseHWInterface : public hardware_interface::SystemInterface {
        //Para qualquer sistema os metodos publicos sempre serao os mesmos a esse (pode copiar e colar)
        public:
            // Lifecycle Nodes overrides 
            hardware_interface::CallbackReturn
                on_configure(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_activate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_deactivate(const rclcpp_lifecycle::State & previous_state) override;


            // System Interface overrides
            hardware_interface::CallbackReturn
                on_init(const hardware_interface::HardwareInfo & info) override;

            // Leitura de qualquer sensor ou estado do hardware. Exemplo: posição, velocidade, corrente, etc. (manda para o controller manager)
            hardware_interface::return_type
                read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            // Escrita de comandos para o hardware. Exemplo: posição, velocidade, torque, etc. (recebe do controller manager)
            hardware_interface::return_type
                write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            // sharedpointer para o node/driver do motor, para que possa ser usado em todas as funções da classe
            std::shared_ptr<MaxonMotorsNode> driver_;

            // configuracao geral do driver (host/porta pigpio, pwm range, etc)
            MaxonDriverConfig driver_config_;

            // nomes das juntas e configuracao por roda
            std::vector<std::string> joint_names_;
            std::vector<MaxonMotorConfig> motor_configs_;

            // executor dedicado para rodar o node dos motores em paralelo
            rclcpp::executors::SingleThreadedExecutor node_executor_;
            std::thread node_spin_thread_;

    }; // class MobileBaseHWInterface

} //namespace mobile_base_hardware

#endif
