#ifndef MOBILE_BASE_HW_INTERFACE_HPP
#define MOBILE_BASE_HW_INTERFACE_HPP

#include <array>
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
            // DESTRUTOR EXPLICITO — nao e' cosmetico.
            //
            // Sem ele, o ros2_control_node ABORTAVA no encerramento. Rastro
            // capturado no bringup de 2026-09-08:
            //   ControllerManager::~ControllerManager()
            //     -> ResourceManager::~ResourceManager()
            //       -> ~MobileBaseHWInterface -> ~thread -> std::__terminate
            // O destrutor de std::thread chama std::terminate() se a thread ainda
            // for JOINABLE, e node_spin_thread_ so' era joinada no on_cleanup —
            // que nao roda quando o componente e' destruido a partir de active ou
            // inactive.
            //
            // A consequencia grave nao e' o abort: e' que node_spin_thread_ e'
            // declarado DEPOIS de driver_, logo destruido ANTES dele. O terminate
            // disparava antes de ~MaxonMotorsNode, entao shutdown_hardware()
            // NUNCA rodava — nada de neutro por 120 ms no fio, nada de cortar os
            // pulsos, e sobretudo nada de lgGpioFree. E' a causa do problema
            // conhecido "Ctrl-C nao mata os nos, ficam zumbis segurando GPIOs"
            // (docs/raspberry_tempo_real.md).
            ~MobileBaseHWInterface() override;

            // Lifecycle Nodes overrides 
            hardware_interface::CallbackReturn
                on_configure(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_activate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            // Caminho normal de desligamento do lifecycle. Sem este override, sair
            // de active/inactive ia direto para o destrutor.
            hardware_interface::CallbackReturn
                on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn
                on_error(const rclcpp_lifecycle::State & previous_state) override;


            // System Interface overrides
            hardware_interface::CallbackReturn
                on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

            // Leitura de qualquer sensor ou estado do hardware. Exemplo: posição, velocidade, corrente, etc. (manda para o controller manager)
            hardware_interface::return_type
                read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            // Escrita de comandos para o hardware. Exemplo: posição, velocidade, torque, etc. (recebe do controller manager)
            hardware_interface::return_type
                write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            /// Encerra o executor e junta a thread de spin. Idempotente: chamada
            /// pelo on_cleanup, pelo on_shutdown, pelo on_error e pelo destrutor.
            void parar_spin();
            /// Encerra o hardware e solta o driver. Idempotente.
            void soltar_driver();

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

                int front_left_motor_id_ = 0;
                int front_right_motor_id_ = 1;
                int back_left_motor_id_ = 2;
                int back_right_motor_id_ = 3;

                // Ultimo instantaneo de encoder consumido pelo read(). Cada consumidor
                // guarda o seu proprio "anterior": e' isso que tira o aliasing entre o
                // laco do driver e o do controller_manager.
                MaxonMotorsNode::EncoderSnapshot last_snap_{};
                bool have_last_snap_ = false;

                // Nomes das interfaces montados UMA vez. Antes eram concatenados a cada
                // read(): "front_left_wheel_joint/velocity" tem 31 caracteres, estoura o
                // SSO da libstdc++ (15) e ia para o heap — ~1200 malloc/s dentro de uma
                // thread que roda com chrt -f 50.
                std::array<std::string, 4> iface_velocity_{};
                std::array<std::string, 4> iface_position_{};

                // O driver so' pode ser reportado como morto DEPOIS de ter ficado
                // saudavel: sem isso um read() antes do on_activate mataria o
                // componente na subida.
                bool was_healthy_ = false;

    }; // class MobileBaseHWInterface

} //namespace mobile_base_hardware

#endif
