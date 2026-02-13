#include "caramelo_hardware/mobile_base_hw_interface.hpp"

#include <array>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_base_hardware {

    namespace {
        constexpr std::array<int, 4> kDefaultPwmPins{18, 23, 24, 25};
        constexpr std::array<int, 4> kDefaultEncAPins{5, 12, 16, 20};
        constexpr std::array<int, 4> kDefaultEncBPins{6, 13, 19, 21};
    } // namespace

    // Esse seria o construtor da Classe.
    // Ele é necessário para criar o objeto do tipo MobileBaseHWInterface, que é o hardware interface do ros2_control.
    hardware_interface::CallbackReturn MobileBaseHWInterface::on_init
        (const hardware_interface::HardwareInfo & info)
        {
            // Igual para todos os futuros hardwares.
            if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
                return hardware_interface::CallbackReturn::ERROR;
            }

            // atributo privado para guardar as informações do hardware, que vem do arquivo de configuração (URDF)
            // Inerente do ros2_control
            info_ = info;

            if (info_.joints.empty()) {
                RCLCPP_ERROR(get_logger(), "Nenhuma junta foi declarada no ros2_control.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Parametros gerais do node/driver (com default seguro)
            driver_config_ = MaxonDriverConfig{};
            if (info_.hardware_parameters.count("pigpio_host")) {
                driver_config_.pigpio_host = info_.hardware_parameters.at("pigpio_host");
            }
            if (info_.hardware_parameters.count("pigpio_port")) {
                driver_config_.pigpio_port = info_.hardware_parameters.at("pigpio_port");
            }
            if (info_.hardware_parameters.count("pwm_frequency_hz")) {
                driver_config_.pwm_frequency_hz = std::stoi(info_.hardware_parameters.at("pwm_frequency_hz"));
            }
            if (info_.hardware_parameters.count("pwm_range")) {
                driver_config_.pwm_range = std::stoi(info_.hardware_parameters.at("pwm_range"));
            }
            if (info_.hardware_parameters.count("encoder_glitch_filter_us")) {
                driver_config_.encoder_glitch_filter_us = std::stoi(info_.hardware_parameters.at("encoder_glitch_filter_us"));
            }
            if (info_.hardware_parameters.count("update_period_ms")) {
                driver_config_.update_period_ms = std::stoi(info_.hardware_parameters.at("update_period_ms"));
            }
            if (info_.hardware_parameters.count("encoder_counts_per_wheel_rev")) {
                driver_config_.encoder_counts_per_wheel_rev = std::stod(info_.hardware_parameters.at("encoder_counts_per_wheel_rev"));
            }
            if (info_.hardware_parameters.count("max_wheel_rad_per_sec")) {
                driver_config_.max_wheel_rad_per_sec = std::stod(info_.hardware_parameters.at("max_wheel_rad_per_sec"));
            }

            joint_names_.clear();
            motor_configs_.clear();
            joint_names_.reserve(info_.joints.size());
            motor_configs_.reserve(info_.joints.size());

            for (std::size_t i = 0; i < info_.joints.size(); ++i) {
                const auto & joint = info_.joints[i];
                joint_names_.push_back(joint.name);

                MaxonMotorConfig cfg;
                if (i < kDefaultPwmPins.size()) {
                    cfg.pwm_gpio = kDefaultPwmPins[i];
                    cfg.enc_a_gpio = kDefaultEncAPins[i];
                    cfg.enc_b_gpio = kDefaultEncBPins[i];
                }

                if (joint.parameters.count("pwm_gpio")) {
                    cfg.pwm_gpio = std::stoi(joint.parameters.at("pwm_gpio"));
                }
                if (joint.parameters.count("dir_gpio")) {
                    cfg.dir_gpio = std::stoi(joint.parameters.at("dir_gpio"));
                }
                if (joint.parameters.count("enc_a_gpio")) {
                    cfg.enc_a_gpio = std::stoi(joint.parameters.at("enc_a_gpio"));
                }
                if (joint.parameters.count("enc_b_gpio")) {
                    cfg.enc_b_gpio = std::stoi(joint.parameters.at("enc_b_gpio"));
                }
                if (joint.parameters.count("command_sign")) {
                    cfg.command_sign = std::stod(joint.parameters.at("command_sign"));
                }
                if (joint.parameters.count("feedback_sign")) {
                    cfg.feedback_sign = std::stod(joint.parameters.at("feedback_sign"));
                }

                if (cfg.pwm_gpio < 0 || cfg.enc_a_gpio < 0 || cfg.enc_b_gpio < 0) {
                    RCLCPP_ERROR(get_logger(), "Pinos invalidos para a junta '%s'.", joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                motor_configs_.push_back(cfg);
            }

            //obrigatório retornar SUCCESS ou ERROR, para o ros2_control saber se a inicialização foi bem sucedida ou não.
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_configure
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state; // para evitar warning de variável não utilizada

            // Aqui é onde você pode configurar o hardware e abrir comunicação.
            driver_ = std::make_shared<MaxonMotorsNode>();
            if (!driver_->initialize(driver_config_, motor_configs_)) {
                RCLCPP_ERROR(get_logger(), "Falha ao inicializar MaxonMotorsNode.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            /** Adiciona o node do driver ao executor dedicado e inicia a thread de spin
             *  para processar os callbacks do driver em paralelo. */
            node_executor_.add_node(driver_);
            node_spin_thread_ = std::thread([this]() {node_executor_.spin();});

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_activate
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state;

            if (!driver_ || !driver_->is_initialized()) {
                RCLCPP_ERROR(get_logger(), "Driver nao inicializado.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            for (const auto & joint_name : joint_names_) {
                set_state(joint_name + "/position", 0.0);
                set_state(joint_name + "/velocity", 0.0);
                set_command(joint_name + "/velocity", 0.0);
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
                driver_->shutdown_hardware();
                node_executor_.remove_node(driver_);
                driver_.reset();
            }

            node_executor_.cancel();
            if (node_spin_thread_.joinable()) {
                node_spin_thread_.join();
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

            for (std::size_t i = 0; i < joint_names_.size(); ++i) {
                double position_unused = 0.0;
                double velocity = 0.0;
                if (!driver_->get_feedback(i, position_unused, velocity)) {
                    return hardware_interface::return_type::ERROR;
                }

                set_state(joint_names_[i] + "/velocity", velocity);
                // Para calcular a posição, basta integrar a velocidade ao longo do tempo.
                set_state(
                    joint_names_[i] + "/position",
                    get_state(joint_names_[i] + "/position") + velocity * period.seconds());
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

        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
            driver_->set_command_velocity(
                i, get_command(joint_names_[i] + "/velocity"));
        }

        return hardware_interface::return_type::OK;
    }

} // namespace mobile_base_hardware

// Essa macro é necessária para registrar a classe MobileBaseHWInterface como um plugin do tipo hardware_interface::SystemInterface, para que o ros2_control possa carregá-la dinamicamente.
#include "pluginlib/class_list_macros.hpp"

// Essa macro é necessária para registrar a classe MobileBaseHWInterface como um plugin do tipo hardware_interface::SystemInterface, para que o ros2_control possa carregá-la dinamicamente.
// provide -> namespace :: nome da classe, nome da classe pai (interface) :: tipo do plugin (hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHWInterface, hardware_interface::SystemInterface)
