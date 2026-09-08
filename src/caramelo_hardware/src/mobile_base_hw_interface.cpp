#include "caramelo_hardware/mobile_base_hw_interface.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_base_hardware {

    namespace {
        // Fiacao FISICA da PCB (placa e chicote ja fabricados; imutavel).
        // A nomenclatura aqui e' HONESTA: A e' o A fisico.
        //
        // Ate 2026-09-01 estes nomes estavam TROCADOS de proposito, porque a
        // decodificacao era 1x numa linha so' e o canal A fisico tinha quique na
        // descida (15304 descidas duplas em 15516 ciclos na BR); contar pelo B
        // era o conserto inteiro, e o "sentido" vinha do comando. Com quadratura
        // x4 por amostragem os DOIS canais sao usados, trocar os nomes passaria a
        // INVERTER o sentido decodificado, e o quique some no filtro de
        // permanencia (medido: 232+194 transicoes ilegais -> 0). O sentido agora
        // e' um parametro medido, nao um efeito colateral de nomenclatura.
        constexpr int kPwmFrontLeft = 17;
        constexpr int kEncAFrontLeft = 5;
        constexpr int kEncBFrontLeft = 6;

        constexpr int kPwmFrontRight = 23;
        constexpr int kEncAFrontRight = 27;
        constexpr int kEncBFrontRight = 22;

        constexpr int kPwmBackLeft = 24;
        constexpr int kEncABackLeft = 16;
        constexpr int kEncBBackLeft = 26;

        constexpr int kPwmBackRight = 25;
        constexpr int kEncABackRight = 20;
        constexpr int kEncBBackRight = 21;

        // Sinal do encoder em ESPACO DE PULSO, medido na bancada 2026-09-01:
        // cutucando cada roda com um pulso de "frente", as quatro contaram
        // NEGATIVO com sign=+1 na ordem A/B fisica. Girando a mao no sentido de
        // marcha a frente do robo, FL e BL contaram positivo e FR e BR negativo —
        // que e' exatamente o espelhamento mecanico das rodas esquerdas, tratado
        // depois por feedback_sign. Logo, -1 uniforme alinha o decodificador com
        // o espaco de pulso, preservando a convencao de junta que a odometria ja
        // usava.
        constexpr double kEncoderSign = -1.0;
    } // namespace

    MobileBaseHWInterface::~MobileBaseHWInterface()
    {
        // Ordem OBRIGATORIA: primeiro a thread, depois o hardware.
        //
        // node_spin_thread_ e' declarado DEPOIS de driver_, entao o compilador o
        // destroi ANTES. Se ele chegar joinable no ~thread, std::terminate()
        // aborta o processo e ~MaxonMotorsNode nunca roda — o desligamento
        // ordenado dos ESCs e o lgGpioFree ficam para tras. Fazer o trabalho
        // aqui, no CORPO do destrutor (que executa antes da destruicao dos
        // membros), e' o que garante os dois.
        //
        // Nada aqui pode lancar: excecao escapando de destrutor e' std::terminate
        // do mesmo jeito.
        try {
            parar_spin();
            soltar_driver();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "excecao no destrutor: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "excecao desconhecida no destrutor.");
        }
    }

    void MobileBaseHWInterface::parar_spin()
    {
        node_executor_.cancel();
        if (node_spin_thread_.joinable()) {
            node_spin_thread_.join();
        }
    }

    void MobileBaseHWInterface::soltar_driver()
    {
        if (!driver_) {
            return;
        }
        // shutdown_hardware() e' idempotente e faz o desligamento ORDENADO:
        // neutro -> 120 ms no fio -> corta os pulsos -> libera as linhas.
        driver_->shutdown_hardware();
        node_executor_.remove_node(driver_);
        driver_.reset();
        have_last_snap_ = false;
        was_healthy_ = false;
    }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_shutdown
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state;
            // Caminho normal de desligamento: solta tudo aqui, para o destrutor
            // nao ter trabalho nenhum. Sem este override, sair de active ou
            // inactive pulava direto para o destrutor.
            parar_spin();
            soltar_driver();
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_error
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state;
            // Mesmo tratamento do shutdown: um componente em erro que deixa a
            // thread viva volta a abortar no destrutor.
            parar_spin();
            soltar_driver();
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    // Esse seria o construtor da Classe.
    // Ele é necessário para criar o objeto do tipo MobileBaseHWInterface, que é o hardware interface do ros2_control.
    hardware_interface::CallbackReturn MobileBaseHWInterface::on_init
        (const hardware_interface::HardwareComponentInterfaceParams & params)
        {
            // Igual para todos os futuros hardwares.
            if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
                return hardware_interface::CallbackReturn::ERROR;
            }

            // atributo privado para guardar as informações do hardware, que vem do arquivo de configuração (URDF)
            // Inerente do ros2_control
            info_ = params.hardware_info;

            if (info_.joints.empty()) {
                RCLCPP_ERROR(get_logger(), "Nenhuma junta foi declarada no ros2_control.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Parametros gerais do node/driver (fixos no código)
            driver_config_ = MaxonDriverConfig{};
            // 2026-07-27: gpiochip_device agora e' LIDO DE VERDADE do URDF (antes
            // era so' documentado e ignorado — auditoria do port lgpio, bug #5).
            // -1 = auto-detecta pelo label "rp1" (Pi 5) com fallback gpiochip0.
            const auto chip_it = info_.hardware_parameters.find("gpiochip_device");
            if (chip_it != info_.hardware_parameters.end()) {
                try {
                    driver_config_.gpiochip_device = std::stoi(chip_it->second);
                } catch (const std::exception &) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Parametro 'gpiochip_device' invalido ('%s'); usando auto-deteccao.",
                        chip_it->second.c_str());
                }
            }

            // Mapa afim do ESC em rad/s de RODA (piso e escala cheia), configuravel
            // pelo URDF para permitir recalibracao sem recompilar. Ver
            // docs/esc_stm32_comportamento_e_riscos.md para a origem dos valores.
            const auto read_double_param =
                [this](const char * name, double & target) {
                    const auto it = info_.hardware_parameters.find(name);
                    if (it == info_.hardware_parameters.end()) {
                        return;
                    }
                    try {
                        target = std::stod(it->second);
                    } catch (const std::exception &) {
                        RCLCPP_WARN(
                            get_logger(),
                            "Parametro '%s' invalido ('%s'); mantendo %.3f.",
                            name, it->second.c_str(), target);
                    }
                };
            const auto read_int_param =
                [this](const char * name, int & target) {
                    const auto it = info_.hardware_parameters.find(name);
                    if (it == info_.hardware_parameters.end()) {
                        return;
                    }
                    try {
                        target = std::stoi(it->second);
                    } catch (const std::exception &) {
                        RCLCPP_WARN(
                            get_logger(),
                            "Parametro '%s' invalido ('%s'); mantendo %d.",
                            name, it->second.c_str(), target);
                    }
                };
            const auto read_bool_param =
                [this](const char * name, bool & target) {
                    const auto it = info_.hardware_parameters.find(name);
                    if (it == info_.hardware_parameters.end()) {
                        return;
                    }
                    const std::string & v = it->second;
                    target = (v == "true" || v == "True" || v == "1");
                };
            read_double_param("min_wheel_rad_per_sec", driver_config_.min_wheel_rad_per_sec);
            read_double_param("max_wheel_rad_per_sec", driver_config_.max_wheel_rad_per_sec);
            read_double_param("command_timeout_s", driver_config_.command_timeout_s);
            // Trim por ramo (us; +N = ramo mais rapido). Calibrado no chao
            // 2026-07-29: ramo de FRENTE ~8% lento vs mapa -> guinada de ~23
            // graus em 3,4m. Ver maxon_motors_node.hpp.
            read_double_param("pulse_trim_forward_us", driver_config_.pulse_trim_forward_us);
            read_double_param("pulse_trim_reverse_us", driver_config_.pulse_trim_reverse_us);
            // Encoder: 1024 ciclos por canal por volta do motor x gearbox 1:28 x
            // quadratura x4 = 114688 counts por volta de RODA. CONFIRMADO na
            // bancada 2026-09-01 girando cada roda 3 voltas a mao: 112.7k a 113.1k
            // counts/volta medidos (deficit compativel com erro de marcacao e, na
            // BR, ~1% de perda por glitch antes do filtro).
            // A decodificacao 1x anterior existia porque a contagem era por
            // EVENTO; por amostragem o custo independe da velocidade. Ver
            // quadrature_decoder.hpp.
            driver_config_.encoder_counts_per_wheel_rev = 1024.0 * 28.0 * 4.0;
            read_double_param(
                "encoder_counts_per_wheel_rev", driver_config_.encoder_counts_per_wheel_rev);
            read_int_param("encoder_stable_samples", driver_config_.encoder_stable_samples);
            read_int_param("sampler_cpu", driver_config_.sampler_cpu);
            // Prioridade de tempo real do laco de controle e da thread de tx do
            // lgpio. 0 = nao elevar. Ate 2026-09-08 o launch subia o PROCESSO
            // INTEIRO com "chrt -f 50" e as nove threads de DDS iam junto; ver
            // maxon_motors_node.hpp para a medicao que motivou a mudanca.
            read_int_param("control_rt_priority", driver_config_.control_rt_priority);
            // Instrumentacao do disparo de partida: cara em log, desligada por
            // default, religavel sem recompilar quando o problema voltar.
            read_bool_param("log_pwm_reprogram", driver_config_.log_pwm_reprogram);
            read_bool_param("log_pulse_trace", driver_config_.log_pulse_trace);
            // Gate de seguranca do failsafe. true = pode CORTAR os pulsos quando o
            // laco de controle morre; so' vale com o firmware novo dos ESCs, em que
            // Ton=0 PARA o motor. Com o firmware antigo, perda de PWM e' lida como
            // RE MAXIMA, e o estado seguro passa a ser neutro sustentado.
            {
                const auto it = info_.hardware_parameters.find("esc_failsafe_cut_pulses");
                if (it != info_.hardware_parameters.end()) {
                    const std::string v = it->second;
                    driver_config_.esc_failsafe_cut_pulses =
                        (v == "true" || v == "True" || v == "1");
                }
            }

            joint_names_.clear();
            motor_configs_.clear();
            joint_names_.reserve(info_.joints.size());
            motor_configs_.reserve(info_.joints.size());

            for (std::size_t i = 0; i < info_.joints.size(); ++i) {
                const auto & joint = info_.joints[i];
                joint_names_.push_back(joint.name);

                MaxonMotorConfig cfg;
                cfg.encoder_sign = kEncoderSign;
                if (joint.name == "front_left_wheel_joint") {
                    cfg.pwm_gpio = kPwmFrontLeft;
                    cfg.enc_a_gpio = kEncAFrontLeft;
                    cfg.enc_b_gpio = kEncBFrontLeft;
                } else if (joint.name == "front_right_wheel_joint") {
                    cfg.pwm_gpio = kPwmFrontRight;
                    cfg.enc_a_gpio = kEncAFrontRight;
                    cfg.enc_b_gpio = kEncBFrontRight;
                } else if (joint.name == "back_left_wheel_joint") {
                    cfg.pwm_gpio = kPwmBackLeft;
                    cfg.enc_a_gpio = kEncABackLeft;
                    cfg.enc_b_gpio = kEncBBackLeft;
                } else if (joint.name == "back_right_wheel_joint") {
                    cfg.pwm_gpio = kPwmBackRight;
                    cfg.enc_a_gpio = kEncABackRight;
                    cfg.enc_b_gpio = kEncBBackRight;
                }

                // Offset de pulso POR RODA, vindo do <param> da propria <joint>.
                // Compensa o zero de cada ESC (tolerancia do oscilador da placa).
                {
                    const auto it = joint.parameters.find("pulse_offset_us");
                    if (it != joint.parameters.end()) {
                        try {
                            cfg.pulse_offset_us = std::stod(it->second);
                        } catch (const std::exception &) {
                            RCLCPP_WARN(
                                get_logger(),
                                "pulse_offset_us invalido ('%s') na junta '%s'; usando 0.",
                                it->second.c_str(), joint.name.c_str());
                        }
                    }
                }

                if (cfg.pwm_gpio < 0 || cfg.enc_a_gpio < 0 || cfg.enc_b_gpio < 0) {
                    RCLCPP_ERROR(get_logger(), "Pinos invalidos para a junta '%s'.", joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                motor_configs_.push_back(cfg);
            }

            front_left_motor_id_ = 0;
            front_right_motor_id_ = 1;
            back_left_motor_id_ = 2;
            back_right_motor_id_ = 3;

            // Nomes de interface indexados pelo MOTOR, derivados da ordem real das
            // juntas no URDF. Antes o read()/write() usavam uma tabela fixa
            // (front_left=0, front_right=1, ...): reordenar as <joint> no xacro
            // mandava comando para a roda errada SEM nenhum erro.
            for (std::size_t i = 0; i < joint_names_.size() && i < iface_velocity_.size(); ++i) {
                iface_velocity_[i] = joint_names_[i] + "/velocity";
                iface_position_[i] = joint_names_[i] + "/position";
            }

            //obrigatório retornar SUCCESS ou ERROR, para o ros2_control saber se a inicialização foi bem sucedida ou não.
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_configure
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state; // para evitar warning de variável não utilizada

            // Defesa contra configure sem cleanup antes: sem isto, a thread de
            // spin anterior continuaria viva girando um executor cujo node acabou
            // de ser trocado, e o driver antigo seguiria com as linhas de GPIO na
            // mao — o proximo initialize() falharia com "Device or resource busy"
            // e o motivo nao apareceria em lugar nenhum.
            parar_spin();
            soltar_driver();

            // Aqui é onde você pode configurar o hardware e abrir comunicação.
            driver_ = std::make_shared<MaxonMotorsNode>();
            if (!driver_->initialize(driver_config_, motor_configs_)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Falha ao inicializar MaxonMotorsNode. Verifique /dev/gpiochip* (grupo gpio), liblgpio, GPIOs e alimentacao da base.");
                driver_->shutdown_hardware();
                driver_.reset();
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

            set_state("front_left_wheel_joint/velocity", 0.0);
            set_state("front_right_wheel_joint/velocity", 0.0);
            set_state("back_left_wheel_joint/velocity", 0.0);
            set_state("back_right_wheel_joint/velocity", 0.0);

            set_state("front_left_wheel_joint/position", 0.0);
            set_state("front_right_wheel_joint/position", 0.0);
            set_state("back_left_wheel_joint/position", 0.0);
            set_state("back_right_wheel_joint/position", 0.0);

            // Semente do instantaneo: o primeiro read() apos ativar nao pode ver como
            // "delta" tudo que o encoder contou enquanto o componente estava inativo.
            have_last_snap_ = false;
            if (driver_ && driver_->read_encoder_snapshot(last_snap_)) {
                have_last_snap_ = true;
            }

            // Comandos comecam como NaN no ros2_control ate o controlador ativar;
            // zera-os aqui para o write() nunca propagar NaN ao driver (NaN
            // virava pulso de 1000us = re maxima nos 4 ESCs durante o bringup).
            set_command("front_left_wheel_joint/velocity", 0.0);
            set_command("front_right_wheel_joint/velocity", 0.0);
            set_command("back_left_wheel_joint/velocity", 0.0);
            set_command("back_right_wheel_joint/velocity", 0.0);

            // Mapa junta -> motor em voz alta: ele e' derivado da ordem do URDF e
            // manda comando para uma roda especifica. Um erro aqui move a roda
            // errada sem nenhuma mensagem de erro.
            for (std::size_t i = 0; i < iface_velocity_.size(); ++i) {
                RCLCPP_INFO(
                    get_logger(), "motor %zu -> %s (PWM GPIO %d, enc A%d/B%d)",
                    i, iface_velocity_[i].c_str(),
                    (i < motor_configs_.size()) ? motor_configs_[i].pwm_gpio : -1,
                    (i < motor_configs_.size()) ? motor_configs_[i].enc_a_gpio : -1,
                    (i < motor_configs_.size()) ? motor_configs_[i].enc_b_gpio : -1);
            }

            // Guarda de null: o on_deactivate destruia o driver, entao um ciclo
            // inactive -> active derrubava o ros2_control_node inteiro aqui.
            if (driver_) {
                driver_->stop_all_motors();
            }
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_deactivate
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state;

            // on_deactivate PARA os motores mas NAO destroi o driver: a amostragem
            // do encoder precisa continuar viva para que o robo arrastado a mao
            // continue sendo contado. A liberacao do hardware e' do on_cleanup.
            if (driver_) {
                driver_->stop_all_motors();
            }
            have_last_snap_ = false;

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_cleanup
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state;

            parar_spin();
            soltar_driver();

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::return_type MobileBaseHWInterface::read
        (const rclcpp::Time & time, const rclcpp::Duration & period)
        {
            (void)time;
            (void)period;

            if (!driver_ || !driver_->is_initialized()) {
                return hardware_interface::return_type::OK;
            }

            const auto saude = driver_->health();
            if (saude == MaxonMotorsNode::Health::Ok) {
                was_healthy_ = true;
            } else if (saude == MaxonMotorsNode::Health::Morto && was_healthy_) {
                // Ate 2026-09-01 o read() devolvia OK com o driver morto: o
                // controller_manager nunca sabia, tudo aparecia verde em
                // 'ros2 control list_controllers' e o robo ficava surdo. DEACTIVATE
                // desativa o componente (e os controladores que usam as interfaces
                // dele) sem passar por on_error.
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Driver dos motores em estado MORTO (failsafe cortou os pulsos). "
                    "Desativando o componente.");
                return hardware_interface::return_type::DEACTIVATE;
            }

            // Feedback direto do instantaneo do encoder: delta de CONTAGENS desde a
            // leitura anterior e dt do relogio MONOTONICO da propria amostra.
            //
            // O caminho antigo (posicao integrada a 100 Hz no driver, diferenciada de
            // volta a 100 Hz aqui, em outro relogio) fazia alguns ciclos verem delta
            // zero e outros verem dois — a velocidade exportada oscilava entre 0 e ~2x
            // e o mecanum_drive_controller integrava esse ruido direto no /odom/wheel.
            MaxonMotorsNode::EncoderSnapshot snap;
            if (!driver_->read_encoder_snapshot(snap)) {
                return hardware_interface::return_type::OK;
            }
            if (!have_last_snap_) {
                last_snap_ = snap;
                have_last_snap_ = true;
                return hardware_interface::return_type::OK;
            }

            const double dt = (snap.t_mono_ns > last_snap_.t_mono_ns)
                ? static_cast<double>(snap.t_mono_ns - last_snap_.t_mono_ns) * 1e-9
                : 0.0;
            const double rad_por_count = driver_->rad_per_count();

            for (std::size_t i = 0; i < iface_velocity_.size(); ++i) {
                const int64_t delta_counts = snap.counts[i] - last_snap_.counts[i];
                const double delta_rad =
                    static_cast<double>(delta_counts) * rad_por_count * driver_->feedback_sign(i);

                // Sem deadband na leitura: o controlador integra a velocidade das rodas
                // para a odometria, e zerar velocidades pequenas acumula erro
                // sistematico no /odom/wheel. Deadband so' faz sentido no COMANDO.
                if (dt > 1e-6) {
                    set_state(iface_velocity_[i], delta_rad / dt);
                }
                set_state(iface_position_[i], get_state(iface_position_[i]) + delta_rad);
            }

            last_snap_ = snap;
            return hardware_interface::return_type::OK;
        }

    hardware_interface::return_type MobileBaseHWInterface::write
        (const rclcpp::Time & time, const rclcpp::Duration & period)
        {
            (void)time;
            (void)period;

            if (!driver_ || !driver_->is_initialized()) {
                return hardware_interface::return_type::OK;
            }

            for (std::size_t i = 0; i < iface_velocity_.size(); ++i) {
                const double cmd = get_command(iface_velocity_[i]);
                // NaN aqui nao e' teorico: o ros2_control inicializa os comandos
                // como NaN ate o controlador ativar, e o driver trata isso como
                // neutro. Mas NaN DEPOIS de ativo significa interface errada, e o
                // sintoma e' o robo simplesmente nao andar, sem erro nenhum.
                if (!std::isfinite(cmd)) {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Comando nao-finito na interface '%s' (motor %zu).",
                        iface_velocity_[i].c_str(), i);
                }
                driver_->set_command_velocity(i, cmd);
            }

            return hardware_interface::return_type::OK;
        }

} // namespace mobile_base_hardware

// Essa macro é necessária para registrar a classe MobileBaseHWInterface como um plugin do tipo hardware_interface::SystemInterface, para que o ros2_control possa carregá-la dinamicamente.
#include "pluginlib/class_list_macros.hpp"
// Essa macro é necessária para registrar a classe MobileBaseHWInterface como um plugin do tipo hardware_interface::SystemInterface, para que o ros2_control possa carregá-la dinamicamente.
// provide -> namespace :: nome da classe, nome da classe pai (interface) :: tipo do plugin (hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHWInterface, hardware_interface::SystemInterface)
