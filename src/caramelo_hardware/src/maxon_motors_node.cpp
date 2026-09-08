#include "caramelo_hardware/maxon_motors_node.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <thread>

#include <dirent.h>
#include <pthread.h>
#include <sched.h>

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
#include <lgpio.h>
#endif

namespace mobile_base_hardware
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
// Base mecanum de 4 rodas com mapa de pinos vindo de uma PCB fixa.
constexpr std::size_t kMaxWheels = 4;
// Amostras lidas por rajada. Ler e RAMIFICAR em cada amostra rende 1.04 MHz
// porque o core espera a transacao PCIe voltar; em rajada varias ficam em voo e
// a taxa sobe para ~5.4 MHz (medido na Pi 5 em 2026-09-01).
constexpr std::size_t kSamplerBurst = 16;
// Periodo de publicacao do instantaneo (1 kHz): dez vezes o laco de controle,
// barato o bastante para nao atrapalhar a amostragem.
constexpr int64_t kSnapshotPeriodNs = 1000000;
// Frequencia dos pulsos servo para os ESCs (mesma dos 50Hz default do pigpio).
constexpr int kServoFrequencyHz = 50;

int clamp_int(int value, int min_v, int max_v)
{
	return std::min(std::max(value, min_v), max_v);
}

int64_t steady_now_ns()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

std::vector<int> MaxonMotorsNode::tids_do_processo()
{
	// Le /proc/self/task. E' a UNICA forma de achar a thread que o lgpio cria
	// para gerar os trens de pulso: a biblioteca nao devolve handle, nao nomeia
	// a thread e nao tem API de prioridade.
	std::vector<int> tids;
	DIR * d = ::opendir("/proc/self/task");
	if (d == nullptr) {
		return tids;
	}
	while (const dirent * e = ::readdir(d)) {
		const int tid = std::atoi(e->d_name);
		if (tid > 0) {
			tids.push_back(tid);
		}
	}
	::closedir(d);
	std::sort(tids.begin(), tids.end());
	return tids;
}

void MaxonMotorsNode::aplicar_prioridade_rt(const char * quem)
{
	const int prio = driver_config_.control_rt_priority;
	if (prio <= 0) {
		return;
	}
	sched_param sp{};
	sp.sched_priority = prio;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
		RCLCPP_WARN(
			get_logger(),
			"%s: sem permissao para SCHED_FIFO %d; seguindo em prioridade normal. "
			"Os pulsos servo podem ESTICAR sob carga (medido em 2026-07: o ramo de "
			"frente acelerava ~2x). Confira /etc/security/limits.d/99-realtime.conf "
			"(ver docs/raspberry_tempo_real.md).",
			quem, prio);
		return;
	}
	RCLCPP_INFO(get_logger(), "%s: SCHED_FIFO %d.", quem, prio);
}

MaxonMotorsNode::MaxonMotorsNode()
: Node("maxon_motors_node")
{
	// Best-effort de propriedade 1: este topico e' TELEMETRIA, consumido por SSH
	// e Wi-Fi. Com QoS reliable, um consumidor lento aplica contrapressao no
	// publish() — que roda na thread de controle sob chrt -f 50.
	velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
		"maxon/wheel_velocity", rclcpp::SensorDataQoS());
}

MaxonMotorsNode::~MaxonMotorsNode()
{
	shutdown_hardware();
}


bool MaxonMotorsNode::verificar_arme()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// A amostragem ainda nao subiu neste ponto (a thread comeca depois), entao a
	// verificacao le o RIO direto, com o mesmo decodificador.
	std::array<caramelo::ChannelMap, kMaxWheels> canais{};
	for (std::size_t i = 0; i < motors_.size() && i < kMaxWheels; ++i) {
		canais[i].a_bit = static_cast<uint8_t>(motors_[i].config.enc_a_gpio);
		canais[i].b_bit = static_cast<uint8_t>(motors_[i].config.enc_b_gpio);
		canais[i].sign = 1;
	}

	// Limiar: 0,05 volta de roda. Bem acima do ruido (o repouso mede +-1 count)
	// e bem abaixo de qualquer arranque de verdade, que passa de 1 volta.
	const int64_t kLimiteCounts =
		static_cast<int64_t>(0.05 * std::max(1.0, driver_config_.encoder_counts_per_wheel_rev));

	for (int tentativa = 1; tentativa <= 3; ++tentativa) {
		caramelo::QuadratureDecoder<kMaxWheels> dec(
			canais, static_cast<uint32_t>(std::max(1, driver_config_.encoder_stable_samples)));
		dec.reset(rio_.read_in());

		// Janela de observacao: 3 s.
		//
		// 400 ms NAO bastavam, e o motivo importa: o ESC nao reage na hora ao
		// pulso ruim. Medido em 10 subidas com janela de 400 ms, 2 ainda tiveram
		// roda disparando, e o verificador nao viu nada — o disparo acontecia
		// DEPOIS da janela. O firmware tem tempo de armacao proprio, entao entre
		// receber o pulso truncado e comecar a girar passa mais de meio segundo.
		// A janela precisa cobrir esse atraso, senao ela certifica um robo que vai
		// sair andando logo em seguida.
		uint32_t buf[kSamplerBurst];
		const int64_t fim = steady_now_ns() + 3000000000LL;
		while (steady_now_ns() < fim) {
			for (int rep = 0; rep < 16; ++rep) {
				rio_.read_burst(buf);
				for (std::size_t k = 0; k < kSamplerBurst; ++k) { dec.update(buf[k]); }
			}
		}

		bool alguma_girando = false;
		for (std::size_t i = 0; i < motors_.size(); ++i) {
			const int64_t giro = dec.count(i) < 0 ? -dec.count(i) : dec.count(i);
			if (giro > kLimiteCounts) {
				alguma_girando = true;
				RCLCPP_ERROR(
					get_logger(),
					"ARME (tentativa %d): roda %zu (GPIO %d) girou %.3f volta SEM COMANDO.",
					tentativa, i, motors_[i].config.pwm_gpio,
					static_cast<double>(dec.count(i)) /
						std::max(1.0, driver_config_.encoder_counts_per_wheel_rev));
			}
		}

		if (!alguma_girando) {
			RCLCPP_INFO(get_logger(), "ARME: as 4 rodas paradas (tentativa %d).", tentativa);
			if (tentativa > 1) {
				RCLCPP_WARN(get_logger(), "ARME: estabilizou na tentativa %d.", tentativa);
			}
			return true;
		}

		// Corta os pulsos: com o firmware novo, Ton=0 PARA o motor, e cortar nao
		// tem o risco de truncar que reprogramar tem. Depois re-arma do zero.
		for (std::size_t i = 0; i < motors_.size(); ++i) {
			lgTxServo(
				chip_handle_.load(), motors_[i].config.pwm_gpio, 0,
				kServoFrequencyHz, servo_offset_us(i), 0);
			motors_[i].last_pulse_us.store(-1, std::memory_order_relaxed);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(600));
		for (std::size_t i = 0; i < motors_.size(); ++i) {
			send_servo_pulse(i, neutral_pulse_width_us(), true);
			std::this_thread::sleep_for(std::chrono::milliseconds(25));
		}
	}
	return false;
#else
	return true;
#endif
}

bool MaxonMotorsNode::initialize(
	const MaxonDriverConfig & driver_config,
	const std::vector<MaxonMotorConfig> & motor_configs)
{
	shutdown_hardware();
	driver_config_ = driver_config;

	if (motor_configs.empty()) {
		RCLCPP_ERROR(get_logger(), "MaxonMotorsNode recebeu lista vazia de motores.");
		return false;
	}

	const double counts_per_rev = std::max(1.0, driver_config_.encoder_counts_per_wheel_rev);
	rad_per_count_ = kTwoPi / counts_per_rev;

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// Threads existentes ANTES de qualquer chamada ao lgpio. A diferenca depois
	// da inicializacao e' exatamente o que a biblioteca criou (ver o bloco
	// "TEMPO REAL SO PARA QUEM PRECISA", mais abaixo).
	const std::vector<int> tids_antes = tids_do_processo();

	// Abre o gpiochip do header de 40 pinos. Na Pi 5 as linhas ficam no RP1
	// (label "pinctrl-rp1"; o NUMERO muda com o kernel). -1 = varre pelos
	// labels; numero explicito no URDF (<param name="gpiochip_device">) pula
	// a deteccao.
	int chip_number = driver_config_.gpiochip_device;
	if (chip_number >= 0) {
		chip_handle_.store(lgGpiochipOpen(chip_number));
	} else {
		chip_handle_.store(-1);
		for (int c = 0; c < 16 && chip_handle_.load() < 0; ++c) {
			const int h = lgGpiochipOpen(c);
			if (h < 0) {
				continue;
			}
			lgChipInfo_t info;
			if (lgGpioGetChipInfo(h, &info) == LG_OKAY &&
				std::strstr(info.label, "rp1") != nullptr)
			{
				chip_handle_.store(h);
				chip_number = c;
			} else {
				lgGpiochipClose(h);
			}
		}
		if (chip_handle_.load() < 0) {
			// Pi 4 e anteriores: header no pinctrl-bcm*. Aceitamos, mas SO se o
			// label confirmar — cair no gpiochip0 as cegas e' perigoso na Pi 5,
			// onde ele e' o gpio-brcmstb interno do BCM2712 e as linhas 24/25
			// sao BT_RTS/BT_CTS. Um fallback errado nao "deixa de funcionar":
			// ele dirige pinos internos da placa achando que fala com os ESCs.
			// (Aconteceu na bancada de 2026-09-01 com um script de teste.)
			for (int c = 0; c < 16 && chip_handle_.load() < 0; ++c) {
				const int h = lgGpiochipOpen(c);
				if (h < 0) {
					continue;
				}
				lgChipInfo_t info;
				if (lgGpioGetChipInfo(h, &info) == LG_OKAY &&
					std::strstr(info.label, "pinctrl-bcm") != nullptr)
				{
					chip_handle_.store(h);
					chip_number = c;
				} else {
					lgGpiochipClose(h);
				}
			}
		}
		if (chip_handle_.load() < 0) {
			RCLCPP_ERROR(
				get_logger(),
				"Nenhum gpiochip com label de header (pinctrl-rp1 ou pinctrl-bcm*) "
				"foi encontrado. RECUSANDO abrir um chip as cegas. Rode 'gpiodetect' "
				"e informe <param name=\"gpiochip_device\"> no ros2_control do URDF.");
			return false;
		}
	}
	if (chip_handle_.load() < 0) {
		RCLCPP_ERROR(
			get_logger(),
			"lgGpiochipOpen falhou com codigo %d (chip %d). Confira permissao de "
			"/dev/gpiochip* (grupo gpio) e o parametro gpiochip_device.",
			chip_handle_.load(),
			chip_number);
		return false;
	}

	if (motor_configs.size() != kMaxWheels) {
		RCLCPP_ERROR(
			get_logger(),
			"Este driver e' da base mecanum de %zu rodas (o mapa de pinos vem de uma "
			"PCB fixa); recebeu %zu.",
			kMaxWheels, motor_configs.size());
		return false;
	}

	motors_.clear();
	motors_.resize(motor_configs.size());
	last_snap_ = EncoderSnapshot{};
	have_last_snap_ = false;

	for (std::size_t i = 0; i < motor_configs.size(); ++i) {
		auto & motor = motors_[i];
		motor.config = motor_configs[i];

		// Rodas ESQUERDAS (GPIO 17/24) montadas espelhadas: comando e feedback
		// invertidos. ATENCAO: amarrado ao NUMERO do pino — remapear pinos exige
		// revisar aqui.
		if (motor.config.pwm_gpio == 17 || motor.config.pwm_gpio == 24) {
			motor.config.command_sign = -1.0;
			motor.config.feedback_sign = -1.0;
		}

		if (lgGpioClaimOutput(chip_handle_.load(), 0, motor.config.pwm_gpio, 0) < 0) {
			RCLCPP_ERROR(
				get_logger(),
				"lgGpioClaimOutput falhou para PWM GPIO %d.",
				motor.config.pwm_gpio);
			shutdown_hardware();
			return false;
		}
		if (!send_servo_pulse(i, neutral_pulse_width_us(), true)) {
			RCLCPP_ERROR(
				get_logger(),
				"lgTxServo inicial (neutro) falhou para PWM GPIO %d.",
				motor.config.pwm_gpio);
			shutdown_hardware();
			return false;
		}

		// Espaca o INICIO de cada trem de pulsos em mais de um periodo de 20 ms.
		//
		// MEDIDO em 2026-09-02: subindo o bringup com as rodas suspensas, rodas
		// giravam SOZINHAS ate 0,86 volta durante a inicializacao, sem nenhum
		// comando (os logs mostram apenas 1500 us). O padrao delatou a causa: as
		// rodas que se moviam eram sempre os indices PARES ou sempre os IMPARES,
		// que e' exatamente como os offsets escalonados (indice x 5 ms) se
		// agrupam dentro do periodo de 20 ms — pares em 0/10 ms, impares em
		// 5/15 ms, meio periodo de diferenca.
		//
		// Iniciando os quatro canais em sequencia rapida, o PRIMEIRO pulso de
		// parte deles sai truncado ou esticado, dependendo de onde a chamada cai
		// na fase do periodo. Um pulso fora da banda neutra e' um comando de
		// aceleracao para o ESC — e no chao isso e' o robo saindo andando.
		//
		// Esperar um periodo inteiro entre os inicios faz cada canal comecar com
		// a fase limpa, sem interagir com o anterior.
		std::this_thread::sleep_for(std::chrono::milliseconds(25));
		}

	// TEMPO REAL SO PARA QUEM PRECISA.
	//
	// O lgpio gera os trens de pulso servo por SOFTWARE, numa thread propria que
	// ele cria internamente. Se essa thread for preemptada, o pulso ESTICA — e
	// pulso esticado e' comando de velocidade errado (medido em 27-29/07: o ramo
	// de frente acelerava ~2x sob carga). Era por isso que o launch subia o
	// processo inteiro com "chrt -f 50".
	//
	// O problema e' que "o processo inteiro" inclui as nove threads de DDS, que
	// passavam a preemptar o EKF e o resto do sistema em prioridade de tempo
	// real. Aqui elevamos so' as threads do lgpio, achadas por diferenca de
	// /proc/self/task, e o laco de controle (que sobe a si mesmo em
	// control_loop). O DDS fica em prioridade normal, que e' onde ele deve estar.
	{
		const std::vector<int> tids_depois = tids_do_processo();
		lgpio_tids_.clear();
		std::set_difference(
			tids_depois.begin(), tids_depois.end(),
			tids_antes.begin(), tids_antes.end(),
			std::back_inserter(lgpio_tids_));

		// Teto de sanidade: esperamos 1, no maximo 2. Se apareceu mais que isso,
		// alguma outra parte do processo criou thread na mesma janela e nao da'
		// para saber qual e' qual — nesse caso NAO elevamos nada e dizemos por
		// que, em vez de promover uma thread aleatoria a tempo real.
		if (lgpio_tids_.size() > 2) {
			RCLCPP_WARN(
				get_logger(),
				"%zu threads novas durante a init do lgpio (esperado 1-2); nao da' "
				"para distinguir a thread de tx. NENHUMA foi elevada a tempo real — "
				"os pulsos podem esticar sob carga.",
				lgpio_tids_.size());
			lgpio_tids_.clear();
		} else if (driver_config_.control_rt_priority > 0) {
			// Prioridade: pwm_tx_priority quando pedida (acima do amostrador),
			// senao a mesma do laco de controle (comportamento anterior).
			const int prio = (driver_config_.pwm_tx_cpu >= 0 &&
				driver_config_.pwm_tx_priority > 0)
				? driver_config_.pwm_tx_priority
				: driver_config_.control_rt_priority;
			sched_param sp{};
			sp.sched_priority = prio;
			for (const int tid : lgpio_tids_) {
				// AFINIDADE PRIMEIRO, prioridade depois: assim a thread nunca chega
				// a rodar em prioridade alta num core que recebe interrupcao.
				if (driver_config_.pwm_tx_cpu >= 0) {
					cpu_set_t set;
					CPU_ZERO(&set);
					CPU_SET(driver_config_.pwm_tx_cpu, &set);
					if (::sched_setaffinity(tid, sizeof(set), &set) != 0) {
						RCLCPP_WARN(
							get_logger(),
							"nao consegui prender a thread %d do lgpio no core %d (%s). "
							"Ela segue em qualquer core — inclusive o que recebe as "
							"interrupcoes, onde o pulso ESTICA sob rajada de rede.",
							tid, driver_config_.pwm_tx_cpu, std::strerror(errno));
					} else {
						RCLCPP_INFO(
							get_logger(),
							"thread %d do lgpio (tx dos pulsos) presa no core %d.",
							tid, driver_config_.pwm_tx_cpu);
					}
				}
				// sched_setscheduler aceita TID como pid na semantica do Linux.
				if (::sched_setscheduler(tid, SCHED_FIFO, &sp) != 0) {
					RCLCPP_WARN(
						get_logger(),
						"nao consegui por a thread %d do lgpio em SCHED_FIFO %d (%s).",
						tid, prio, std::strerror(errno));
				} else {
					RCLCPP_INFO(
						get_logger(), "thread %d do lgpio (tx dos pulsos): SCHED_FIFO %d.",
						tid, prio);
				}
			}
			if (lgpio_tids_.empty()) {
				RCLCPP_WARN(
					get_logger(),
					"o lgpio nao criou thread nova na inicializacao. Se os pulsos "
					"esticarem sob carga, a thread de tx nasce depois e precisa ser "
					"elevada em outro ponto.");
			}
		}
	}

	// Amostragem do encoder: mapeia o RIO do RP1 e configura as 8 linhas como
	// entrada com pull-up e Schmitt. Isso NAO usa lgpio — e' MMIO direto, e por
	// isso convive com o lgpio segurando as linhas de PWM.
	{
		const std::string err = rio_.open_device();
		if (!err.empty()) {
			RCLCPP_ERROR(get_logger(), "Amostragem do encoder indisponivel: %s", err.c_str());
			shutdown_hardware();
			return false;
		}
		for (const auto & motor : motors_) {
			rio_.configure_input(static_cast<unsigned>(motor.config.enc_a_gpio), true, true);
			rio_.configure_input(static_cast<unsigned>(motor.config.enc_b_gpio), true, true);
		}
	}

	// ARME VERIFICADO: confere se alguma roda saiu girando sozinha e corrige.
	//
	// POR QUE: iniciar o trem de pulsos exige chamar lgTxServo, e essa chamada
	// cancela e reinicia o trem. Se ela cair DENTRO do pulso alto, trunca o Ton,
	// e pulso truncado cai na banda de RE do firmware, que arma re sem tempo de
	// armacao (ver send_servo_pulse). Nao da' para arrancar o trem sem essa
	// chamada, entao o risco e' inevitavel — o que da' para fazer e' VERIFICAR.
	//
	// Medido em 2026-09-02, com as rodas suspensas: em 12 subidas do bringup,
	// 2 tiveram roda girando sozinha, uma delas 2,4 voltas, e numa outra a roda
	// ainda girava a 2,4 rad/s (o piso do ESC) quando medida. No chao isso e' o
	// robo saindo andando ao ligar.
	//
	// A verificacao so' existe aqui, na inicializacao: em operacao normal roda
	// girando sem comando e' o robo sendo ARRASTADO A MAO, que e' um caso de uso
	// legitimo e nao pode ser confundido com falha.
	RCLCPP_INFO(get_logger(), "ARME: verificando se alguma roda saiu girando...");
	if (!verificar_arme()) {
		RCLCPP_FATAL(
			get_logger(),
			"Nao consegui armar os ESCs em estado parado. RECUSANDO iniciar: subir "
			"assim significaria entregar o robo com roda girando sozinha.");
		shutdown_hardware();
		return false;
	}

	diag_msg_.data.assign(motors_.size(), 0.0);
	diag_divisor_ = 0;
	health_.store(static_cast<int>(Health::Ok), std::memory_order_relaxed);

	last_update_time_ = now();
	partida_ns_ = steady_now_ns();
	guarda_rearme_ns_ = 0;
	guarda_tentativas_ = 0;
	guarda_ativa_ = true;
	pulsos_cortados_ = false;
	last_cycle_ns_.store(steady_now_ns());
	initialized_.store(true, std::memory_order_release);

	sampler_running_.store(true);
	sampler_thread_ = std::thread(&MaxonMotorsNode::sampler_loop, this);

	control_thread_running_.store(true);
	control_thread_ = std::thread(&MaxonMotorsNode::control_loop, this);
	failsafe_thread_running_.store(true);
	failsafe_thread_ = std::thread(&MaxonMotorsNode::failsafe_loop, this);

	RCLCPP_INFO(
		get_logger(),
		"MaxonMotorsNode inicializado: %zu motores, PWM por lgpio no gpiochip%d, "
		"encoder em quadratura x4 por amostragem do RIO (filtro de %d amostras), "
		"failsafe ativo.",
		motors_.size(),
		chip_number,
		driver_config_.encoder_stable_samples);
	return true;
#else
	(void)motor_configs;
	RCLCPP_ERROR(
		get_logger(),
		"Backend lgpio nao foi habilitado neste build de caramelo_hardware. "
		"Instale liblgpio-dev na Raspberry e recompile o pacote.");
	return false;
#endif
}

void MaxonMotorsNode::shutdown_hardware()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	failsafe_thread_running_.store(false);
	if (failsafe_thread_.joinable()) {
		failsafe_thread_.join();
	}
	control_thread_running_.store(false);
	if (control_thread_.joinable()) {
		control_thread_.join();
	}
	// A thread de amostragem so' toca no mmap do RIO; e' joinable de verdade
	// (ao contrario da thread de alertas do lgpio, que nao era nossa e podia
	// ter callback em voo enquanto os vetores eram liberados).
	sampler_running_.store(false);
	if (sampler_thread_.joinable()) {
		sampler_thread_.join();
	}

	if (chip_handle_.load() >= 0) {
		// Shutdown ORDENADO (2026-07-27): neutro -> 120 ms no fio (>=5 pulsos,
		// o firmware ve o neutro de verdade) -> corta os pulsos (largura 0; o
		// firmware detecta a perda e PARA os motores) -> libera linhas.
		// A versao anterior cortava imediatamente apos o neutro: o neutro NUNCA
		// chegava ao fio.
		// Threads ja joinadas acima, entao ninguem mais concorre: usa a versao
		// sem lock para nao depender de reentrancia.
		stop_all_motors_locked();
		std::this_thread::sleep_for(std::chrono::milliseconds(120));
		for (std::size_t i = 0; i < motors_.size(); ++i) {
			if (driver_config_.esc_failsafe_cut_pulses) {
				lgTxServo(
					chip_handle_.load(), motors_[i].config.pwm_gpio, 0,
					kServoFrequencyHz, servo_offset_us(i), 0);
			}
			lgGpioFree(chip_handle_.load(), motors_[i].config.pwm_gpio);
		}
		lgGpiochipClose(chip_handle_.load());
		chip_handle_.store(-1);
	}
	motors_.clear();
	have_last_snap_ = false;
#endif
	initialized_.store(false, std::memory_order_release);
}

bool MaxonMotorsNode::is_initialized() const
{
	return initialized_.load(std::memory_order_acquire);
}

void MaxonMotorsNode::set_command_velocity(std::size_t motor_index, double wheel_velocity_rad_s)
{
	if (motor_index >= motors_.size()) {
		return;
	}
	motors_[motor_index].command_rad_s.store(wheel_velocity_rad_s);
	last_command_ns_.store(steady_now_ns(), std::memory_order_relaxed);
}

bool MaxonMotorsNode::get_velocity(std::size_t motor_index, double & velocity_rad_s) const
{
	if (motor_index >= motors_.size()) {
		return false;
	}
	velocity_rad_s = motors_[motor_index].velocity_rad_s.load();
	return true;
}

bool MaxonMotorsNode::get_feedback(
	std::size_t motor_index, double & position_rad, double & velocity_rad_s) const
{
	if (motor_index >= motors_.size()) {
		return false;
	}
	position_rad = motors_[motor_index].position_rad.load();
	velocity_rad_s = motors_[motor_index].velocity_rad_s.load();
	return true;
}

bool MaxonMotorsNode::send_servo_pulse(std::size_t motor_index, int pulse_us, bool force)
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	if (chip_handle_.load() < 0 || motor_index >= motors_.size()) {
		return false;
	}
	auto & motor = motors_[motor_index];
	// So' reprograma o lgTxServo quando o valor MUDA: cada chamada cancela e
	// reinicia o trem de pulsos, e re-armar a 100 Hz um trem de 50 Hz caia
	// DENTRO do pulso alto e truncava Ton (pulso truncado cai na banda de RE
	// do firmware, que arma re' sem ARMING_TIME — "roda inverte sozinha").
	if (!force && motor.last_pulse_us.load(std::memory_order_relaxed) == pulse_us) {
		return true;
	}
	// Toda reprogramacao e' um evento de RISCO (pode truncar o Ton e virar re),
	// entao nenhuma pode acontecer sem rastro — mas o rastro fica atras de um
	// parametro (log_pwm_reprogram), e nao ligado o tempo todo. Este RCLCPP_INFO
	// sai de dentro do hw_mutex_, na thread do laco de 100 Hz: ele chama
	// get_clock() (mutex interno do rclcpp) e aloca no caminho de formatacao.
	// Foi decisivo para achar o disparo de partida e continua disponivel, mas em
	// operacao normal ele so' adiciona latencia ao caminho critico.
	if (driver_config_.log_pwm_reprogram) {
		RCLCPP_INFO(
			get_logger(), "lgTxServo REPROGRAMA GPIO%d: %d -> %d us (force=%d)",
			motor.config.pwm_gpio, motor.last_pulse_us.load(std::memory_order_relaxed),
			pulse_us, force ? 1 : 0);
	}
	const int rc = lgTxServo(
		chip_handle_.load(), motor.config.pwm_gpio, pulse_us,
		kServoFrequencyHz, servo_offset_us(motor_index), 0);
	if (rc < 0) {
		// NUNCA falhar em silencio: pulso congelado sem log era o mecanismo do
		// "mandei parar e nao para" quando uma chamada falhava.
		health_.store(static_cast<int>(Health::Degradado), std::memory_order_relaxed);
		RCLCPP_ERROR_THROTTLE(
			get_logger(), *get_clock(), 1000,
			"lgTxServo falhou (rc=%d) no GPIO %d (pulso %d us).",
			rc, motor.config.pwm_gpio, pulse_us);
		return false;
	}
	motor.last_pulse_us.store(pulse_us, std::memory_order_relaxed);
	return true;
#else
	(void)motor_index;
	(void)pulse_us;
	(void)force;
	return false;
#endif
}

void MaxonMotorsNode::stop_all_motors()
{
	// Ate 2026-09-01 esta funcao era chamada do on_activate e do on_deactivate
	// (thread do controller_manager) SEM segurar o mutex, colidindo com o
	// update_cycle que escreve nos mesmos motores. O custo do lock aqui e' no
	// maximo um ciclo de controle (10 ms), fora do caminho de 100 Hz.
	std::lock_guard<std::mutex> lock(hw_mutex_);
	stop_all_motors_locked();
}

void MaxonMotorsNode::stop_all_motors_locked()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	if (chip_handle_.load() < 0) {
		return;
	}

	if (pulsos_cortados_) {
		// Pulsos cortados pela guarda: cortado JA e' o estado parado, e mais
		// seguro que neutro (Ton=0 para o motor no firmware novo). Reprogramar
		// aqui re-armaria os ESCs que a guarda acabou de desarmar.
		for (std::size_t i = 0; i < motors_.size(); ++i) {
			motors_[i].command_rad_s.store(0.0);
			motors_[i].moving.store(false, std::memory_order_relaxed);
		}
		return;
	}

	for (std::size_t i = 0; i < motors_.size(); ++i) {
		motors_[i].command_rad_s.store(0.0);
		motors_[i].moving.store(false, std::memory_order_relaxed);
		// force = FALSE de proposito. Reprogramar o lgTxServo cancela e reinicia
		// o trem: se a chamada cair DENTRO do pulso alto, ela trunca o Ton, e
		// pulso truncado cai na banda de RE do firmware, que arma re sem tempo de
		// armacao (ver comentario em send_servo_pulse). Se o canal JA esta em
		// neutro, reprogramar nao melhora nada e so' expoe a esse risco.
		//
		// Medido em 2026-09-02: com force=true aqui, subidas do bringup faziam
		// rodas girarem sozinhas ate 0,86 volta, e numa delas a roda ainda estava
		// girando a 2,4 rad/s (o piso do ESC) no momento da medicao — ou seja, o
		// proprio "parar" era o que mandava andar.
		send_servo_pulse(i, neutral_pulse_width_us(), false);
	}
#endif
}

void MaxonMotorsNode::control_loop()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// Este laco fala com o GPIO a 100 Hz e nao pode ficar esperando o
	// escalonador. Ate 2026-09-08 ele herdava tempo real do "chrt -f 50" que o
	// launch aplicava ao processo inteiro; agora sobe a si mesmo, para que o
	// resto do processo (DDS acima de tudo) NAO suba junto.
	aplicar_prioridade_rt("control_loop");

	// Relogio ABSOLUTO (sleep_until): o sleep_for antigo derivava com a carga
	// e o periodo real do ciclo esticava sob CPU alta.
	auto next_wake = std::chrono::steady_clock::now();
	while (control_thread_running_.load()) {
		update_cycle();
		next_wake += std::chrono::milliseconds(10);
		const auto now_tp = std::chrono::steady_clock::now();
		if (next_wake < now_tp) {
			next_wake = now_tp + std::chrono::milliseconds(10);
		}
		std::this_thread::sleep_until(next_wake);
	}
#endif
}

void MaxonMotorsNode::failsafe_loop()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// Trabalho minimo e SEM o mutex do update_cycle (um holder travado e'
	// exatamente o cenario vigiado). Tenta prioridade RT — sem privilegio,
	// segue em SCHED_OTHER (ainda util: quase nunca preemptada, quase nao roda).
	sched_param sp{};
	sp.sched_priority = 10;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
		RCLCPP_WARN_ONCE(
			get_logger(),
			"failsafe: sem permissao para SCHED_FIFO (ver docs/raspberry_tempo_real.md); "
			"seguindo em SCHED_OTHER.");
	}

	bool pulses_cut = false;
	while (failsafe_thread_running_.load()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		if (!initialized_.load(std::memory_order_acquire) || chip_handle_.load() < 0) {
			continue;
		}
		const int64_t stale_ns = steady_now_ns() - last_cycle_ns_.load();
		if (stale_ns < 250LL * 1000 * 1000) {
			pulses_cut = false;
			continue;
		}
		if (stale_ns < 1000LL * 1000 * 1000) {
			// GPIO PRIMEIRO, log DEPOIS. O RCLCPP_*_THROTTLE chama get_clock(),
			// que toma mutex interno do rclcpp e pode alocar: fazer isso antes do
			// GPIO seria inversao de prioridade exatamente na thread que existe
			// para sobreviver a um sistema travado.
			for (std::size_t i = 0; i < motors_.size(); ++i) {
				lgTxServo(
					chip_handle_.load(), motors_[i].config.pwm_gpio,
					neutral_pulse_width_us(), kServoFrequencyHz,
					servo_offset_us(i), 0);
				motors_[i].last_pulse_us.store(neutral_pulse_width_us(), std::memory_order_relaxed);
			}
			RCLCPP_ERROR_THROTTLE(
				get_logger(), *get_clock(), 1000,
				"FAILSAFE: control_loop parado ha %.0f ms — neutro forcado.",
				stale_ns / 1e6);
		} else if (!pulses_cut) {
			// Travado de verdade: corta os pulsos — o firmware dos ESCs detecta
			// a perda de sinal e PARA os motores (~500 ms).
			health_.store(static_cast<int>(Health::Morto), std::memory_order_relaxed);
			for (std::size_t i = 0; i < motors_.size(); ++i) {
				// GATE DE SEGURANCA: cortar os pulsos so' e' o estado seguro com o
				// firmware novo dos ESCs, em que Ton=0 PARA o motor. No firmware
				// antigo, perda de PWM e' lida como RE MAXIMA — nesse caso o estado
				// seguro e' segurar neutro para sempre, nunca cortar.
				if (driver_config_.esc_failsafe_cut_pulses) {
					lgTxServo(
						chip_handle_.load(), motors_[i].config.pwm_gpio, 0,
						kServoFrequencyHz, servo_offset_us(i), 0);
				} else {
					lgTxServo(
						chip_handle_.load(), motors_[i].config.pwm_gpio,
						neutral_pulse_width_us(), kServoFrequencyHz, servo_offset_us(i), 0);
				}
				motors_[i].last_pulse_us.store(-1, std::memory_order_relaxed);
			}
			RCLCPP_FATAL(
				get_logger(),
				"FAILSAFE: control_loop parado ha %.1f s — %s.",
				stale_ns / 1e9,
				driver_config_.esc_failsafe_cut_pulses
					? "pulsos CORTADOS (firmware dos ESCs para os motores)"
					: "neutro sustentado (corte desabilitado por esc_failsafe_cut_pulses)");
			pulses_cut = true;
		}
	}
#endif
}

bool MaxonMotorsNode::read_encoder_snapshot(EncoderSnapshot & out) const
{
	// Leitura seqlock: o escritor incrementa a sequencia antes e depois de
	// gravar. Se as duas leituras derem o mesmo valor PAR, o que copiamos no
	// meio e' coerente. Sem lock, sem bloquear a thread de amostragem, ~50 ns.
	for (int tentativa = 0; tentativa < 8; ++tentativa) {
		const uint32_t s1 = snap_seq_.load(std::memory_order_acquire);
		if ((s1 & 1u) != 0u) {
			continue;  // escrita em andamento
		}
		out = snap_;
		std::atomic_thread_fence(std::memory_order_acquire);
		if (snap_seq_.load(std::memory_order_relaxed) == s1) {
			return out.t_mono_ns != 0;
		}
	}
	return false;
}

void MaxonMotorsNode::sampler_loop()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// POLITICA DE ESCALONAMENTO — aprendida na marra em 2026-09-01.
	//
	// Esta thread gira em laco fechado consumindo um core inteiro. Com
	// SCHED_FIFO 80 e SEM afinidade, ela pode ser colocada em qualquer core e
	// preempta tudo que estiver la; combinada com kernel.sched_rt_runtime_us=-1
	// (que precisamos para nao levar 50 ms de blackout por segundo), o resultado
	// foi a maquina ficar inutilizavel: os spawners do ros2_control nunca
	// conseguiram falar com o controller_manager e o load subiu para 14.
	//
	// Regra: prioridade de tempo real SO com a thread PRESA num core. Sem
	// afinidade, roda em prioridade normal — o que custa pouco, porque a taxa
	// medida foi a mesma (5.43 MHz) com e sem RT; o RT compra latencia de pior
	// caso, nao vazao. O ideal e' esse core estar isolado (isolcpus).
	bool preso_em_um_core = false;
	if (driver_config_.sampler_cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(driver_config_.sampler_cpu, &set);
		if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0) {
			preso_em_um_core = true;
		} else {
			RCLCPP_WARN(
				get_logger(), "Afinidade da thread de amostragem no core %d falhou.",
				driver_config_.sampler_cpu);
		}
	}

	if (!preso_em_um_core) {
		RCLCPP_WARN(
			get_logger(),
			"Thread de amostragem SEM afinidade: rodando em prioridade normal de "
			"proposito. Uma thread SCHED_FIFO em laco fechado sem core proprio "
			"trava a maquina. Defina <param name=\"sampler_cpu\"> (com isolcpus "
			"nesse core) para habilitar tempo real.");
	} else {
		// Prioridade ACIMA do laco de controle (que sobe com chrt -f 50 no
		// launch): perder amostra de encoder e' pior que atrasar um ciclo de PWM.
		sched_param sp{};
		sp.sched_priority = 80;
		if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
			RCLCPP_WARN(
				get_logger(),
				"SCHED_FIFO 80 na thread de amostragem falhou; seguindo em prioridade "
				"normal (confira /etc/security/limits.d/99-realtime.conf).");
		} else {
			// Com kernel.sched_rt_runtime_us no padrao (950000/1000000), uma
			// thread FIFO que usa 100% da CPU leva um BLACKOUT de 50 ms A CADA
			// SEGUNDO — medido: pior intervalo 49.99 ms com throttling, 16 us sem.
			// Isso vira perda de contagem em rajada, diagnosticada como "encoder
			// ruim". O contador de transicoes ilegais denuncia quando acontece.
			FILE * f = std::fopen("/proc/sys/kernel/sched_rt_runtime_us", "r");
			if (f != nullptr) {
				long v = 0;
				if (std::fscanf(f, "%ld", &v) == 1 && v > 0) {
					RCLCPP_WARN(
						get_logger(),
						"kernel.sched_rt_runtime_us=%ld: a thread de amostragem vai levar "
						"~%.0f ms/s de blackout. Ver tools/setup_pi.sh.",
						v, (1000000.0 - static_cast<double>(v)) / 1000.0);
				}
				std::fclose(f);
			}
		}
	}

	std::array<caramelo::ChannelMap, kMaxWheels> canais{};
	for (std::size_t i = 0; i < motors_.size() && i < kMaxWheels; ++i) {
		canais[i].a_bit = static_cast<uint8_t>(motors_[i].config.enc_a_gpio);
		canais[i].b_bit = static_cast<uint8_t>(motors_[i].config.enc_b_gpio);
		canais[i].sign = static_cast<int8_t>(motors_[i].config.encoder_sign < 0.0 ? -1 : 1);
	}
	caramelo::QuadratureDecoder<kMaxWheels> dec(
		canais, static_cast<uint32_t>(std::max(1, driver_config_.encoder_stable_samples)));
	dec.reset(rio_.read_in());

	uint32_t buf[kSamplerBurst];
	uint64_t amostras = 0;
	int64_t proxima_publicacao = steady_now_ns();

	while (sampler_running_.load(std::memory_order_relaxed)) {
		for (int rep = 0; rep < 16; ++rep) {
			rio_.read_burst(buf);
			for (std::size_t i = 0; i < kSamplerBurst; ++i) {
				dec.update(buf[i]);
			}
		}
		amostras += 16 * kSamplerBurst;

		const int64_t agora = steady_now_ns();
		if (agora >= proxima_publicacao) {
			proxima_publicacao = agora + kSnapshotPeriodNs;
			snap_seq_.fetch_add(1, std::memory_order_release);
			std::atomic_thread_fence(std::memory_order_release);
			for (std::size_t i = 0; i < kMaxWheels; ++i) {
				snap_.counts[i] = dec.count(i);
				snap_.illegal[i] = dec.illegal(i);
			}
			snap_.t_mono_ns = static_cast<uint64_t>(agora);
			snap_.samples = amostras;
			std::atomic_thread_fence(std::memory_order_release);
			snap_seq_.fetch_add(1, std::memory_order_release);
		}
	}
#endif
}


void MaxonMotorsNode::guarda_de_partida()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// POR QUE ESTA GUARDA EXISTE
	//
	// Iniciar o trem de pulsos exige chamar lgTxServo, e essa chamada cancela e
	// reinicia o trem: caindo dentro do pulso alto ela trunca o Ton, e pulso
	// truncado cai na banda de RE do firmware. Medido em dezenas de subidas com
	// as rodas suspensas: os logs das subidas que disparam sao IDENTICOS aos das
	// limpas — mesmas 4 reprogramacoes, mesmo verificador, nenhum failsafe. O
	// software faz exatamente a mesma coisa toda vez; a variavel e' a fase
	// interna do transmissor da lgpio quando o processo arranca, que nao esta
	// sob nosso controle. E as rodas afetadas sao sempre um PAR de offset
	// ({0,10} ms ou {5,15} ms), as duas fases separadas por meio periodo.
	//
	// Nao da' para evitar o disparo pela temporizacao. Da' para garantir que ele
	// nao SOBREVIVA: aqui a roda girando com pulso neutro e' detectada e os
	// pulsos sao cortados (Ton=0 PARA o motor no firmware novo, e cortar nao tem
	// o risco de truncar que reprogramar tem).
	//
	// A guarda vale so' nos primeiros segundos: depois disso, roda girando sem
	// comando e' o robo sendo ARRASTADO A MAO, que e' caso de uso legitimo.
	const int64_t agora = steady_now_ns();
	if (!guarda_ativa_) {
		return;
	}

	// Re-arme pendente de uma deteccao anterior.
	if (guarda_rearme_ns_ != 0) {
		if (agora < guarda_rearme_ns_) {
			return;
		}
		guarda_rearme_ns_ = 0;
		pulsos_cortados_ = false;
		for (std::size_t i = 0; i < motors_.size(); ++i) {
			send_servo_pulse(i, neutral_pulse_width_us(), true);
			// Mesmo espacamento de 25 ms da inicializacao: re-armar os quatro
			// canais em rajada e' o evento que dispara a roda, e re-armar depois
			// de um disparo era exatamente onde a rajada acontecia.
			std::this_thread::sleep_for(std::chrono::milliseconds(25));
		}
		RCLCPP_WARN(get_logger(), "GUARDA DE PARTIDA: re-armado (tentativa %d).",
			guarda_tentativas_);
		return;
	}

	// Janela configuravel; 0 = vigia para SEMPRE. A premissa de "isto e'
	// fenomeno de partida" caiu em 2026-09-08 (disparo 220 s apos a init).
	if (driver_config_.guarda_janela_s > 0.0 &&
		agora - partida_ns_ >
			static_cast<int64_t>(driver_config_.guarda_janela_s * 1e9))
	{
		guarda_ativa_ = false;
		RCLCPP_INFO(get_logger(), "GUARDA DE PARTIDA: janela encerrada, robo estavel.");
		return;
	}

	for (std::size_t i = 0; i < motors_.size(); ++i) {
		const bool neutro =
			motors_[i].last_pulse_us.load(std::memory_order_relaxed) == neutral_pulse_width_us();
		const double v = motors_[i].velocity_rad_s.load();
		// 0,25 rad/s: bem acima do ruido (o repouso mede +-1 count) e baixo o
		// bastante para pegar o disparo antes de o motor chegar ao piso de
		// 2,43 rad/s, cortando ainda na rampa.
		if (!neutro || std::fabs(v) < 0.25) {
			continue;
		}
		// Disparo: pulso neutro e roda girando acima do ruido.
		//
		// MODO OBSERVACAO (guarda_corta = false): registra e NAO intervem. E' o
		// modo de diagnostico — cortar o pulso contem o sintoma, e enquanto a
		// causa nao for achada o dado de como o disparo EVOLUI sozinho vale mais
		// que a intervencao. O throttle evita encher o log durante um giro longo.
		if (!driver_config_.guarda_corta) {
			RCLCPP_ERROR_THROTTLE(
				get_logger(), *get_clock(), 200,
				"DISPARO OBSERVADO: roda %zu (GPIO %d) a %.2f rad/s com pulso NEUTRO "
				"(%d us programado). NAO intervindo (guarda_corta=false). "
				"t=%.1f s apos a init.",
				i, motors_[i].config.pwm_gpio, v,
				motors_[i].last_pulse_us.load(std::memory_order_relaxed),
				static_cast<double>(agora - partida_ns_) * 1e-9);
			continue;
		}

		++guarda_tentativas_;
		RCLCPP_FATAL(
			get_logger(),
			"GUARDA DE PARTIDA: roda %zu (GPIO %d) girando a %.2f rad/s com pulso NEUTRO. "
			"Cortando os pulsos das 4 rodas.",
			i, motors_[i].config.pwm_gpio, v);
		for (std::size_t k = 0; k < motors_.size(); ++k) {
			lgTxServo(
				chip_handle_.load(), motors_[k].config.pwm_gpio, 0,
				kServoFrequencyHz, servo_offset_us(k), 0);
			motors_[k].last_pulse_us.store(-1, std::memory_order_relaxed);
		}
		// Segura o corte: sem isto o update_cycle reprogramava tudo de volta
		// para neutro no ciclo seguinte (10 ms), e o ESC nunca chegava a
		// desarmar.
		pulsos_cortados_ = true;
		if (guarda_tentativas_ >= 4) {
			guarda_ativa_ = false;
			health_.store(static_cast<int>(Health::Morto), std::memory_order_relaxed);
			RCLCPP_FATAL(
				get_logger(),
				"GUARDA DE PARTIDA: 4 disparos seguidos. Deixando os pulsos CORTADOS e "
				"marcando o driver como morto — melhor um robo parado que um solto.");
			return;
		}
		// 1,5 s com os pulsos cortados antes de re-armar. 800 ms nao bastavam: o
		// firmware leva ~500 ms para parar o motor depois da perda de sinal
		// (TURNOFF_TIME_MAX), entao re-armar cedo pegava o ESC ainda girando e
		// encadeava novo disparo — medido, 3 disparos seguidos numa subida.
		guarda_rearme_ns_ = agora + 1500000000LL;
		return;
	}
#endif
}

void MaxonMotorsNode::update_cycle()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	bool publicar_diagnostico = false;
	bool publicar_rastro = false;
	{
	std::lock_guard<std::mutex> lock(hw_mutex_);

	if (!initialized_.load(std::memory_order_acquire) || chip_handle_.load() < 0) {
		return;
	}

	last_update_time_ = now();

	// Feedback vem do INSTANTANEO da thread de amostragem, com o dt do relogio
	// MONOTONICO da propria amostra.
	//
	// Nao usar o relogio ROS aqui: com o chrony configurado como manda o
	// docs/relogio_chrony.md (makestep 1.0 -1), a Pi da STEP no relogio sempre
	// que o offset passa de 1 s — e a Pi 5 desta bancada nao tem bateria de RTC,
	// entao isso acontece de verdade a cada boot. Um step para tras faria
	// (now - last) negativo, o std::max(1e-6, ...) cravaria 1 us e a velocidade
	// publicada explodiria. O tempo ROS fica so' para stamp de mensagem.
	EncoderSnapshot snap;
	const bool tem_snap = read_encoder_snapshot(snap);
	double dt_enc = 0.0;
	if (tem_snap && have_last_snap_ && snap.t_mono_ns > last_snap_.t_mono_ns) {
		dt_enc = static_cast<double>(snap.t_mono_ns - last_snap_.t_mono_ns) * 1e-9;
	}

	// Watchdog de comando: se o write() do ros2_control parar de chegar,
	// comanda neutro em vez de congelar o ultimo PWM.
	const int64_t last_cmd_ns = last_command_ns_.load(std::memory_order_relaxed);
	const int64_t now_ns = steady_now_ns();
	const bool command_stale =
		(last_cmd_ns == 0) ||
		((now_ns - last_cmd_ns) >
			static_cast<int64_t>(driver_config_.command_timeout_s * 1e9));

	bool alguma_roda_ativa = false;
	for (std::size_t i = 0; i < motors_.size(); ++i) {
		auto & motor = motors_[i];
		if (tem_snap && have_last_snap_) {
			const int64_t delta_count = snap.counts[i] - last_snap_.counts[i];
			const double delta_rad =
				static_cast<double>(delta_count) * rad_per_count_ * motor.config.feedback_sign;
			motor.position_rad.store(motor.position_rad.load() + delta_rad);
			if (dt_enc > 1e-6) {
				motor.velocity_rad_s.store(delta_rad / dt_enc);
			}
		}
		diag_msg_.data[i] = motor.velocity_rad_s.load();

		const double cmd_signed = command_stale
			? 0.0
			: motor.command_rad_s.load() * motor.config.command_sign;
		const int pulse_us = velocity_to_pulse_width_us(
			cmd_signed, motor.moving.load(std::memory_order_relaxed),
			static_cast<int>(std::lround(motor.config.pulse_offset_us)));
		motor.moving.store(pulse_us != neutral_pulse_width_us(), std::memory_order_relaxed);
		// Guarda o pulso desta roda para o rastro consolidado (abaixo do laco):
		// logar dentro do laco com THROTTLE colapsa as quatro rodas numa mensagem
		// so', porque o throttle e' por ponto de chamada, nao por roda.
		diag_pulse_[i] = pulse_us;
		diag_cmd_[i] = cmd_signed;
		// NAO reprograma enquanto a guarda de partida esta com os pulsos
		// CORTADOS esperando o ESC desarmar.
		//
		// BUG CORRIGIDO EM 2026-09-08: este send era incondicional. A guarda
		// cortava os pulsos (Ton=0) e marcava last_pulse_us = -1 para segurar o
		// corte por 1,5 s — o tempo que o firmware leva para parar o motor
		// (TURNOFF_TIME_MAX ~500 ms). Mas guarda_rearme_ns_ so' bloqueava a
		// PROPRIA guarda: 10 ms depois este laco via last_pulse_us == -1 !=
		// neutro e reprogramava os quatro canais de volta para neutro. O corte
		// durava um ciclo em vez de 1500, e a recuperacao ainda saia como uma
		// RAJADA de quatro reprogramacoes sem espacamento — que e' exatamente o
		// evento que dispara a roda. A guarda estava alimentando o problema que
		// existe para conter.
		if (!pulsos_cortados_) {
			send_servo_pulse(i, pulse_us, false);
		}
		if (pulse_us != neutral_pulse_width_us()) {
			alguma_roda_ativa = true;
		}
	}

	if (tem_snap) {
		last_snap_ = snap;
		have_last_snap_ = true;
	}

	guarda_de_partida();

	last_cycle_ns_.store(now_ns, std::memory_order_relaxed);
	publicar_diagnostico = (++diag_divisor_ % 5) == 0;

	publicar_rastro = driver_config_.log_pulse_trace && alguma_roda_ativa;
	}  // solta o lock ANTES de publicar

	// Publicacao decimada para 20 Hz e fora do lock. Antes: a 100 Hz, com a
	// mensagem alocada por ciclo, DENTRO do mutex — sob SCHED_FIFO 50 o malloc
	// e a contrapressao do DDS entravam no caminho critico. O proprio
	// docs/calibracao_odometria.md ja dizia para usar media de varios segundos
	// porque o topico e' ruidoso a 100 Hz: ninguem precisava daquela taxa.
	if (publicar_diagnostico) {
		velocity_pub_->publish(diag_msg_);
	}

	// Rastro consolidado do mapa comando -> pulso, 1 Hz. As QUATRO rodas na
	// mesma linha, para dar para comparar os ramos de uma vez (um THROTTLE
	// dentro do laco colapsaria as quatro numa mensagem so', porque o throttle
	// e' por ponto de chamada, nao por roda).
	//
	// Duas correcoes de 2026-09-08:
	//  - Fica atras de log_pulse_trace, e nao ligado o tempo todo.
	//  - A condicao era "!command_stale", que NUNCA e' falsa: o write() do
	//    ros2_control chama set_command_velocity() para as 4 juntas a 100 Hz
	//    independente de haver comando, entao last_command_ns_ esta sempre
	//    fresco. O gate pretendido ("so' com algum comando ativo") nunca fechava
	//    e a linha saia a 1 Hz para sempre. Agora o gate e' o que ele dizia ser:
	//    alguma roda fora do neutro.
	//  - Sai FORA do hw_mutex_. Antes era emitido com o lock na mao, dentro do
	//    laco de 100 Hz.
	// diag_pulse_/diag_cmd_ so' sao tocados por esta thread, entao le-los depois
	// de soltar o lock e' seguro.
	if (publicar_rastro) {
		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 1000,
			"pulsos: m0/GPIO%d %s %dus | m1/GPIO%d %s %dus | m2/GPIO%d %s %dus | m3/GPIO%d %s %dus",
			motors_[0].config.pwm_gpio, diag_cmd_[0] > 0.0 ? "FRENTE" : "RE___", diag_pulse_[0],
			motors_[1].config.pwm_gpio, diag_cmd_[1] > 0.0 ? "FRENTE" : "RE___", diag_pulse_[1],
			motors_[2].config.pwm_gpio, diag_cmd_[2] > 0.0 ? "FRENTE" : "RE___", diag_pulse_[2],
			motors_[3].config.pwm_gpio, diag_cmd_[3] > 0.0 ? "FRENTE" : "RE___", diag_pulse_[3]);
	}
#endif
}

int MaxonMotorsNode::velocity_to_pulse_width_us(
	double wheel_velocity_rad_s, bool currently_moving, int pulse_offset_us) const
{
	// GUARDA CRITICA: o ros2_control inicializa os comandos das juntas como NaN
	// ate o controlador ativar. NaN aqui passava pelas comparacoes (todas
	// falsas), virava lround(NaN) e o clamp final cravava 1000us = RE MAXIMA:
	// os 4 ESCs recebiam ~8s de re total durante o carregamento do bringup
	// (flagrado com monitor pigpio nos GPIOs 17/23/24/25 em 2026-07-18).
	if (!std::isfinite(wheel_velocity_rad_s)) {
		return neutral_pulse_width_us();
	}
	// O firmware do ESC (B-G431B-ESC1) roda controle de velocidade em MALHA
	// FECHADA (FOC + halls) e interpreta o pulso de forma AFIM com um PISO.
	// speed_min/max do firmware <-> min/max_wheel_rad_per_sec do URDF: manter
	// SEMPRE casados com o binario gravado nos 4 ESCs (2026-07-27: 650 rpm ->
	// piso 2.43 rad/s de roda).
	const double floor_rad = std::max(0.0, driver_config_.min_wheel_rad_per_sec);
	const double max_rad = std::max(floor_rad + 1e-6, driver_config_.max_wheel_rad_per_sec);
	const double magnitude = std::abs(wheel_velocity_rad_s);

	// Politica "mais proximo executavel" COM HISTERESE (2026-07-27): o ESC nao
	// gira abaixo do piso. LIGAR exige |cmd| >= piso; DESLIGAR so' abaixo de
	// 0.4*piso. Sem histerese, comandos perto do limiar oscilavam 0 <-> piso a
	// 100 Hz (chattering) — pior desde que o piso dobrou (650 rpm).
	const double on_threshold = currently_moving ? 0.4 * floor_rad : floor_rad;
	if (magnitude < std::max(1e-9, on_threshold)) {
		return neutral_pulse_width_us();
	}

	const double clamped = std::clamp(magnitude, floor_rad, max_rad);
	const double norm = (clamped - floor_rad) / (max_rad - floor_rad);

	// Margem de partida: pulso EXATAMENTE no limiar (1540/1460us) fica na
	// fronteira de arme do firmware e, com a tolerancia do oscilador de cada
	// ESC (ate ~+-22us @1500us), a placa pode ler o pulso abaixo do limiar e
	// nao partir. 30us de margem garante partida mesmo na pior placa medida.
	// 2026-08-03: a reta agora ANCORA em piso+margem (1570/1430us) casada com
	// ESC_TON_MAP_MIN/ESC_TREV_MAP_MIN do firmware — antes a margem era so um
	// clamp e virava +307rpm de referencia no piso (comandado 650, rodava 957).
	// Mudar a margem/ancora la = mudar aqui.
	constexpr int kMargemPartidaUs = 30;
	// Piso da margem: com trim NEGATIVO a ancora nao pode escorregar de volta
	// para dentro da banda morta (ancora < 1540/>1460 = ESC nao parte).
	constexpr int kMargemMinimaUs = 10;

	// Trim por ramo (ver .hpp): positivo = ramo mais rapido. Desloca o mapa E
	// a ancora do clamp juntos — no piso de operacao o pulso fica cravado no
	// clamp inferior, entao trim so' no valor computado seria engolido.
	// 2026-09-01: esta funcao foi reconstruida apos o merge 101ad74, que
	// concatenou os dois lados do conflito e deixou o pacote SEM COMPILAR
	// (pulse_us redeclarado, lo/hi inexistentes). As duas intencoes sao
	// ORTOGONAIS e ambas valem: a ancora em piso+margem vem de 1ca3e21 (dev) e
	// o trim por ramo vem de 16df821 (rasp_5). ATENCAO: o trim de 8us foi
	// calibrado contra a ancora ANTIGA (1540, 26.09us por rad/s); com a ancora
	// nova (1570, 24.39us por rad/s) a inclinacao mudou -6.5% e o VALOR
	// NUMERICO precisa ser reconfirmado no teste reto de 3,4m.
	if (wheel_velocity_rad_s > 0.0) {
		const int trim = static_cast<int>(std::lround(driver_config_.pulse_trim_forward_us));
		// offset da placa: soma nos DOIS ramos (compensa o zero do ESC), ao
		// contrario do trim, que e' por ramo.
		const int off = pulse_offset_us;
		const double pulse_f =
			static_cast<double>(MaxonDriverConfig::kPulseUsForwardMin + kMargemPartidaUs) +
			norm * static_cast<double>(
				MaxonDriverConfig::kPulseUsForwardMax - MaxonDriverConfig::kPulseUsForwardMin -
				kMargemPartidaUs) +
			static_cast<double>(trim) + static_cast<double>(off);
		const int lo = clamp_int(
			MaxonDriverConfig::kPulseUsForwardMin + kMargemPartidaUs + trim + off,
			MaxonDriverConfig::kPulseUsForwardMin + kMargemMinimaUs,
			MaxonDriverConfig::kPulseUsForwardMax);
		const int pulse_us = static_cast<int>(std::lround(pulse_f));
		return clamp_int(pulse_us, lo, MaxonDriverConfig::kPulseUsForwardMax);
	}

	// Re: o pulso DESCE com a velocidade, entao "mais rapido" = subtrair o trim.
	const int trim = static_cast<int>(std::lround(driver_config_.pulse_trim_reverse_us));
	const int off = pulse_offset_us;
	const double pulse_f =
		static_cast<double>(MaxonDriverConfig::kPulseUsReverseMin - kMargemPartidaUs) -
		norm * static_cast<double>(
			MaxonDriverConfig::kPulseUsReverseMin - kMargemPartidaUs -
			MaxonDriverConfig::kPulseUsReverseMax) -
		static_cast<double>(trim) + static_cast<double>(off);
	const int hi = clamp_int(
		MaxonDriverConfig::kPulseUsReverseMin - kMargemPartidaUs - trim + off,
		MaxonDriverConfig::kPulseUsReverseMax,
		MaxonDriverConfig::kPulseUsReverseMin - kMargemMinimaUs);
	const int pulse_us = static_cast<int>(std::lround(pulse_f));
	return clamp_int(pulse_us, MaxonDriverConfig::kPulseUsReverseMax, hi);
}

int MaxonMotorsNode::neutral_pulse_width_us() const
{
	return clamp_int(
		MaxonDriverConfig::kPulseUsNeutral,
		MaxonDriverConfig::kPulseUsNeutralMin,
		MaxonDriverConfig::kPulseUsNeutralMax);
}

}  // namespace mobile_base_hardware
