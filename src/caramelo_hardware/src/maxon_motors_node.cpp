#include "caramelo_hardware/maxon_motors_node.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <cstring>

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

		// A leitura do encoder NAO passa mais pelo lgpio: e' amostragem direta
		// do RIO do RP1 (ver sampler_loop). O lgpio fica so' com o PWM.
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

	diag_msg_.data.assign(motors_.size(), 0.0);
	diag_divisor_ = 0;
	health_.store(static_cast<int>(Health::Ok), std::memory_order_relaxed);

	last_update_time_ = now();
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

	for (std::size_t i = 0; i < motors_.size(); ++i) {
		motors_[i].command_rad_s.store(0.0);
		motors_[i].moving = false;
		send_servo_pulse(i, neutral_pulse_width_us(), true);
	}
#endif
}

void MaxonMotorsNode::control_loop()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
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

void MaxonMotorsNode::update_cycle()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	bool publicar_diagnostico = false;
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
		send_servo_pulse(i, pulse_us, false);
	}

	if (tem_snap) {
		last_snap_ = snap;
		have_last_snap_ = true;
	}

	last_cycle_ns_.store(now_ns, std::memory_order_relaxed);
	publicar_diagnostico = (++diag_divisor_ % 5) == 0;

	// Rastro consolidado do mapa comando -> pulso, 1 Hz, so' com algum comando
	// ativo. Mostra as QUATRO rodas na mesma linha, para dar para comparar os
	// ramos de uma vez.
	if (!command_stale) {
		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 1000,
			"pulsos: m0/GPIO%d %s %dus | m1/GPIO%d %s %dus | m2/GPIO%d %s %dus | m3/GPIO%d %s %dus",
			motors_[0].config.pwm_gpio, diag_cmd_[0] > 0 ? "FRENTE" : "RE___", diag_pulse_[0],
			motors_[1].config.pwm_gpio, diag_cmd_[1] > 0 ? "FRENTE" : "RE___", diag_pulse_[1],
			motors_[2].config.pwm_gpio, diag_cmd_[2] > 0 ? "FRENTE" : "RE___", diag_pulse_[2],
			motors_[3].config.pwm_gpio, diag_cmd_[3] > 0 ? "FRENTE" : "RE___", diag_pulse_[3]);
	}
	}  // solta o lock ANTES de publicar

	// Publicacao decimada para 20 Hz e fora do lock. Antes: a 100 Hz, com a
	// mensagem alocada por ciclo, DENTRO do mutex — sob SCHED_FIFO 50 o malloc
	// e a contrapressao do DDS entravam no caminho critico. O proprio
	// docs/calibracao_odometria.md ja dizia para usar media de varios segundos
	// porque o topico e' ruidoso a 100 Hz: ninguem precisava daquela taxa.
	if (publicar_diagnostico) {
		velocity_pub_->publish(diag_msg_);
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
