#ifndef CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_
#define CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "caramelo_hardware/quadrature_decoder.hpp"
#include "caramelo_hardware/rp1_rio.hpp"

namespace mobile_base_hardware
{

struct MaxonMotorConfig
{
	int pwm_gpio = -1;
	// Canais A e B FISICOS do encoder. Desde 2026-09-01 os DOIS sao usados: a
	// decodificacao e' quadratura x4 por amostragem, entao o sentido e' MEDIDO.
	// Antes so' um canal era contado e o "sentido" vinha do ultimo comando —
	// razao pela qual arrastar o robo a mao era impossivel de contar.
	int enc_a_gpio = -1;
	int enc_b_gpio = -1;
	// Sinal do COMANDO: -1 nas rodas montadas espelhadas (esquerdas), para que
	// um pulso de "frente" mova o robo para frente.
	double command_sign = 1.0;
	// Sinal do FEEDBACK em espaco de junta.
	double feedback_sign = 1.0;
	// Deslocamento de pulso POR PLACA (us), somado nos DOIS ramos.
	//
	// NAO confundir com pulse_trim_*_us: o trim e' POR RAMO e assimetrico
	// (frente soma, re subtrai), para corrigir uma diferenca entre andar para
	// frente e para tras. Este offset e' SIMETRICO e existe para compensar o
	// zero de cada ESC: o oscilador de cada placa tem tolerancia de ~+-1.5%
	// (ate ~+-22 us em 1500 us), entao o pulso que a placa MEDE nao e' o que a
	// Pi ENVIA. Sintoma: a roda anda mais devagar num sentido e mais rapido no
	// outro, com erro de mesma magnitude e sinais opostos.
	//
	// Como calibrar: comande a mesma velocidade para frente e para tras e meca.
	// erro_us = (|w_re| - |w_frente|) / 2 * 24.39   (us por rad/s de roda)
	// Um valor POSITIVO significa que a placa le menos do que enviamos.
	//
	// PENDENTE: medido ~12 us no lado direito em 2026-09-01, mas a bateria do
	// robo morreu logo depois do teste — o dado esta contaminado por queda de
	// tensao sob carga e NAO foi aplicado. Recalibrar com bateria carregada
	// antes de sair do zero.
	double pulse_offset_us = 0.0;
	// Sinal do ENCODER: converte o sentido decodificado (A/B fisicos) para o
	// sentido positivo da roda. MEDIDO na bancada 2026-09-01 girando cada roda
	// a mao no sentido de marcha a frente: FL +1, FR -1, BL +1, BR -1.
	double encoder_sign = 1.0;
};

struct MaxonDriverConfig
{
	// /dev/gpiochipN do header de 40 pinos. -1 = auto-detecta pelo label
	// ("rp1" na Pi 5; fallback gpiochip0 nas Pi 4 e anteriores).
	// 2026-07-27: agora e' lido de verdade do URDF (<param name="gpiochip_device">).
	int gpiochip_device = -1;
	// 2026-07: bandas casadas com o firmware corrigido dos ESCs (banda morta
	// alargada p/ tolerar o oscilador +-1.5% das placas): forward >=1540us,
	// reverso <=1460us, neutro 1500us. NAO usar com firmware antigo (1520/1480).
	static constexpr int kPulseUsReverseMax = 1000;
	static constexpr int kPulseUsReverseMin = 1460;
	static constexpr int kPulseUsNeutralMin = 1460;
	static constexpr int kPulseUsNeutralMax = 1540;
	static constexpr int kPulseUsForwardMin = 1540;
	static constexpr int kPulseUsForwardMax = 2000;
	static constexpr int kPulseUsNeutral =
		(kPulseUsNeutralMin + kPulseUsNeutralMax) / 2;
	// Encoder em quadratura x4 por AMOSTRAGEM (2026-09-01): 1024 ciclos por
	// canal por volta de motor x gearbox 1:28 x 4 = 114688 counts por volta de
	// RODA. Confirmado na bancada girando cada roda 3 voltas a mao: 112.7k a
	// 113.1k counts/volta medidos, deficit compativel com erro de marcacao.
	// A contagem por EVENTO (que forcou o x1 anterior) foi abandonada: ver
	// quadrature_decoder.hpp para o argumento de custo.
	double encoder_counts_per_wheel_rev = 1024.0 * 28.0 * 4.0;
	// Amostras de permanencia exigidas para aceitar um estado (filtro de
	// glitch). A/B medido em 2026-09-01: 8 zera as transicoes ilegais causadas
	// pelo ruido das fases do motor no chicote do proprio encoder, sem perder
	// nenhuma borda legitima.
	int encoder_stable_samples = 8;
	// Core onde a thread de amostragem roda (-1 = sem afinidade). Ideal: um
	// core isolado por isolcpus.
	int sampler_cpu = -1;
	// Mapa AFIM do firmware Caramelo (B-G431B-ESC1 / MCSDK modificado):
	//   motor_rpm = speed_min + (pulso_us - 1540) * (speed_max - speed_min) / 460
	// speed_min/max vem do firmware (mc_parameters.c: speed_min_valueRPM) e o
	// equivalente em rad/s de RODA (gearbox 1:28) vem do URDF
	// (<param name="min/max_wheel_rad_per_sec">) — manter os DOIS casados com o
	// firmware gravado nos 4 ESCs. 2026-07-27: speed_min 650 rpm -> 2.43 rad/s.
	double min_wheel_rad_per_sec = 2.43;
	double max_wheel_rad_per_sec = 20.06;
	// Trim de calibracao POR RAMO (us; positivo = roda mais rapido naquele
	// ramo). Medido no chao 2026-07-29 (3m + trena, ida e volta): o ramo de
	// FRENTE rodava ~8% abaixo do previsto pelo mapa enquanto o de RE batia
	// exato -> robo guinava ~23 graus em 3,4m. Deficit equivalente: ~7,3us.
	// Valores vem do URDF (<param name="pulse_trim_forward/reverse_us">).
	double pulse_trim_forward_us = 0.0;
	double pulse_trim_reverse_us = 0.0;
	// Watchdog: sem set_command_velocity() por mais que isso -> PWM neutro
	// (protege contra morte do write()/controller_manager com PWM congelado).
	double command_timeout_s = 0.5;
	// Se o failsafe pode CORTAR os pulsos (largura 0) ou apenas segurar neutro.
	// Com o firmware novo dos ESCs, Ton=0 PARA o motor e cortar e' o estado mais
	// seguro. Com o firmware antigo, perda de PWM e' interpretada como RE
	// MAXIMA — por isso isto e' um parametro e nao uma constante: uma Pi com ESC
	// antigo nao pode herdar o comportamento novo em silencio.
	bool esc_failsafe_cut_pulses = true;
};

class MaxonMotorsNode : public rclcpp::Node
{
public:
	MaxonMotorsNode();
	~MaxonMotorsNode() override;

	bool initialize(
		const MaxonDriverConfig & driver_config,
		const std::vector<MaxonMotorConfig> & motor_configs);

	void shutdown_hardware();
	bool is_initialized() const;

	void set_command_velocity(std::size_t motor_index, double wheel_velocity_rad_s);
	bool get_velocity(std::size_t motor_index, double & velocity_rad_s) const;
	bool get_feedback(std::size_t motor_index, double & position_rad, double & velocity_rad_s) const;

	void stop_all_motors();


	/// Instantaneo coerente dos contadores do encoder, com o relogio MONOTONICO
	/// da amostra. Publicado pela thread de amostragem por seqlock.
	struct EncoderSnapshot
	{
		int64_t counts[4] = {0, 0, 0, 0};
		uint64_t illegal[4] = {0, 0, 0, 0};
		uint64_t t_mono_ns = 0;
		uint64_t samples = 0;
	};

	/// Le o instantaneo sem lock (seqlock). Devolve false se nao houver dado.
	///
	/// O instantaneo e' uma VISAO imutavel: cada consumidor (o update_cycle do
	/// driver e o read() do ros2_control) guarda o seu proprio "anterior" e
	/// calcula o delta. E' isso que elimina o aliasing entre os dois lacos de
	/// 100 Hz, em que um integrava a posicao e o outro a diferenciava de volta
	/// num relogio diferente — alguns ciclos viam delta zero e outros viam dois,
	/// e a velocidade exportada oscilava entre 0 e ~2x.
	bool read_encoder_snapshot(EncoderSnapshot & out) const;

	/// Radianos de roda por count do encoder (2*pi / counts_per_wheel_rev).
	double rad_per_count() const { return rad_per_count_; }

	/// Sinal de feedback em espaco de JUNTA para a roda i (+1/-1).
	double feedback_sign(std::size_t i) const
	{
		return (i < motors_.size()) ? motors_[i].config.feedback_sign : 1.0;
	}

	/// Saude do driver, para o read()/write() nao mentirem OK com o hardware
	/// morto. Ok = normal; Degradado = uma escrita de PWM falhou (logado com
	/// throttle); Morto = o failsafe cortou os pulsos ou o chip sumiu.
	enum class Health : int { Ok = 0, Degradado = 1, Morto = 2 };
	Health health() const { return static_cast<Health>(health_.load(std::memory_order_relaxed)); }

private:
	struct MotorRuntime
	{
		MotorRuntime() = default;

		MotorRuntime(const MotorRuntime & other)
		: config(other.config),
			position_rad(other.position_rad.load()),
			velocity_rad_s(other.velocity_rad_s.load()),
			command_rad_s(other.command_rad_s.load()),
			last_pulse_us(other.last_pulse_us.load()),
			moving(other.moving.load())
		{
		}

		MotorRuntime & operator=(const MotorRuntime & other)
		{
			if (this != &other) {
				config = other.config;
				position_rad.store(other.position_rad.load());
				velocity_rad_s.store(other.velocity_rad_s.load());
				command_rad_s.store(other.command_rad_s.load());
				last_pulse_us.store(other.last_pulse_us.load());
				moving.store(other.moving.load());
			}
			return *this;
		}

		MotorRuntime(MotorRuntime && other) noexcept
		: config(std::move(other.config)),
			position_rad(other.position_rad.load()),
			velocity_rad_s(other.velocity_rad_s.load()),
			command_rad_s(other.command_rad_s.load()),
			last_pulse_us(other.last_pulse_us.load()),
			moving(other.moving.load())
		{
		}

		MotorRuntime & operator=(MotorRuntime && other) noexcept
		{
			if (this != &other) {
				config = std::move(other.config);
				position_rad.store(other.position_rad.load());
				velocity_rad_s.store(other.velocity_rad_s.load());
				command_rad_s.store(other.command_rad_s.load());
				last_pulse_us.store(other.last_pulse_us.load());
				moving.store(other.moving.load());
			}
			return *this;
		}

		MaxonMotorConfig config;
		std::atomic<double> position_rad{0.0};
		std::atomic<double> velocity_rad_s{0.0};
		std::atomic<double> command_rad_s{0.0};
		// Ultimo pulso efetivamente programado (us); -1 = nenhum. Usado para so'
		// reprogramar o lgTxServo quando o valor MUDA (reprogramar a 100 Hz um
		// trem de 50 Hz truncava pulsos — auditoria do port, bug #2).
		std::atomic<int> last_pulse_us{-1};
		// Histerese do piso: ligar exige |cmd| >= floor; desligar so' abaixo de
		// 0.4*floor (mata o chattering 0 <-> piso a 100 Hz).
		std::atomic<bool> moving{false};
	};

	void control_loop();
	void failsafe_loop();
	/// Versao que assume o mutex JA tomado (usada pelo shutdown, que ja joinou
	/// as threads).
	void stop_all_motors_locked();
	/// Laco de amostragem do RIO: le em rajada, alimenta a quadratura e publica
	/// o instantaneo por seqlock. Roda em SCHED_FIFO alto, idealmente num core
	/// isolado. Nao faz syscall, nao aloca, nao pega lock.
	void sampler_loop();
	void update_cycle();
	// force = envia mesmo sem mudanca (init/stop/failsafe). Retorna false se o
	// lgTxServo falhar (logado com throttle; erro NUNCA e' silencioso).
	bool send_servo_pulse(std::size_t motor_index, int pulse_us, bool force);
	int velocity_to_pulse_width_us(
		double wheel_velocity_rad_s, bool currently_moving, int pulse_offset_us) const;
	int neutral_pulse_width_us() const;
	// Offset do pulso de cada roda dentro do periodo de 20 ms: 0/5/10/15 ms.
	// Com offset=0 nas 4, a thread de tx do lgpio agendava as 4 bordas para o
	// MESMO instante -> skew sistematico por ordem de atendimento (~40-80us =
	// ate ~2-3 rad/s de diferenca entre rodas; era o "curvar do nada").
	// Nota 2026-07-27: sonda trocando FL<->FR de slot provou que o slot NAO
	// influi na partida (+200ms da FL seguiu a RODA, nao o slot -> hardware
	// FL, provavel hall; ver docs). Qualquer permutacao de slots serve.
	static int servo_offset_us(std::size_t motor_index)
	{
		return static_cast<int>(motor_index) * 5000;
	}

	MaxonDriverConfig driver_config_;
	std::vector<MotorRuntime> motors_;

	// Amostragem do encoder (substitui a contagem por alertas do lgpio).
	caramelo::Rp1Rio rio_;
	std::thread sampler_thread_;
	std::atomic<bool> sampler_running_{false};
	// Seqlock: o escritor incrementa antes e depois de gravar, entao um leitor
	// que ve o mesmo valor PAR nas duas pontas leu um instantaneo coerente.
	// Custa ~50 ns no read(), sem lock e sem bloquear a thread de amostragem.
	mutable std::atomic<uint32_t> snap_seq_{0};
	EncoderSnapshot snap_{};
	std::atomic<int> health_{0};
	// Mensagem de diagnostico pre-dimensionada: publicar a 100 Hz alocando um
	// Float64MultiArray por ciclo era ~100 malloc/s dentro da thread que roda
	// com chrt -f 50.
	std_msgs::msg::Float64MultiArray diag_msg_;
	int diag_divisor_ = 0;
	// Rastro do mapa comando -> pulso (so' diagnostico; tocado pela thread de controle).
	int diag_pulse_[4] = {0, 0, 0, 0};
	double diag_cmd_[4] = {0.0, 0.0, 0.0, 0.0};

	double rad_per_count_ = 0.0;
	// Lidos pela thread de failsafe (que de proposito NAO pega o mutex) e pela
	// thread de amostragem, enquanto a thread do controller_manager escreve.
	std::atomic<int> chip_handle_{-1};
	std::atomic<bool> initialized_{false};
	rclcpp::Time last_update_time_;

	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;

	// Ultimo instantaneo consumido pelo update_cycle (so' a thread de controle
	// toca nestes).
	EncoderSnapshot last_snap_{};
	bool have_last_snap_ = false;

	std::thread control_thread_;
	std::atomic<bool> control_thread_running_{false};
	// Failsafe (2026-07-27): o lgTxServo com cycles=0 e' AUTONOMO — se o
	// control_loop congelar, o ultimo pulso continua saindo para sempre, valido
	// e fresco, e nenhum watchdog (driver ou firmware) dispara. Esta thread
	// dedicada (trabalho minimo, tenta SCHED_FIFO) vigia o heartbeat do
	// update_cycle: stale > 250 ms -> neutro; > 1 s -> corta os pulsos (o
	// firmware entao PARA os motores em ~500 ms).
	std::thread failsafe_thread_;
	std::atomic<bool> failsafe_thread_running_{false};
	std::atomic<int64_t> last_cycle_ns_{0};
	// Protege TUDO que fala com o lgpio ou escreve last_pulse_us/moving.
	// EXCECAO deliberada: a thread de failsafe nao pega este mutex — e' para
	// isso que ela existe. Por isso todo campo que ela toca e' atomico ou
	// imutavel apos a inicializacao.
	mutable std::mutex hw_mutex_;

	// Instante (steady clock, ns) do ultimo comando recebido via
	// set_command_velocity(); usado pelo watchdog de comando no update_cycle().
	std::atomic<int64_t> last_command_ns_{0};
};

}  // namespace mobile_base_hardware

#endif  // CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_
