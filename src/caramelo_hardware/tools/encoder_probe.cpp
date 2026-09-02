// encoder_probe — bancada dos encoders do Caramelo, standalone.
//
// Roda FORA do ros2_control: sem controller_manager, sem URDF, sem EKF. Usa o
// MESMO QuadratureDecoder do driver de producao, entao o que se mede aqui e' o
// que o robo vai fazer.
//
// Nao toca em nenhuma linha de PWM (17/23/24/25): so' le os 8 canais de encoder.
// Ainda assim, recusa rodar se houver um ros2_control_node vivo — dois donos do
// mesmo GPIO e' diagnostico perdido.
//
// Modos:
//   --info              estado dos pads/funcao/niveis das 8 linhas
//   --edges  <segundos> bordas CRUAS por linha (mede o chilrear do encoder parado)
//   --count  <segundos> contagem em quadratura x4 por roda, com voltas e ilegais
//
// Exemplos:
//   encoder_probe --info
//   encoder_probe --edges 10
//   encoder_probe --count 30 --rt --cpu 3

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <sched.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include "caramelo_hardware/quadrature_decoder.hpp"
#include "caramelo_hardware/rp1_rio.hpp"

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
#include <lgpio.h>
#endif

namespace
{

constexpr std::size_t kWheels = 4;

struct WheelPins
{
	const char * name;
	unsigned a_gpio;   // canal A FISICO
	unsigned b_gpio;   // canal B FISICO
};

// Fiacao fisica da PCB (imutavel: placa e chicote ja fabricados).
// A nomenclatura aqui e' HONESTA — A e' o A fisico. O sentido de cada roda e'
// resolvido por sinal explicito, nao trocando os nomes dos canais.
constexpr WheelPins kPins[kWheels] = {
	{"FL", 5, 6},
	{"FR", 27, 22},
	{"BL", 16, 26},
	{"BR", 20, 21},
};

// 1024 ciclos por canal por volta de motor x reducao 28:1 x decodificacao 4x.
// E' EXATAMENTE isso que o teste das 10 voltas a mao vai confirmar ou refutar:
// "1024 counts/volta, 2 canais" admite ler como 1024 ciclos POR CANAL (4096 em
// quadratura) ou 1024 ja em quadratura (256 ciclos por canal). E a reducao
// nominal "28:1" tambem e' suspeita: redutor planetario costuma ter razao
// fracionaria. O teste mede o PRODUTO, que e' a constante que o software usa.
constexpr double kCountsPerWheelRevX4 = 1024.0 * 28.0 * 4.0;
constexpr double kTwoPiLocal = 6.28318530717958647692;

// Linhas de PWM dos 4 ESCs. So' sao tocadas no modo --hold-neutral.
constexpr WheelPins kPwm[kWheels] = {
	{"FL", 17, 17},
	{"FR", 23, 23},
	{"BL", 24, 24},
	{"BR", 25, 25},
};

constexpr int kNeutroUs = 1500;
constexpr int kServoHz = 50;

uint32_t g_stable = 8;   // amostras de permanencia exigidas (filtro de glitch)
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
int g_chip = -1;

/// Abre o gpiochip do RP1 conferindo o LABEL, e diz em voz alta qual escolheu.
///
/// Isto e' um modo de falha real e caro: nesta Pi o RP1 e' o gpiochip4, e o
/// gpiochip0 e' o gpio-brcmstb interno do BCM2712, onde as linhas 24/25 sao
/// BT_RTS/BT_CTS. Um fallback silencioso para o chip 0 nao "nao funciona": ele
/// dirige pinos internos da placa achando que esta falando com os ESCs.
int abrir_chip_rp1(int forcado)
{
	if (forcado >= 0) {
		const int h = lgGpiochipOpen(forcado);
		if (h < 0) { std::fprintf(stderr, "lgGpiochipOpen(%d) falhou\n", forcado); return -1; }
		std::printf("gpiochip%d aberto (forcado por --chip)\n", forcado);
		return h;
	}
	for (int c = 0; c < 16; ++c) {
		const int h = lgGpiochipOpen(c);
		if (h < 0) { continue; }
		lgChipInfo_t info;
		if (lgGpioGetChipInfo(h, &info) == LG_OKAY && std::strstr(info.label, "rp1") != nullptr) {
			std::printf("gpiochip%d selecionado (label \"%s\", %d linhas)\n",
				c, info.label, info.lines);
			return h;
		}
		lgGpiochipClose(h);
	}
	std::fprintf(stderr,
		"NENHUM gpiochip com label contendo \"rp1\" foi encontrado.\n"
		"RECUSANDO abrir o gpiochip0 as cegas: nesta placa ele e' o gpio-brcmstb\n"
		"interno (linhas 24/25 = BT_RTS/BT_CTS), nao o header de 40 pinos.\n"
		"Confira com: gpiodetect   e force com: --chip N\n");
	return -1;
}

/// Segura pulso neutro nos 4 ESCs: estado seguro, e' o que silencia o alarme de
/// "sem sinal" das placas. Offsets escalonados 0/5/10/15 ms como na producao,
/// para as 4 bordas nao caírem no mesmo instante do periodo de 20 ms.
bool iniciar_neutro(int chip_forcado)
{
	g_chip = abrir_chip_rp1(chip_forcado);
	if (g_chip < 0) { return false; }
	for (std::size_t i = 0; i < kWheels; ++i) {
		const int pino = static_cast<int>(kPwm[i].a_gpio);
		if (lgGpioClaimOutput(g_chip, 0, pino, 0) < 0) {
			std::fprintf(stderr, "claim do PWM GPIO%d falhou (o bringup esta no ar?)\n", pino);
			return false;
		}
		if (lgTxServo(g_chip, pino, kNeutroUs, kServoHz, static_cast<int>(i) * 5000, 0) < 0) {
			std::fprintf(stderr, "lgTxServo GPIO%d falhou\n", pino);
			return false;
		}
		std::printf("  %s PWM GPIO%-3d = %d us\n", kPwm[i].name, pino, kNeutroUs);
	}
	std::printf("neutro ativo nos 4 ESCs — o alarme deve parar\n");
	return true;
}

/// Saida ordenada: reafirma neutro, deixa >=5 pulsos no fio e so' entao para.
void encerrar_neutro()
{
	if (g_chip < 0) { return; }
	for (std::size_t i = 0; i < kWheels; ++i) {
		lgTxServo(g_chip, static_cast<int>(kPwm[i].a_gpio), kNeutroUs, kServoHz,
			static_cast<int>(i) * 5000, 0);
	}
	usleep(120000);
	for (std::size_t i = 0; i < kWheels; ++i) {
		const int pino = static_cast<int>(kPwm[i].a_gpio);
		lgTxServo(g_chip, pino, 0, kServoHz, 0, 0);
		lgGpioFree(g_chip, pino);
	}
	lgGpiochipClose(g_chip);
	g_chip = -1;
	std::printf("neutro encerrado, linhas de PWM liberadas\n");
}
#endif  // CARAMELO_HAS_LGPIO

uint64_t now_ns()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

/// Recusa rodar junto com o stack de controle: dois donos do mesmo GPIO
/// produzem "Device or resource busy" e medicoes que nao querem dizer nada.
bool ros2_control_is_running()
{
	DIR * d = opendir("/proc");
	if (d == nullptr) { return false; }
	bool found = false;
	struct dirent * e;
	while (!found && (e = readdir(d)) != nullptr) {
		if (e->d_name[0] < '0' || e->d_name[0] > '9') { continue; }
		char path[300];
		std::snprintf(path, sizeof(path), "/proc/%s/cmdline", e->d_name);
		FILE * f = std::fopen(path, "rb");
		if (f == nullptr) { continue; }
		char buf[512] = {0};
		const std::size_t n = std::fread(buf, 1, sizeof(buf) - 1, f);
		std::fclose(f);
		for (std::size_t i = 0; i + 1 < n; ++i) { if (buf[i] == '\0') { buf[i] = ' '; } }
		if (std::strstr(buf, "ros2_control_node") != nullptr) { found = true; }
	}
	closedir(d);
	return found;
}

void apply_realtime(int cpu)
{
	if (cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(cpu, &set);
		if (sched_setaffinity(0, sizeof(set), &set) != 0) { std::perror("sched_setaffinity"); }
	}
	struct sched_param sp;
	sp.sched_priority = 80;
	if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
		std::perror("SCHED_FIFO (seguindo sem RT)");
	} else {
		// ATENCAO: com kernel.sched_rt_runtime_us no padrao (950000/1000000) uma
		// thread FIFO que usa 100% da CPU leva um BLACKOUT de 50 ms A CADA
		// SEGUNDO. Medido nesta Pi: pior intervalo 49.99 ms com o throttling
		// ligado, 16 us com ele desligado. Sem tratar isso, o decodificador
		// perde contagens em rajada e o sintoma vira "encoder ruim".
		FILE * f = std::fopen("/proc/sys/kernel/sched_rt_runtime_us", "r");
		if (f != nullptr) {
			long v = 0;
			if (std::fscanf(f, "%ld", &v) == 1 && v > 0) {
				std::printf(
					"AVISO: kernel.sched_rt_runtime_us=%ld -> blackout de ~%.0f ms/s na thread RT.\n"
					"       Para medir de verdade: sudo sysctl -w kernel.sched_rt_runtime_us=-1\n",
					v, (1000000.0 - static_cast<double>(v)) / 1000.0);
			}
			std::fclose(f);
		}
		mlockall(MCL_CURRENT | MCL_FUTURE);
	}
}

int mode_info(caramelo::Rp1Rio & rio)
{
	const uint32_t in = rio.read_in();
	const uint32_t oe = rio.read_oe();
	std::printf("RIO_IN = 0x%08x   RIO_OE = 0x%08x\n\n", in, oe);
	std::printf("%-4s %-4s %-5s %-6s %-9s %-6s %s\n",
		"roda", "gpio", "canal", "nivel", "funcsel", "pad", "flags do pad");
	for (const auto & w : kPins) {
		for (int ch = 0; ch < 2; ++ch) {
			const unsigned g = (ch == 0) ? w.a_gpio : w.b_gpio;
			const uint32_t pad = rio.read_pad(g);
			const uint32_t ctrl = rio.read_ctrl(g);
			std::printf("%-4s %-4u %-5s %-6u %-9u 0x%02x   %s%s%s%s%s\n",
				w.name, g, (ch == 0) ? "A" : "B", (in >> g) & 1u, ctrl & 0x1fu, pad,
				(pad & caramelo::kPadIe) ? "IE " : "",
				(pad & caramelo::kPadOd) ? "OD " : "",
				(pad & caramelo::kPadPue) ? "PULLUP " : "",
				(pad & caramelo::kPadPde) ? "PULLDOWN " : "",
				(pad & caramelo::kPadSchmitt) ? "SCHMITT" : "");
		}
	}
	return 0;
}

int mode_edges(caramelo::Rp1Rio & rio, double secs)
{
	// Conta transicoes CRUAS por linha, sem interpretar quadratura. E' assim que
	// se quantifica o chilrear (~60 kHz medidos em julho com o motor energizado
	// e a roda parada em cima de uma borda optica) e a contagem dobrada por
	// ringing na borda.
	uint32_t mask = 0;
	for (const auto & w : kPins) { mask |= (1u << w.a_gpio) | (1u << w.b_gpio); }

	uint64_t edges[32] = {0};
	uint32_t prev = rio.read_in() & mask;
	const uint64_t t0 = now_ns();
	const uint64_t tend = t0 + static_cast<uint64_t>(secs * 1e9);
	uint64_t samples = 0;

	constexpr std::size_t kBurst = 16;
	uint32_t buf[kBurst];
	while (!g_stop.load(std::memory_order_relaxed)) {
		for (int rep = 0; rep < 16; ++rep) {
			rio.read_burst(buf);
			for (std::size_t i = 0; i < kBurst; ++i) {
				const uint32_t cur = buf[i] & mask;
				if (cur != prev) {
					uint32_t diff = cur ^ prev;
					while (diff) {
						const int b = __builtin_ctz(diff);
						++edges[b];
						diff &= diff - 1;
					}
					prev = cur;
				}
			}
		}
		samples += 16 * kBurst;
		if (now_ns() >= tend) { break; }
	}
	const double dur = static_cast<double>(now_ns() - t0) / 1e9;

	std::printf("janela=%.2f s  amostras=%" PRIu64 "  taxa=%.2f MHz\n\n",
		dur, samples, (static_cast<double>(samples) / dur) / 1e6);
	std::printf("%-4s %-5s %-5s %14s %12s\n", "roda", "canal", "gpio", "bordas", "bordas/s");
	for (const auto & w : kPins) {
		for (int ch = 0; ch < 2; ++ch) {
			const unsigned g = (ch == 0) ? w.a_gpio : w.b_gpio;
			std::printf("%-4s %-5s %-5u %14" PRIu64 " %12.1f\n",
				w.name, (ch == 0) ? "A" : "B", g, edges[g],
				static_cast<double>(edges[g]) / dur);
		}
	}
	return 0;
}

int mode_count(caramelo::Rp1Rio & rio, double secs, double counts_per_rev)
{
	std::array<caramelo::ChannelMap, kWheels> chans{};
	for (std::size_t i = 0; i < kWheels; ++i) {
		chans[i].a_bit = static_cast<uint8_t>(kPins[i].a_gpio);
		chans[i].b_bit = static_cast<uint8_t>(kPins[i].b_gpio);
		chans[i].sign = 1;   // sinal por roda e' calibrado por MEDICAO, nao deduzido
	}
	caramelo::QuadratureDecoder<kWheels> dec(chans, g_stable);
	dec.reset(rio.read_in());

	const uint64_t t0 = now_ns();
	const uint64_t tend = t0 + static_cast<uint64_t>(secs * 1e9);
	uint64_t next_print = t0 + 1000000000ull;
	uint64_t samples = 0, worst_gap = 0, tprev = t0;

	std::printf("contando %.0f s (quadratura x4, %.0f counts por volta de roda)\n",
		secs, counts_per_rev);
	std::printf("gire as rodas a mao — Ctrl-C encerra antes do prazo\n\n");

	constexpr std::size_t kBurst = 16;
	uint32_t buf[kBurst];
	while (!g_stop.load(std::memory_order_relaxed)) {
		for (int rep = 0; rep < 16; ++rep) {
			rio.read_burst(buf);
			for (std::size_t i = 0; i < kBurst; ++i) { dec.update(buf[i]); }
		}
		samples += 16 * kBurst;
		const uint64_t t = now_ns();
		const uint64_t gap = t - tprev;
		tprev = t;
		if (gap > worst_gap) { worst_gap = gap; }
		if (t >= next_print) {
			next_print += 1000000000ull;
			std::printf("[%5.1fs]", static_cast<double>(t - t0) / 1e9);
			for (std::size_t i = 0; i < kWheels; ++i) {
				std::printf("  %s=%+9" PRId64 " (%+7.3f volta)",
					kPins[i].name, dec.count(i),
					static_cast<double>(dec.count(i)) / counts_per_rev);
			}
			std::printf("\n");
			std::fflush(stdout);
		}
		if (t >= tend) { break; }
	}

	const double dur = static_cast<double>(now_ns() - t0) / 1e9;
	std::printf("\n=== resultado ===\n");
	std::printf("janela=%.2f s  amostras=%" PRIu64 "  taxa=%.2f MHz  pior gap(bloco de 256)=%" PRIu64 " ns\n",
		dur, samples, (static_cast<double>(samples) / dur) / 1e6, worst_gap);
	std::printf("%-4s %14s %12s %14s\n", "roda", "counts(x4)", "voltas", "ilegais");
	for (std::size_t i = 0; i < kWheels; ++i) {
		std::printf("%-4s %14" PRId64 " %12.4f %14" PRIu64 "\n",
			kPins[i].name, dec.count(i),
			static_cast<double>(dec.count(i)) / counts_per_rev, dec.illegal(i));
	}
	std::printf("\nilegais > 0 = as duas linhas mudaram entre amostras consecutivas\n"
		"(perda de amostra ou chilrear simultaneo). Em regime saudavel deve ser 0.\n");
	return 0;
}

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
/// Cutuca cada roda por vez: pulso acima do piso por um instante, volta a
/// neutro, e mede o que o encoder viu.
///
/// Serve a dois propositos de uma vez:
///  1) destrava as rodas — o ESC armado em neutro segura o eixo, e um comando
///     curto solta o motor para que o robo possa ser girado a mao;
///  2) da o SINAL de cada roda sob um comando de sentido CONHECIDO, que e'
///     exatamente a calibracao que nao pode ser deduzida do codigo (hoje o
///     sinal e' decidido por numero de pino, o que quebra em silencio se
///     alguem remapear).
///
/// Uma roda por vez, sempre. As outras tres sao observadas junto para flagrar
/// diafonia entre canais (contagem numa roda que nao foi comandada).
int mode_nudge(caramelo::Rp1Rio & rio, int pulse_us, double hold_s)
{
	std::array<caramelo::ChannelMap, kWheels> chans{};
	for (std::size_t i = 0; i < kWheels; ++i) {
		chans[i].a_bit = static_cast<uint8_t>(kPins[i].a_gpio);
		chans[i].b_bit = static_cast<uint8_t>(kPins[i].b_gpio);
		chans[i].sign = 1;
	}
	caramelo::QuadratureDecoder<kWheels> dec(chans, g_stable);
	dec.reset(rio.read_in());

	constexpr std::size_t kBurst = 16;
	uint32_t buf[kBurst];
	auto amostrar_ate = [&](uint64_t t_fim) {
		while (now_ns() < t_fim && !g_stop.load(std::memory_order_relaxed)) {
			for (int rep = 0; rep < 16; ++rep) {
				rio.read_burst(buf);
				for (std::size_t i = 0; i < kBurst; ++i) { dec.update(buf[i]); }
			}
		}
	};

	std::printf("\ncutucando cada roda com %d us por %.2f s (piso do ESC ~1570 us)\n",
		pulse_us, hold_s);
	std::printf("RODAS SUSPENSAS. Uma de cada vez.\n\n");

	for (std::size_t w = 0; w < kWheels && !g_stop.load(std::memory_order_relaxed); ++w) {
		int64_t antes[kWheels];
		for (std::size_t i = 0; i < kWheels; ++i) { antes[i] = dec.count(i); }

		const int pino = static_cast<int>(kPwm[w].a_gpio);
		lgTxServo(g_chip, pino, pulse_us, kServoHz, static_cast<int>(w) * 5000, 0);
		amostrar_ate(now_ns() + static_cast<uint64_t>(hold_s * 1e9));
		lgTxServo(g_chip, pino, kNeutroUs, kServoHz, static_cast<int>(w) * 5000, 0);
		// Deixa a inercia acabar antes de fechar a conta (o firmware segura a
		// referencia por ~0.5 s depois do comando).
		amostrar_ate(now_ns() + 1200000000ull);

		std::printf("%s (GPIO%d):", kPwm[w].name, pino);
		for (std::size_t i = 0; i < kWheels; ++i) {
			const int64_t d = dec.count(i) - antes[i];
			std::printf("  %s=%+8" PRId64, kPins[i].name, d);
		}
		std::printf("  | ilegais:");
		for (std::size_t i = 0; i < kWheels; ++i) {
			std::printf(" %s=%" PRIu64, kPins[i].name, dec.illegal(i));
		}
		std::printf("  descartadas=%" PRIu64 "\n", dec.rejected());
		std::fflush(stdout);
	}

	std::printf("\nleia assim: na linha da roda comandada, a coluna dela deve ser a UNICA\n"
		"nao-nula. O SINAL diz o sentido que o encoder ve quando o ESC recebe um\n"
		"pulso de FRENTE — e' o que calibra encoder_sign por roda.\n");
	return 0;
}
#endif  // CARAMELO_HAS_LGPIO


/// Varredura de PULSOS: mede a curva real pulso -> velocidade de cada placa.
///
/// POR QUE ISTO EXISTE: o driver assume uma reta afim unica para as quatro
/// rodas (piso em 1570 us = 2.43 rad/s, escala cheia em 2000 us = 20.06 rad/s).
/// Medindo, as rodas da direita entregam ~6% menos velocidade que as da
/// esquerda com o MESMO pulso. Tentar corrigir isso inferindo um deslocamento a
/// partir da reta assumida FALHOU em 2026-09-01 — piorou a assimetria para 24%,
/// porque a resposta real a um degrau de pulso nao bateu com a inclinacao da
/// reta. Ou seja: a reta esta errada, e nao adianta corrigir parametros dela.
///
/// Aqui nao se assume nada: comanda-se um pulso EXPLICITO e mede-se o que sai.
/// O resultado e' a curva de cada placa, de onde saem piso e inclinacao reais.
///
/// Uma roda por vez; as outras tres ficam em neutro e sao observadas junto,
/// para flagrar diafonia.
int mode_pulse_sweep(
	caramelo::Rp1Rio & rio, int alvo_roda, double dwell_s, double settle_s)
{
	std::array<caramelo::ChannelMap, kWheels> chans{};
	for (std::size_t i = 0; i < kWheels; ++i) {
		chans[i].a_bit = static_cast<uint8_t>(kPins[i].a_gpio);
		chans[i].b_bit = static_cast<uint8_t>(kPins[i].b_gpio);
		// -1 alinha o decodificador com o ESPACO DE PULSO: pulso de frente passa
		// a contar positivo, que e' como a tabela abaixo fica legivel.
		chans[i].sign = -1;
	}
	caramelo::QuadratureDecoder<kWheels> dec(chans, g_stable);
	dec.reset(rio.read_in());

	constexpr std::size_t kBurst = 16;
	uint32_t buf[kBurst];
	auto amostrar_ate = [&](uint64_t t_fim) {
		while (now_ns() < t_fim && !g_stop.load(std::memory_order_relaxed)) {
			for (int rep = 0; rep < 16; ++rep) {
				rio.read_burst(buf);
				for (std::size_t i = 0; i < kBurst; ++i) { dec.update(buf[i]); }
			}
		}
	};

	// Pontos escolhidos para cobrir a faixa util sem gastar bateria a toa.
	// Frente: acima do limiar de arme (1540) com margem. Re: espelhado.
	static const int kPulsosFrente[] = {1580, 1600, 1630, 1660, 1700, 1750, 1800};
	static const int kPulsosRe[]     = {1420, 1400, 1370, 1340, 1300, 1250, 1200};
	const double rad_por_count = kTwoPiLocal / kCountsPerWheelRevX4;

	for (std::size_t w = 0; w < kWheels; ++w) {
		if (alvo_roda >= 0 && static_cast<int>(w) != alvo_roda) { continue; }
		if (g_stop.load(std::memory_order_relaxed)) { break; }
		const int pino = static_cast<int>(kPwm[w].a_gpio);
		std::printf("\n===== %s (PWM GPIO %d) =====\n", kPwm[w].name, pino);
		std::printf("%8s %12s %12s %10s\n", "pulso_us", "rad/s", "counts", "diafonia");

		for (int ramo = 0; ramo < 2; ++ramo) {
			const int * pulsos = (ramo == 0) ? kPulsosFrente : kPulsosRe;
			for (int k = 0; k < 7 && !g_stop.load(std::memory_order_relaxed); ++k) {
				const int pulso = pulsos[k];
				lgTxServo(g_chip, pino, pulso, kServoHz, static_cast<int>(w) * 5000, 0);
				// Deixa o ESC estabilizar antes de medir (a rampa dele e' ~50 ms,
				// mas o laco de velocidade leva mais para assentar).
				amostrar_ate(now_ns() + static_cast<uint64_t>(settle_s * 1e9));

				int64_t antes[kWheels];
				for (std::size_t i = 0; i < kWheels; ++i) { antes[i] = dec.count(i); }
				const uint64_t t0 = now_ns();
				amostrar_ate(t0 + static_cast<uint64_t>(dwell_s * 1e9));
				const double dt = static_cast<double>(now_ns() - t0) * 1e-9;

				const int64_t d = dec.count(w) - antes[w];
				double diafonia = 0.0;
				for (std::size_t i = 0; i < kWheels; ++i) {
					if (i == w) { continue; }
					const double outra = std::fabs(
						static_cast<double>(dec.count(i) - antes[i]) * rad_por_count / dt);
					if (outra > diafonia) { diafonia = outra; }
				}
				std::printf("%8d %12.4f %12lld %10.4f\n",
					pulso, static_cast<double>(d) * rad_por_count / dt,
					static_cast<long long>(d), diafonia);
				std::fflush(stdout);

				// Volta a neutro entre pontos: nao acumula calor nem deixa a
				// roda girando enquanto o proximo ponto e' preparado.
				lgTxServo(g_chip, pino, kNeutroUs, kServoHz, static_cast<int>(w) * 5000, 0);
				amostrar_ate(now_ns() + 1200000000ull);
			}
		}
	}
	std::printf("\nA coluna diafonia deve ficar ~0: e' a maior velocidade vista nas\n"
		"rodas que NAO foram comandadas naquele ponto.\n");
	return 0;
}

void usage()
{
	std::printf(
		"uso: encoder_probe [--info | --edges SEG | --count SEG] [opcoes]\n"
		"  --info           estado de pad/funcao/nivel das 8 linhas de encoder\n"
		"  --edges SEG      bordas cruas por linha (mede chilrear e ringing)\n"
		"  --count SEG      contagem em quadratura x4 por roda\n"
		"  --pulse-sweep    mede a curva REAL pulso -> velocidade de cada placa\n"
		"  --wheel fl|fr|bl|br|all   restringe a varredura a uma roda\n"
		"  --dwell S        janela de medicao por ponto (default 2.0)\n"
		"  --hold-neutral   segura 1500 us nos 4 ESCs enquanto mede (silencia o\n"
		"                   alarme das placas energizadas; estado seguro)\n"
		"  --chip N         forca o gpiochip do PWM em vez de detectar pelo label\n"
		"  --rt             SCHED_FIFO 80 + mlockall\n"
		"  --cpu N          fixa a thread no core N\n"
		"  --cpr N          counts por volta de roda em x4 (default %.0f)\n",
		kCountsPerWheelRevX4);
}

}  // namespace

int main(int argc, char ** argv)
{
	enum class Mode { None, Info, Edges, Count } mode = Mode::None;
	double secs = 10.0;
	double cpr = kCountsPerWheelRevX4;
	bool rt = false;
	bool hold_neutral = false;
	int cpu = -1;
	int chip = -1;
	bool do_nudge = false;
	bool do_sweep = false;
	int nudge_us = 1590;
	int roda = -1;
	double dwell = 2.0;
	double nudge_s = 0.4;

	for (int i = 1; i < argc; ++i) {
		const std::string a = argv[i];
		if (a == "--info") { mode = Mode::Info; }
		else if (a == "--edges" && i + 1 < argc) { mode = Mode::Edges; secs = std::atof(argv[++i]); }
		else if (a == "--count" && i + 1 < argc) { mode = Mode::Count; secs = std::atof(argv[++i]); }
		else if (a == "--cpu" && i + 1 < argc) { cpu = std::atoi(argv[++i]); }
		else if (a == "--cpr" && i + 1 < argc) { cpr = std::atof(argv[++i]); }
		else if (a == "--chip" && i + 1 < argc) { chip = std::atoi(argv[++i]); }
		else if (a == "--stable" && i + 1 < argc) { g_stable = static_cast<uint32_t>(std::atoi(argv[++i])); }
		else if (a == "--rt") { rt = true; }
		else if (a == "--hold-neutral") { hold_neutral = true; }
		else if (a == "--nudge") { do_nudge = true; hold_neutral = true; }
		else if (a == "--nudge-us" && i + 1 < argc) { nudge_us = std::atoi(argv[++i]); }
		else if (a == "--nudge-s" && i + 1 < argc) { nudge_s = std::atof(argv[++i]); }
		else if (a == "--pulse-sweep") { do_sweep = true; hold_neutral = true; }
		else if (a == "--dwell" && i + 1 < argc) { dwell = std::atof(argv[++i]); }
		else if (a == "--wheel" && i + 1 < argc) {
			const std::string r = argv[++i];
			roda = (r == "fl") ? 0 : (r == "fr") ? 1 : (r == "bl") ? 2 : (r == "br") ? 3 : -1;
			if (roda < 0 && r != "all") { std::fprintf(stderr, "roda invalida: %s\n", r.c_str()); return 2; }
		}
		else { usage(); return 2; }
	}
	if (mode == Mode::None && !do_nudge && !do_sweep) { usage(); return 2; }

	if (ros2_control_is_running()) {
		std::fprintf(stderr,
			"RECUSANDO: ha um ros2_control_node vivo. Dois donos do mesmo GPIO nao medem nada.\n"
			"Derrube o bringup antes (SIGINT no grupo de processos).\n");
		return 3;
	}

	std::signal(SIGINT, on_signal);
	std::signal(SIGTERM, on_signal);

	caramelo::Rp1Rio rio;
	const std::string err = rio.open_device();
	if (!err.empty()) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}

	// Entrada + pull-up + Schmitt nas 8 linhas. Nenhuma linha de PWM e' tocada.
	for (const auto & w : kPins) {
		rio.configure_input(w.a_gpio, true, true);
		rio.configure_input(w.b_gpio, true, true);
	}

	if (hold_neutral) {
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
		if (!iniciar_neutro(chip)) {
			encerrar_neutro();
			return 1;
		}
#else
		std::fprintf(stderr, "--hold-neutral exige lgpio, e este binario foi compilado sem.\n");
		return 1;
#endif
	}

	if (rt) { apply_realtime(cpu); }

	int rc = 0;
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// A cutucada vem ANTES do modo escolhido: ela destrava os eixos, entao um
	// --count logo em seguida ja encontra as rodas livres para girar a mao.
	if (do_nudge) { rc = mode_nudge(rio, nudge_us, nudge_s); }
	if (do_sweep) { rc = mode_pulse_sweep(rio, roda, dwell, 1.5); }
#endif
	switch (mode) {
		case Mode::Info: rc = mode_info(rio); break;
		case Mode::Edges: rc = mode_edges(rio, secs); break;
		case Mode::Count: rc = mode_count(rio, secs, cpr); break;
		default: break;
	}

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	if (hold_neutral) { encerrar_neutro(); }
#endif
	return rc;
}
