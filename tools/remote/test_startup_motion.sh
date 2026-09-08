#!/bin/bash
# Mede MOVIMENTO NAO COMANDADO durante a subida do bringup.
#
# O sintoma relatado pelo operador: ao subir o bringup (ou launches de task), os
# motores se mexem sozinhos e o robo sai girando ou andando. Isso e' pior que
# qualquer erro de calibracao — e' atuacao sem comando.
#
# COMO A MEDICAO FUNCIONA: a contagem do encoder nasce em ZERO quando o driver
# inicializa (QuadratureDecoder::reset no sampler_loop). Logo, se a posicao das
# juntas nao for zero depois da subida, as rodas giraram durante ela. Nao ha
# ambiguidade possivel: ninguem comandou nada nessa janela.
#
# Uso:  ssh raspberrypi@<host> 'bash -s' -- <repeticoes> < tools/remote/test_startup_motion.sh
#
# SEGURANCA: rode com as RODAS SUSPENSAS. E' justamente o cenario em que o robo
# se move sem comando.

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

N="${1:-3}"

for i in $(seq 1 "$N"); do
	echo "########## subida $i de $N ##########"

	# Estado limpo antes de cada tentativa: um bringup anterior vivo invalida a
	# medicao (a contagem nao comeca do zero).
	for p in ros2_control_node "ros2 launch" robot_state_publisher ekf_node mesh_server; do
		pkill -f "$p" 2>/dev/null
	done
	sleep 4

	LOG="$CARAMELO_LOGS/startup_${i}.log"
	setsid bash -c '
		exec ros2 launch raspberry_bringup hardware_bringup.launch.py \
			use_manipulator:=false use_lidar:=false use_imu:=false
	' </dev/null >"$LOG" 2>&1 &

	# Espera o controlador ficar ativo. So' entao ha /joint_states para ler.
	esperou=0
	while [ "$esperou" -lt 40 ]; do
		if timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "mecanum_controller.*active"; then
			break
		fi
		esperou=$((esperou + 2))
		sleep 2
	done

	echo "controlador ativo apos ${esperou}s"
	sleep 3

	echo "--- posicao das juntas (ordem BL, BR, FL, FR) ---"
	# Qualquer valor != 0 aqui e' giro que aconteceu durante a subida.
	timeout -s INT 10 ros2 topic echo /joint_states --field position --once 2>/dev/null | grep "^array"

	echo "--- velocidade agora (deve ser zero: ninguem comandou) ---"
	timeout -s INT 10 ros2 topic echo /joint_states --field velocity --once 2>/dev/null | grep "^array"

	echo "--- avisos do driver na subida ---"
	grep -iE "error|warn|nao-finito|FAILSAFE|MORTO" "$LOG" | grep -v "Timestamp in header" | head -4

	# ARME e GUARDA DE PARTIDA sao logados SEMPRE (nao dependem de
	# log_pwm_reprogram/log_pulse_trace, que ficam desligados em operacao). Sao
	# eles que dizem se houve disparo E se a mitigacao pegou:
	#   "ARME ... girou X volta SEM COMANDO"  -> disparou antes de o driver subir
	#   "GUARDA DE PARTIDA: roda ... NEUTRO"  -> disparou depois, e foi cortada
	#   "GUARDA DE PARTIDA: janela encerrada" -> os 25 s passaram limpos
	echo "--- arme e guarda de partida ---"
	grep -E "ARME|GUARDA DE PARTIDA" "$LOG" | head -8

	echo "--- pulsos registrados durante a subida (so' com log_pulse_trace) ---"
	grep -E "pulsos:|REPROGRAMA" "$LOG" | head -3

	sleep 2
done

echo
echo "########## encerrando ##########"
for p in ros2_control_node "ros2 launch" robot_state_publisher ekf_node mesh_server; do
	pkill -f "$p" 2>/dev/null
done
sleep 3
echo "LEITURA: posicao != 0 em qualquer roda = ela girou SEM COMANDO durante a subida."
echo "Compare entre as repeticoes: se o padrao se repete, e' deterministico"
echo "(sequencia de inicializacao); se varia, e' condicao de corrida ou arme do ESC."
