#!/bin/sh
# Verificacao de saude do bringup no ar, com criterios objetivos.
#
# Uso: ssh raspberrypi@<host> 'bash -s' < tools/remote/check_stack.sh

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

echo "===== componentes de hardware ====="
timeout 25 ros2 control list_hardware_components 2>&1 | head -10

echo "===== controladores (todos devem estar active) ====="
timeout 25 ros2 control list_controllers 2>&1 | head -10

echo "===== /robot_description: Publisher count DEVE ser 1 ====="
# Mais de um publisher = robot_state_publisher zumbi servindo URDF velho num
# topico latched. O controller_manager pode engolir a descricao antiga sem
# nenhum erro, e ai "parametro novo do xacro nao faz efeito".
timeout 20 ros2 topic info /robot_description -v 2>/dev/null | grep -i "publisher count"

echo "===== tempo real: SO as threads de controle, nunca as de DDS ====="
CM=$(pgrep -f ros2_control_node | head -1)
# 2026-09-08: a thread PRINCIPAL agora e' SCHED_OTHER de proposito. Ate aqui o
# launch subia o processo inteiro com "chrt -f 50" e as 35 threads viravam tempo
# real, DDS incluso — o que fazia o EKF perder deadline (medido: 21 estouros em
# 2 min, ate 152 ms num ciclo de 25 ms, com os cores 68% ociosos). O criterio
# correto passou a ser POR THREAD:
#   - algumas poucas em FF (control_loop, tx do lgpio, update do
#     controller_manager, failsafe em 10, amostrador em 80);
#   - NENHUMA thread "dds.*" em FF.
if [ -n "$CM" ]; then
	ps -L -o tid,cls,rtprio,psr,comm -p "$CM" 2>/dev/null | sed -n '1p;/FF/p'
	DDS_RT=$(ps -L -o cls,comm -p "$CM" 2>/dev/null | awk '$1=="FF" && $2 ~ /^dds/ {n++} END{print n+0}')
	if [ "$DDS_RT" -gt 0 ]; then
		echo "  FALHA: $DDS_RT thread(s) de DDS em tempo real — o chrt do processo voltou?"
	else
		echo "  OK: nenhuma thread de DDS em tempo real"
	fi
	if ! ps -L -o cls,rtprio -p "$CM" 2>/dev/null | awk '$1=="FF" && $2==80 {f=1} END{exit !f}'; then
		echo "  FALHA: amostrador do encoder NAO esta em SCHED_FIFO 80"
	fi
fi

echo "===== taxas ====="
echo "-- /joint_states (esperado ~100 Hz) --"
timeout -s INT 10 ros2 topic hz /joint_states 2>&1 | tail -2
echo "-- /odom/wheel (esperado ~100 Hz, mesmo SEM comando) --"
timeout -s INT 10 ros2 topic hz /odom/wheel 2>&1 | tail -2
echo "-- /maxon/wheel_velocity (telemetria, esperado ~20 Hz) --"
# Janela maior: a 20 Hz, 8 s nao junta amostra suficiente para o hz reportar.
timeout -s INT 15 ros2 topic hz /maxon/wheel_velocity 2>&1 | tail -2

echo "===== TF odom -> base_footprint ====="
# O tf2_echo do Jazzy nao tem --once; corta pela janela de tempo.
timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -10

echo "===== recursos ====="
uptime
free -m | head -2
echo -n "temperatura: "; awk '{printf "%.1f C\n", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null
echo -n "governor: "; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null
ps -eo pid,pri,rtprio,pcpu,rss,comm --sort=-pcpu 2>/dev/null | head -6

echo "===== thread de amostragem (RTPRIO 80): deve estar no core isolado ====="
# PSR = core em que a thread esta. Com isolcpus, esse core nao recebe mais nada.
CM=$(pgrep -f ros2_control_node | head -1)
[ -n "$CM" ] && ps -L -o tid,rtprio,psr,pcpu,comm -p "$CM" 2>/dev/null | awk 'NR==1 || $2==80'
echo -n "cores isolados: "; cat /sys/devices/system/cpu/isolated 2>/dev/null
