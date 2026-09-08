#!/bin/sh
# Amostrador de recursos de LONGA DURACAO, para separar vazamento de memoria de
# starvation de CPU quando a Pi "congela".
#
# Uso (do PC, por stdin — ver nota de argv em run_bringup.sh):
#   ssh raspberrypi@<host> 'bash -s' -- start 5   < tools/remote/memsnap.sh
#   ssh raspberrypi@<host> 'bash -s' -- report    < tools/remote/memsnap.sh
#   ssh raspberrypi@<host> 'bash -s' -- stop      < tools/remote/memsnap.sh
#
# POR QUE fd e thread junto com RSS: o sintoma relatado e' a Pi parar de
# responder, e com 16 GB sem swap um vazamento de memoria puro seria morto pelo
# OOM killer (que deixa rastro no dmesg) em vez de congelar. Congelamento sem
# rastro tem a cara de starvation por SCHED_FIFO, e so' da' para separar os dois
# medindo RSS, fd, threads e CPU ao mesmo tempo.
#
# O modo "start" GRAVA um coletor autocontido em $CARAMELO_LOGS e o roda com
# setsid. Nao usa "$0": este script chega por stdin, entao $0 nao e' um caminho.

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

PIDFILE="$CARAMELO_LOGS/memsnap.pid"
LATEST="$CARAMELO_LOGS/memsnap.latest"
TICK="$CARAMELO_LOGS/memsnap_tick.sh"

escreve_coletor() {
	cat > "$TICK" <<'TICKEOF'
#!/bin/sh
# Uma amostra do stack. Gerado por tools/remote/memsnap.sh — nao editar a mao.
PADROES='ros2_control:ros2_control_node
ekf:ekf_node
rsp:robot_state_publisher
lidar:sllidar_node
imu:wit_ros2_imu
scan_norm:scan_normalizer
mesh:mesh_server'

agora="$(date +%s)"
iso="$(date -Iseconds)"
mem_avail="$(awk '/MemAvailable/{print $2}' /proc/meminfo)"
load="$(awk '{print $1}' /proc/loadavg)"

echo "$PADROES" | while IFS=: read -r nome padrao; do
	[ -z "$nome" ] && continue
	pid="$(pgrep -f "$padrao" 2>/dev/null | head -1)"
	[ -z "$pid" ] && continue
	rss="$(awk '/^VmRSS:/{print $2}' "/proc/$pid/status" 2>/dev/null)"
	thr="$(awk '/^Threads:/{print $2}' "/proc/$pid/status" 2>/dev/null)"
	fds="$(ls "/proc/$pid/fd" 2>/dev/null | wc -l)"
	cpu="$(awk '{print $14+$15}' "/proc/$pid/stat" 2>/dev/null)"
	[ -z "$rss" ] && continue
	printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
		"$agora" "$iso" "$nome" "$pid" "$rss" "$thr" "$fds" "$cpu" "$mem_avail"
done
printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
	"$agora" "$iso" "_sistema" "0" "0" "0" "0" "$load" "$mem_avail"
TICKEOF
	chmod +x "$TICK"
}

case "${1:-report}" in
start)
	intervalo="${2:-5}"
	if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
		echo "JA RODANDO pid=$(cat "$PIDFILE") log=$(cat "$LATEST" 2>/dev/null)"
		exit 0
	fi
	escreve_coletor
	RUN_ID="$(date +%Y%m%d_%H%M%S)"
	LOG="$CARAMELO_LOGS/mem_$RUN_ID.tsv"
	echo "$LOG" > "$LATEST"
	printf 'epoch\tiso\tproc\tpid\trss_kb\tthreads\tfds\tcpu_ticks\tmem_avail_kb\n' > "$LOG"
	# setsid + </dev/null: sobrevive a queda do SSH. O PGID gravado e' o alvo do
	# stop, para levar junto o `sleep` filho (senao o laco reaparece).
	setsid sh -c '
		echo $$ > "'"$PIDFILE"'"
		while :; do
			sh "'"$TICK"'" >> "'"$LOG"'" 2>/dev/null
			sleep '"$intervalo"'
		done
	' </dev/null >/dev/null 2>&1 &
	sleep 1
	echo "INICIADO pid=$(cat "$PIDFILE" 2>/dev/null) intervalo=${intervalo}s log=$LOG"
	;;
stop)
	if [ -f "$PIDFILE" ]; then
		pid="$(cat "$PIDFILE")"
		kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
		rm -f "$PIDFILE"
		echo "PARADO pid=$pid"
	else
		echo "nao estava rodando"
	fi
	;;
report)
	LOG="${2:-$(cat "$LATEST" 2>/dev/null)}"
	if [ -z "$LOG" ] || [ ! -f "$LOG" ]; then
		echo "sem amostras"
		exit 1
	fi
	echo "log=$LOG"
	# Taxa em MB/h calculada entre a PRIMEIRA e a ULTIMA amostra de cada
	# processo: um vazamento de verdade aparece como taxa positiva sustentada,
	# nao como um degrau (que e' so' cache/arena do alocador assentando).
	awk -F'\t' '
		NR == 1 { next }
		$3 == "_sistema" {
			if (sis0 == "") { sis0 = $9; t0s = $1 }
			sis1 = $9; t1s = $1; load = $8; next
		}
		{
			if (!(($3) in r0)) { r0[$3]=$5; t0[$3]=$1; f0[$3]=$7; h0[$3]=$6; c0[$3]=$8 }
			r1[$3]=$5; t1[$3]=$1; f1[$3]=$7; h1[$3]=$6; c1[$3]=$8; n[$3]++
			if ($5+0 > pico[$3]+0) pico[$3]=$5
		}
		END {
			printf "%-11s %6s %8s %8s %8s %9s %9s %7s %6s\n", \
				"proc","amostr","rss0_MB","rss1_MB","pico_MB","MB/h","fd","thr","cpu%"
			for (p in r1) {
				dt = t1[p] - t0[p]; if (dt <= 0) dt = 1
				printf "%-11s %6d %8.1f %8.1f %8.1f %9.2f %4d>%-4d %7d %6.1f\n", \
					p, n[p], r0[p]/1024.0, r1[p]/1024.0, pico[p]/1024.0, \
					(r1[p]-r0[p])/1024.0*3600.0/dt, f0[p], f1[p], h1[p], \
					(c1[p]-c0[p])/dt
			}
			dts = t1s - t0s; if (dts <= 0) dts = 1
			printf "\nMemAvailable: %.0f -> %.0f MB (%.2f MB/h) | janela %.1f min | load %s\n", \
				sis0/1024.0, sis1/1024.0, (sis1-sis0)/1024.0*3600.0/dts, dts/60.0, load
		}
	' "$LOG"
	;;
*)
	echo "uso: memsnap.sh start [intervalo_s] | report [log] | stop"
	exit 2
	;;
esac
