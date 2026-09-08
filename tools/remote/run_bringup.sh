#!/bin/sh
# Sobe o bringup de forma que sobreviva ao fim da sessao SSH e possa ser
# encerrado limpo depois.
#
# Uso (do PC, por stdin — NUNCA por argv, ver nota de path conversion abaixo):
#   ssh raspberrypi@<host> 'bash -s' -- use_lidar:=false use_imu:=false \
#       < tools/remote/run_bringup.sh
#
# NOTA sobre argv: no Git Bash do Windows, um argumento como /opt/ros/... e'
# convertido para caminho Windows antes de sair da maquina. Por isso todo
# script deste diretorio e' enviado por STDIN, com heredoc de delimitador entre
# aspas simples.

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

# Guarda contra dois donos do mesmo hardware. Subir um segundo
# robot_state_publisher e' especialmente traicoeiro: ele publica o URDF num
# topico latched (TRANSIENT_LOCAL) e o controller_manager novo pode engolir a
# descricao ANTIGA sem nenhum erro — parametros novos do xacro simplesmente
# "nao fazem efeito".
if pgrep -f 'ros2_control_node|robot_state_publisher' >/dev/null 2>&1; then
	echo "ABORTANDO: ja ha processos ROS no ar. Rode stop_bringup.sh antes."
	pgrep -af 'ros2_control_node|robot_state_publisher' | head -5
	exit 3
fi

# GUARDA DE RELOGIO. A Pi nao tem bateria de RTC: ela acorda com a hora do
# ultimo desligamento e so' conserta quando alcanca o notebook. Medido em
# 2026-09-08: "System clock wrong by 535917 seconds" e um STEP de 6,2 DIAS.
#
# Subir o bringup ANTES desse passo e' pior que esperar: os nos nascem com a
# hora errada e levam o salto EM VOO. Os stamps de /scan e /odom pulam para o
# futuro, a TF extrapola e o Nav2 passa a descartar mensagem em silencio — o
# modo de falha mais caro que existe aqui, porque nao gera erro nenhum.
#
# CARAMELO_IGNORA_RELOGIO=1 pula a checagem (bancada sem o notebook na rede).
if [ "${CARAMELO_IGNORA_RELOGIO:-0}" != "1" ] && command -v chronyc >/dev/null 2>&1; then
	i=0
	while [ "$i" -lt 30 ]; do
		LEAP="$(chronyc tracking 2>/dev/null | awk -F': *' '/Leap status/{print $2}')"
		REFID="$(chronyc tracking 2>/dev/null | awk -F': *' '/Reference ID/{print $2}')"
		case "$LEAP" in
			Normal*)
				case "$REFID" in
					00000000*|7F7F01*) ;;   # nao sincronizado / referencia local
					*) break ;;
				esac
				;;
		esac
		[ "$i" = 0 ] && echo "aguardando o relogio sincronizar (chronyc: ${LEAP:-sem resposta})..."
		i=$((i + 1))
		sleep 1
	done
	if [ "$i" -ge 30 ]; then
		echo "ABORTANDO: o relogio nao sincronizou em 30 s."
		chronyc tracking 2>&1 | sed -n '1,4p'
		echo "Subir assim faz os nos levarem um STEP de relogio em voo (stamps pulam,"
		echo "TF extrapola, Nav2 descarta scan em silencio). Confira o chrony no"
		echo "notebook (10.42.0.1) ou force com CARAMELO_IGNORA_RELOGIO=1."
		exit 4
	fi
	echo "relogio OK: $(chronyc tracking | awk -F': *' '/System time/{print $2}')"
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG="$CARAMELO_LOGS/bringup_$RUN_ID.log"

# setsid: nova sessao, entao o processo NAO recebe SIGHUP quando o SSH cai, e
# ganha um process group proprio — que e' o alvo do SIGINT no stop_bringup.
# O `echo $$` de dentro do bash -c grava o PGID de forma deterministica; o $!
# do shell pai nao serve, porque sem job control o filho pode herdar o grupo
# do shell.
# </dev/null e a redirecao sao o que faz o ssh RETORNAR em vez de pendurar
# esperando o canal fechar.
setsid bash -c '
  echo $$ > "'"$CARAMELO_PGID_FILE"'"
  exec ros2 launch raspberry_bringup hardware_bringup.launch.py '"$*"'
' </dev/null >"$LOG" 2>&1 &

sleep 2
echo "RUN_ID=$RUN_ID"
echo "PGID=$(cat "$CARAMELO_PGID_FILE" 2>/dev/null)"
echo "LOG=$LOG"
