#!/bin/bash
# Configuracao de sistema da Raspberry do Caramelo — idempotente e verificavel.
#
# POR QUE ISTO EXISTE: ate 2026-09-01 tudo que faz o robo funcionar fora do
# workspace (limites de tempo real, governor, throttling de RT, isolamento de
# core, regras udev, chrony) existia SO na Pi, digitado a mao e documentado em
# prosa. Uma Pi reinstalada perdia tudo em silencio — e foi exatamente isso que
# aconteceu na troca da Pi 4 para a Pi 5, com um sintoma caro: sem o governor em
# performance, o trem de pulsos servo estica sob carga e as RODAS GIRAM SOZINHAS.
#
# Uso (sempre por stdin, do PC):
#   ssh raspberrypi@<host> 'bash -s' < tools/setup_pi.sh            # --check
#   ssh raspberrypi@<host> 'bash -s' -- --apply < tools/setup_pi.sh
#
# Saida: 0 = tudo conforme | 1 = ha desvio aplicavel | 2 = desvio que exige
# reboot ou intervencao manual.

set -uo pipefail

MODO="${1:---check}"
APLICAR=0
[ "$MODO" = "--apply" ] && APLICAR=1

RC=0
MUDOU=0
BACKUP="/var/backups/caramelo/$(date +%Y%m%d_%H%M%S)"

ok()      { printf '  \033[32mOK\033[0m   %s\n' "$*"; }
drift()   { printf '  \033[33mDESVIO\033[0m %s\n' "$*"; [ "$RC" -lt 1 ] && RC=1; }
manual()  { printf '  \033[31mMANUAL\033[0m %s\n' "$*"; RC=2; }
secao()   { printf '\n== %s ==\n' "$*"; }

# Instala um arquivo so' se o conteudo diferir, guardando backup do anterior.
instalar() {
	local destino="$1" modo="$2" conteudo="$3"
	if [ -f "$destino" ] && printf '%s' "$conteudo" | cmp -s - "$destino"; then
		ok "$destino"
		return 0
	fi
	drift "$destino difere do versionado"
	if [ "$APLICAR" = "1" ]; then
		sudo -n mkdir -p "$(dirname "$destino")" "$BACKUP"
		[ -f "$destino" ] && sudo -n cp -a "$destino" "$BACKUP/" 2>/dev/null
		printf '%s' "$conteudo" | sudo -n tee "$destino" >/dev/null
		sudo -n chmod "$modo" "$destino"
		MUDOU=$((MUDOU + 1))
		printf '       aplicado\n'
	fi
}

secao "1. limites de tempo real"
# Sem isto o `chrt -f 50` do hardware_bringup.launch.py falha e o launch QUEBRA
# de proposito — a alternativa seria rodar sem tempo real e mentir que esta ok.
instalar /etc/security/limits.d/99-realtime.conf 0644 'raspberrypi - rtprio 98
raspberrypi - memlock unlimited
'
if [ "$(ulimit -r)" = "98" ]; then ok "ulimit -r = 98"; else manual "ulimit -r = $(ulimit -r); exige nova sessao de login"; fi
if chrt -f 50 true 2>/dev/null; then ok "chrt -f 50 permitido"; else manual "chrt -f 50 negado: o launch vai quebrar"; fi

secao "2. governor de CPU"
# O unit antigo tinha um bug de escape: o systemd expande \$c em ExecStart, e a
# linha virava "echo performance > ; done". O servico falhava em silencio e a Pi
# ficava em ondemand. Esta versao nao usa variavel de shell.
instalar /etc/systemd/system/cpufreq-performance.service 0644 '[Unit]
Description=CPU governor performance (pulsos servo lgpio estaveis)
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
# NAO usar variavel de shell aqui: o systemd expande $c em ExecStart.
ExecStart=/usr/bin/bash -c '"'"'echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null'"'"'

[Install]
WantedBy=multi-user.target
'
GOVS=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort -u | tr '\n' ' ')
if [ "$(echo "$GOVS" | tr -d ' ')" = "performance" ]; then
	ok "governor = performance nos 4 cores"
else
	drift "governor = $GOVS (esperado performance)"
	if [ "$APLICAR" = "1" ]; then
		sudo -n systemctl daemon-reload
		sudo -n systemctl enable --now cpufreq-performance.service >/dev/null 2>&1
		sudo -n systemctl disable --now ondemand.service >/dev/null 2>&1
		MUDOU=$((MUDOU + 1))
	fi
fi

secao "3. throttling de tempo real"
# Com o padrao (950000/1000000) uma thread SCHED_FIFO que usa 100% de um core
# leva um BLACKOUT de 50 ms A CADA SEGUNDO. Medido nesta Pi: pior intervalo
# entre amostras de encoder 49.99 ms com o throttling ligado, 16 us sem ele.
instalar /etc/sysctl.d/99-caramelo-rt.conf 0644 '# A thread de amostragem do encoder roda a ~5.4 MHz num core isolado. Com o
# throttling padrao ela perderia contagem em rajada, e o sintoma apareceria como
# "encoder ruim". Ver quadrature_decoder.hpp.
kernel.sched_rt_runtime_us = -1
'
if [ "$(cat /proc/sys/kernel/sched_rt_runtime_us)" = "-1" ]; then
	ok "sched_rt_runtime_us = -1"
else
	drift "sched_rt_runtime_us = $(cat /proc/sys/kernel/sched_rt_runtime_us)"
	[ "$APLICAR" = "1" ] && { sudo -n sysctl --system >/dev/null 2>&1; MUDOU=$((MUDOU + 1)); }
fi

secao "4. isolamento de core para a amostragem"
# A thread de amostragem consome um core inteiro por projeto. Sem isolcpus ela
# compete com o resto do sistema; e uma thread RT em laco fechado sem core
# proprio TRAVA a maquina (aconteceu: load 14, spawners sem alcancar o
# controller_manager).
if grep -q "isolcpus=3" /boot/firmware/cmdline.txt 2>/dev/null; then
	ok "isolcpus=3 no cmdline.txt"
else
	manual "falta 'isolcpus=3 nohz_full=3 rcu_nocbs=3' no fim da linha de /boot/firmware/cmdline.txt (exige reboot)"
fi
ISOL=$(cat /sys/devices/system/cpu/isolated 2>/dev/null)
if [ "$ISOL" = "3" ]; then ok "core 3 isolado e ativo"; else manual "cores isolados = '${ISOL:-nenhum}' (reboot pendente?)"; fi

secao "5. udev"
# O Ubuntu, ao contrario do Raspberry Pi OS, nao entrega /dev/gpiomem* ao grupo
# gpio — e sem isso a amostragem por MMIO exigiria root.
instalar /etc/udev/rules.d/99-caramelo-gpiomem.rules 0644 '# Acesso ao RP1 GPIO por MMIO (decodificador de quadratura por amostragem).
SUBSYSTEM=="rpi-gpiomem", GROUP="gpio", MODE="0660"
KERNEL=="gpiomem*", GROUP="gpio", MODE="0660"
'
for d in /dev/gpiomem0 /dev/lidar_usb /dev/imu_usb; do
	if [ -e "$d" ]; then ok "$d presente"; else drift "$d ausente"; fi
done
if [ -r /dev/gpiomem0 ] || [ "$(stat -c %G /dev/gpiomem0 2>/dev/null)" = "gpio" ]; then
	ok "/dev/gpiomem0 acessivel ao grupo gpio"
else
	drift "/dev/gpiomem0 sem acesso de grupo"
	[ "$APLICAR" = "1" ] && { sudo -n udevadm control --reload-rules; sudo -n udevadm trigger; MUDOU=$((MUDOU + 1)); }
fi

secao "6. grupos do usuario"
FALTAM=""
for g in gpio dialout i2c spi video; do id -nG | tr ' ' '\n' | grep -qx "$g" || FALTAM="$FALTAM $g"; done
if [ -z "$FALTAM" ]; then
	ok "usuario nos grupos necessarios"
else
	drift "faltam grupos:$FALTAM"
	[ "$APLICAR" = "1" ] && { sudo -n usermod -aG "$(echo "$FALTAM" | tr ' ' ',' | sed 's/^,//')" "$USER"; MUDOU=$((MUDOU + 1)); manual "novos grupos exigem nova sessao"; }
fi

secao "7. relogio (chrony)"
# A Pi 5 desta bancada NAO tem bateria de RTC: sem chrony ela acorda com a hora
# do ultimo desligamento, e um stamp velho faz o Nav2 descartar scans em
# silencio. Ver docs/relogio_chrony.md.
if command -v chronyd >/dev/null 2>&1; then
	ok "chrony instalado"
	instalar /etc/chrony/conf.d/caramelo-notebook.conf 0644 '# O notebook (10.42.0.1) e o servidor NTP da rede do robo, e serve hora mesmo
# sem internet (local stratum 10 no lado dele). Ver docs/relogio_chrony.md.
server 10.42.0.1 iburst prefer minpoll 4 maxpoll 6
'
	# makestep TEM que estar no chrony.conf principal: o confdir e' lido ANTES,
	# entao o "makestep 1 3" default sobrescreveria um drop-in.
	if grep -qx "makestep 1.0 -1" /etc/chrony/chrony.conf 2>/dev/null; then
		ok "makestep 1.0 -1 no chrony.conf principal"
	else
		drift "falta 'makestep 1.0 -1' no /etc/chrony/chrony.conf"
		if [ "$APLICAR" = "1" ]; then
			sudo -n sed -i 's/^makestep .*/makestep 1.0 -1/' /etc/chrony/chrony.conf
			grep -qx "makestep 1.0 -1" /etc/chrony/chrony.conf || \
				echo "makestep 1.0 -1" | sudo -n tee -a /etc/chrony/chrony.conf >/dev/null
			sudo -n systemctl restart chrony
			MUDOU=$((MUDOU + 1))
		fi
	fi
	# Os "pool" do Ubuntu comentados: na competicao nao ha internet, e o
	# docs/relogio_chrony.md manda deixar o notebook como fonte UNICA para nao
	# haver briga de selecao. Encontrado ATIVO em 2026-09-08 — a Pi tinha 4 pools
	# alem do notebook.
	if grep -q '^pool ' /etc/chrony/chrony.conf 2>/dev/null; then
		drift "'pool' ativo no chrony.conf (o notebook deve ser a fonte unica)"
		if [ "$APLICAR" = "1" ]; then
			sudo -n sed -i 's/^pool /#pool /' /etc/chrony/chrony.conf
			sudo -n systemctl restart chrony
			MUDOU=$((MUDOU + 1))
		fi
	else
		ok "pools do Ubuntu comentados (notebook e' a fonte unica)"
	fi

	# chrony-wait: a Pi NAO tem bateria de RTC e acorda com a hora do ultimo
	# desligamento. Medido em 2026-09-08: o chronyd registrou "System clock wrong
	# by 535917 seconds" e deu um STEP de 6,2 DIAS depois de alcancar o notebook.
	# Se o bringup subir antes desse passo, os nos nascem com a hora errada e
	# levam o salto EM VOO: stamps de /scan e /odom pulam, a TF extrapola e o
	# Nav2 descarta mensagem em silencio. chrony-wait.service atrasa
	# time-sync.target ate o relogio fechar, dando um alvo de ordenacao para
	# qualquer unidade e um sinal para o run_bringup.sh.
	if systemctl is-enabled chrony-wait >/dev/null 2>&1; then
		ok "chrony-wait habilitado (nada roda antes do passo do relogio)"
	else
		drift "chrony-wait DESABILITADO: o bringup pode subir antes do step de relogio"
		if [ "$APLICAR" = "1" ]; then
			sudo -n systemctl enable chrony-wait >/dev/null 2>&1
			MUDOU=$((MUDOU + 1))
		fi
	fi

	# A fonte selecionada tem que ser o notebook (^*), nao um pool da internet.
	if chronyc -n sources 2>/dev/null | grep -q '^\^\*.*10\.42\.0\.1'; then
		ok "fonte selecionada = 10.42.0.1 (notebook)"
	else
		drift "a fonte selecionada NAO e' 10.42.0.1: $(chronyc -n sources 2>/dev/null | awk '/^\^\*/{print $2}')"
	fi

	# Um drop-in de bancada sobrevivente e' erro: ele mente que a hora esta boa.
	if ls /etc/chrony/conf.d/zz-bancada-*.conf >/dev/null 2>&1; then
		manual "drop-in de BANCADA ainda instalado: $(ls /etc/chrony/conf.d/zz-bancada-*.conf) — remover antes da missao"
	fi
else
	manual "chrony NAO instalado (precisa de internet: sudo apt install chrony)"
fi
timedatectl | grep -q "System clock synchronized: yes" && ok "relogio sincronizado" || drift "relogio nao sincronizado"

secao "8. config.txt"
grep -q "usb_max_current_enable=1" /boot/firmware/config.txt 2>/dev/null \
	&& ok "usb_max_current_enable=1" \
	|| manual "falta usb_max_current_enable=1 em /boot/firmware/config.txt (LiDAR pode ficar limitado a 600 mA)"

printf '\n== resumo ==\n'
printf 'modo: %s | mudancas aplicadas: %d\n' "$MODO" "$MUDOU"
case "$RC" in
	0) printf 'conforme\n' ;;
	1) printf 'ha desvio aplicavel: rode com --apply\n' ;;
	2) printf 'ha desvio que exige reboot ou intervencao manual (ver linhas MANUAL)\n' ;;
esac
exit "$RC"
