#!/bin/bash
# Um passo de calibracao de odometria: comanda um movimento e reporta o que a
# odometria de RODA achou que andou.
#
# Uso (por stdin, com argumentos depois de --):
#   ssh raspberrypi@<host> 'bash -s' -- 0.15 0 0 6 < tools/remote/calib_move.sh
#                                        vx  vy wz segundos
#
# Le /odom/wheel, NAO /odom. Esta e' a regra de ouro da calibracao: /odom ja
# passou pelo EKF, que funde a IMU e suaviza — calibrar contra ele e' calibrar
# contra o filtro, nao contra as rodas.
#
# SEGURANCA: com o robo no chao ele VAI se deslocar. vx=0.15 por 6 s da ~0,9 m.
# Confira o espaco livre antes.

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

VX="${1:-0.15}"; VY="${2:-0.0}"; WZ="${3:-0.0}"; DUR="${4:-6}"

if ! timeout 15 ros2 control list_controllers 2>/dev/null | grep -q "mecanum_controller.*active"; then
	echo "ABORTANDO: mecanum_controller nao esta ativo. Suba o bringup antes."
	exit 3
fi

captura() {
	timeout -s INT 10 ros2 topic echo /odom/wheel --field pose.pose --once 2>/dev/null
}

echo "=== pose inicial (/odom/wheel) ==="
ANTES=$(captura)
echo "$ANTES" | head -12

# O topic pub nao carimba o header; o controlador avisa e usa o tempo atual.
# --times limita a rajada: sem isso, um Ctrl-C deixaria o ultimo comando valendo.
CICLOS=$(python3 -c "print(int($DUR*20))")
echo
echo "=== comandando vx=$VX vy=$VY wz=$WZ por ${DUR}s ($CICLOS mensagens a 20 Hz) ==="
timeout "$((DUR + 6))" ros2 topic pub -r 20 --times "$CICLOS" \
	/mecanum_controller/reference geometry_msgs/msg/TwistStamped \
	"{header: {frame_id: base_footprint}, twist: {linear: {x: $VX, y: $VY}, angular: {z: $WZ}}}" \
	>/dev/null 2>&1

# O reference_timeout do controlador (0,15 s) ja para os motores sozinho quando
# a publicacao acaba; a espera aqui e' para a inercia mecanica assentar.
sleep 3

echo "=== pose final (/odom/wheel) ==="
DEPOIS=$(captura)
echo "$DEPOIS" | head -12

echo
python3 - "$ANTES" "$DEPOIS" <<'PY'
import math, re, sys

def ler(txt):
	def num(campo, bloco):
		m = re.search(rf'{campo}:\s*(-?[\d.eE+-]+)', bloco)
		return float(m.group(1)) if m else 0.0
	pos = txt.split('orientation')[0]
	ori = 'orientation' + txt.split('orientation')[1] if 'orientation' in txt else ''
	return (num('x', pos), num('y', pos),
	        num('z', ori), num('w', ori))

ax, ay, az, aw = ler(sys.argv[1])
bx, by, bz, bw = ler(sys.argv[2])
dx, dy = bx - ax, by - ay
yaw_a = 2.0 * math.atan2(az, aw)
yaw_b = 2.0 * math.atan2(bz, bw)
dyaw = math.atan2(math.sin(yaw_b - yaw_a), math.cos(yaw_b - yaw_a))

print("=== o que a ODOMETRIA DE RODA registrou ===")
print(f"  deslocamento x : {dx:+.4f} m")
print(f"  deslocamento y : {dy:+.4f} m")
print(f"  distancia      : {math.hypot(dx, dy):.4f} m")
print(f"  rotacao        : {dyaw:+.4f} rad = {math.degrees(dyaw):+.2f} graus")
print()
print("Agora meca o REAL com trena/transferidor e compare:")
print("  Etapa 1 (raio):    wheels_radius_novo = atual * d_real / d_odom")
print("  Etapa 2 (lx+ly):   soma_nova = soma_atual * theta_odom / theta_real")
PY
