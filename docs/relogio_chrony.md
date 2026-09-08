# Relógio do robô: chrony (notebook = servidor NTP da Pi)

A Pi 4 não tem RTC: acorda com hora podre e o timesyncd só corrige com
internet, aos trancos (skew de 606 ms já quebrou AMCL/collision_monitor —
scans com stamp velho são descartados e a TF extrapola). Solução: o notebook
serve a hora na rede do robô (10.42.0.0/24) mesmo SEM internet, e a Pi
sincroniza nele com passo (step) agressivo no boot.

## Notebook (servidor) — JÁ CONFIGURADO (2026-07)

`/etc/chrony/conf.d/caramelo-ntp-server.conf`:

```
allow 10.42.0.0/24
local stratum 10
```

`local stratum 10` = continua servindo a hora mesmo sem upstream (offline na
competição). Verificar: `chronyc tracking` (deve mostrar Reference ID de um
pool OU 127.127.1.1 quando offline).

## Pi (cliente) — uma vez, com sudo

```bash
sudo apt install -y chrony            # desativa o systemd-timesyncd sozinho
sudo tee /etc/chrony/conf.d/caramelo-notebook.conf > /dev/null <<'EOF'
server 10.42.0.1 iburst prefer minpoll 4 maxpoll 6
EOF
sudo sed -i 's/^pool /#pool /' /etc/chrony/chrony.conf
sudo sed -i 's/^makestep .*/makestep 1.0 -1/' /etc/chrony/chrony.conf
sudo systemctl restart chrony
```

- `pool` comentados: o notebook é a ÚNICA fonte (sem briga de seleção quando
  não há internet).
- `makestep 1.0 -1`: SEMPRE dá step quando o offset passa de 1 s (essencial no
  boot da Pi, que acorda com a hora do último desligamento). Tem que ser no
  chrony.conf principal: o confdir é lido ANTES, e o `makestep 1 3` default
  sobrescreveria um drop-in.

## Validação

1. Na Pi: `chronyc sources -v` → o `10.42.0.1` com `^*` (fonte selecionada);
   `chronyc tracking` → `System time` < 10 ms.
2. Reboot da Pi (`sudo poweroff` + energia, como sempre) → repetir o passo 1:
   o offset deve fechar < 10 ms em ~1 min (iburst) mesmo sem internet.
3. Fim-a-fim com o bringup no ar: stamp do `/scan` vs relógio do PC < 50 ms
   sustentado (script `skew_check.py` das sessões de navegação).

## Auditoria de 2026-09-08 — o que estava fora do padrão

O chrony em si estava **saudável**: offset de −42 µs para o notebook, `^*` no
`10.42.0.1`, `systemd-timesyncd` inativo. Dois desvios, ambos agora checados
pelo `tools/setup_pi.sh`:

1. **Os 4 `pool` do Ubuntu continuavam ATIVOS** no `chrony.conf`, apesar da
   instrução acima de comentá-los. Sem internet na competição eles só
   atrapalham a seleção de fonte.
2. **`chrony-wait.service` estava desabilitado.** Nada atrasa o
   `time-sync.target`, então nada impede o bringup de subir antes do passo.

### Por que o item 2 é o que importa

O journal da Pi registrou, no boot de 08/09:

```
chronyd: System clock wrong by 535917.439891 seconds
chronyd: System clock was stepped by 535917.439891 seconds
```

Um passo de **6,2 dias**, aplicado assim que ela alcançou o notebook. O chrony
fez a coisa certa — o problema é a **ordem**. Se o bringup subir antes desse
passo, os nós nascem com a hora do último desligamento e levam o salto **em
voo**: os stamps de `/scan` e `/odom` pulam para o futuro, a TF extrapola e o
Nav2 passa a descartar mensagem **em silêncio**. É o modo de falha mais caro
daqui, porque não produz erro nenhum — só "o robô não vê o obstáculo".

O `tools/remote/run_bringup.sh` passou a **recusar subir** com o relógio
dessincronizado (espera até 30 s; escape por `CARAMELO_IGNORA_RELOGIO=1`).
Quem sobe o bringup à mão deve conferir antes:

```bash
chronyc tracking | grep -E "Reference ID|Leap status|System time"
```

`Reference ID` não pode ser `00000000` (não sincronizado) nem `7F7F01xx`
(referência local).

## Checagem de rotina (antes de missão)

`chronyc tracking` na Pi substitui o antigo "restart do timesyncd". Se
`System time` > 50 ms ou a fonte não for `10.42.0.1`, investigar ANTES de
navegar (AMCL descarta scan velho em silêncio).
