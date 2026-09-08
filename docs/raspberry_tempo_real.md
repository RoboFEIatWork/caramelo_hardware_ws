# Configuração de Tempo Real no Raspberry Pi (Caramelo)

> ⚠️ **OBRIGATÓRIO EM CADA Pi NOVA/REINSTALADA — não migra com o workspace!**
> Na troca rasp4→rasp5 (07/2026) esta config ficou para trás e o resultado foi
> grave: na Pi 5 os pulsos servo dos ESCs são gerados por thread de SOFTWARE
> (lgpio; a pigpio/DMA não existe mais), e sem RT a borda de descida ATRASA sob
> carga de CPU → pulso mais longo → roda de FRENTE acelera (~2×!) e a de RÉ
> desacelera. Provado em bancada 27-29/07/2026 (comando 3,6 rad/s: direitas
> foram a 8-9 rad/s durante um `colcon build`). Com RT aplicado: variação ≤8%.
> Status: **aplicado na rasp5 em 2026-07-29**. O `chrt -f 50` no launch foi
> SUBSTITUÍDO em 2026-09-08 por RT dirigido por thread — ver a seção
> "Tempo real POR THREAD" abaixo, que é o que vale hoje.

O `controller_manager` tenta criar a thread de controle com política FIFO e prioridade 50.
Sem permissão, aparece no bringup:

```
[WARN] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted).
```

Sem RT o loop de 100 Hz funciona, mas fica sujeito a jitter quando o sistema carrega
(rede, EKF, LiDAR). A configuração abaixo é feita **uma única vez, à mão, no Pi**.

## Passo a passo (uma vez, no Raspberry)

1. Criar o arquivo de limites (usuário `raspberrypi`):

   ```bash
   sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null <<'EOF'
   raspberrypi - rtprio 98
   raspberrypi - memlock unlimited
   EOF
   ```

2. Reiniciar o Pi (ou fazer logout/login completo da sessão SSH):

   ```bash
   sudo reboot
   ```

3. Verificar depois do login:

   ```bash
   ulimit -r   # deve mostrar 98
   ```

4. Rodar o bringup e confirmar que o WARN de "FIFO RT scheduling" sumiu:

   ```bash
   ros2 launch raspberry_bringup hardware_bringup.launch.py
   ```

## Governor de CPU em performance (recomendado, feito na rasp5)

Evita que o kernel abaixe o clock durante a operação (reduz jitter). Sem
internet na Pi, sem cpufrequtils — usar serviço systemd próprio (foi o feito
em 2026-07-29):

```bash
sudo tee /etc/systemd/system/cpufreq-performance.service > /dev/null <<'EOF'
[Unit]
Description=CPU governor performance (pulsos servo lgpio estaveis)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for c in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > $c; done'

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload && sudo systemctl enable --now cpufreq-performance.service
# Ubuntu tem um servico legado que RESETA para ondemand depois do boot:
sudo systemctl disable --now ondemand.service
```

## Tempo real POR THREAD (2026-09-08 — substituiu o `chrt` no processo)

> Até 08/09/2026 o `hardware_bringup.launch.py` subia o `ros2_control_node` com
> `prefix=["chrt -f 50"]`. **Isso foi removido.** O RT agora é aplicado dentro
> do driver, só nas threads que precisam.

### Por que mudou

O `chrt` no processo colocava **as 35 threads** em `SCHED_FIFO 50` — incluindo
as **nove threads de DDS** (`dds.udp.*`, `dds.shm.*`, `dds.ev.0`,
`dds.asyn.0.0`). Threads de rede em prioridade de tempo real preemptam tudo que
for prioridade normal no mesmo core.

Medido nesta bancada em 08/09/2026, robô com rodas suspensas:

| | bringup sozinho | com o stack do PC no ar |
|---|---|---|
| Estouros de deadline do `ekf_node` | 1 em 4 min | **21 em 2 min** |
| Pior ciclo (orçamento 25 ms) | 26 ms | **152 ms** |
| Ociosidade dos cores 0/1/2 | — | **68%** |

O EKF não estava sem CPU: estava sendo **preemptado**. Assim que os assinantes
do PC acordaram as threads de rede da Pi, elas passaram a roubar a latência do
EKF em prioridade de tempo real.

### Por que isso importa muito mais que o EKF

Duas condições desta Pi transformam o problema acima em travamento:

1. `kernel.sched_rt_runtime_us = -1` (seção 4 do `setup_pi.sh`) — **não há teto
   de CPU para threads RT**. É necessário para a thread de amostragem não levar
   50 ms/s de blackout, mas remove a válvula de escape do resto.
2. O kernel reporta no boot:
   ```
   watchdog: Delayed init of the lockup detector failed: -19
   watchdog: Hard watchdog permanently disabled
   ```
   Um core tomado por thread de tempo real **não gera log e não se recupera**.

Isso é exatamente a assinatura de *"a Raspberry para de responder e congela"* —
o sintoma que vinha sendo atribuído a vazamento de memória. Medição de
16 min / 177 amostras com o stack completo no ar: RSS dos 7 processos
**idêntica** do início ao fim, fds e threads constantes, **zero** menção de OOM
no `dmesg`. Com 16 GB e sem swap, um vazamento seria morto pelo OOM killer (que
deixa rastro) em vez de congelar — "congelou sem rastro" é evidência *contra* a
hipótese de memória. Ver `tools/remote/memsnap.sh`.

### O que roda em tempo real agora

Aplicado pelo driver (`<param name="control_rt_priority">`, default 50, no
`mobile_base.ros2_control.xacro`):

| thread | política | por quê |
|---|---|---|
| amostrador do encoder | FIFO **80**, core 3 isolado | perder amostra é pior que atrasar um PWM |
| `control_loop` | FIFO 50 | fala com o GPIO a 100 Hz |
| tx do lgpio | FIFO 50 | pulso preemptado **estica** = comando errado |
| update do `controller_manager` | FIFO 50 | o próprio CM já faz isso |
| failsafe | FIFO 10 | quase não roda; só precisa não ser esquecido |
| **DDS, executor, resto** | **SCHED_OTHER** | é onde devem estar |

A thread de tx do lgpio não tem handle nem nome — a biblioteca não expõe nada.
O driver a descobre por **diferença de `/proc/self/task`** antes e depois de
inicializar o lgpio. Se aparecer mais de 2 threads novas nessa janela ele não
eleva nenhuma e diz por quê: promover uma thread aleatória a tempo real é pior
que não promover.

Resultado da mesma medição depois da mudança: **3 estouros** (contra 21), pior
ciclo **41 ms** (contra 152), e **zero** nos 60 s seguintes — os 3 foram uma
rajada única no instante em que o Nav2 ativa os 9 servidores.

### Como verificar

`tools/remote/check_stack.sh` checa o critério certo: algumas poucas threads em
`FF` e **nenhuma `dds.*` em `FF`**. Se aparecer DDS em tempo real, alguém
reintroduziu o `chrt` no processo.

Continua dependendo do `limits.d` acima. A diferença é que agora, sem o limite,
o driver **avisa e segue em prioridade normal** em vez de o launch inteiro
quebrar no `chrt`.

## Problemas conhecidos (não corrigir localmente)

- **Ctrl-C no bringup às vezes NÃO mata os nós** (observado 2026-07-29 com
  FIFO): ficam zumbis segurando GPIOs ("Device or resource busy") e — o caso
  mais traiçoeiro — um `robot_state_publisher` zumbi continua servindo o
  **URDF VELHO** no tópico latched `/robot_description`, e o
  `controller_manager` NOVO pode engolir o URDF do zumbi na subida (parâmetros
  novos do xacro "não fazem efeito" sem nenhum erro; custou horas de
  diagnóstico no dia da calibração do trim). **Depois de todo Ctrl-C**:
  ```bash
  pkill -f robot_state_publisher; pkill -f ros2_control_node; \
  pkill -f ekf_node; pkill -f "ros2 launch"; pkill -f wit_ros2_imu
  ps -eo pid,cmd | grep -E "robot_state|ros2_control|ekf|launch" | grep -v grep
  ```
  (a última linha deve voltar vazia). Duplo-publisher detectável do PC com
  `ros2 topic info /robot_description -v | grep "Publisher count"` (deve ser 1).
  Seguro matar: o driver tem shutdown ordenado e o firmware dos ESCs para os
  motores se os pulsos sumirem.

- **`wit_ros2_imu` lança traceback no Ctrl+C** (`TypeError: 'NoneType' object cannot
  be interpreted as an integer` + `RCLError: failed to shutdown`). É um bug de
  encerramento do pacote de terceiros (`~/ros2_ws`): a thread de leitura serial
  corre contra o fechamento da porta, e o `main()` chama `rclpy.shutdown()` num
  contexto já finalizado. **Inofensivo** — acontece só no desligamento, não afeta
  a operação. NÃO editar o pacote localmente: ele é instalado do GitHub e é o
  mesmo em todos os PCs/robôs. Se incomodar, abrir PR no repositório upstream.
