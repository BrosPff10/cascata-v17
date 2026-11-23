# Cascata v17 — Flight Controller para ESP32 + MPU6050

Firmware de controle de atitude para quadricóptero (VANT) de baixo custo, desenvolvido
para o Trabalho de Conclusão de Curso. Roda em um ESP32 DevKit e usa uma malha de
**cascata** — `ângulo → taxa → largura de pulso (µs)` — com **telemetria em tempo real**
página web servida pelo próprio ESP32.

> **Aviso:** este firmware foi validado em **bancada (1 DOF)**, com o drone preso a um
> suporte de segurança. Ele **não é um firmware de voo livre validado**. Réplicas são por
> conta e risco de quem for montar — incluindo a parte física (solda, conexões, direção
> das hélices, balanceamento).

## Recursos

- PID em cascata: `PAngle` (laço externo de ângulo) → `PRate`/`IRate`/`DRate` (laço interno de taxa) + yaw por taxa de giro
- Laço de controle a **250 Hz** (4000 µs), telemetria via WebSocket a **20 Hz**
- **Calibração automática da IMU** na ligação (~4 s) — deixe o drone parado e nivelado
- **Failsafe por heartbeat**: o navegador envia "HB" a cada 200 ms; se faltar comando
  por **500 ms** (Wi-Fi caiu, aba fechada), o controlador **desarma sozinho**
- Armamento condicionado: IMU calibrada + drone nivelado (`|roll|` e `|pitch|` ≤ 5°)
- Página web com telemetria (roll, PID, saída dos ESCs em µs, vibração) e **ajuste de ganhos em tempo real**
- **Gravação de voo**: 60 s a 25 Hz (1500 amostras) na memória, baixável pelo navegador
- **Diagnóstico do loop**: tempo de ciclo, pico, jitter e contagem de estouros (prova de que o ESP32 aguenta os 250 Hz) — tudo no endpoint `/diag.txt`

## Hardware

| Item | Detalhe |
|---|---|
| Placa | ESP32 DevKit |
| IMU | MPU6050, endereço `0x68`, I2C nos GPIO **21 (SDA)** / **22 (SCL)** |
| ESC ×4 | PWM **400 Hz**, faixa 1000–2000 µs — GPIO **19 (A)**, **18 (B)**, **33 (C)**, **32 (D)** |
| Motores | 4× brushless (ex.: A2212 1000 KV) + ESC (ex.: HW-30A) |
| Hélices | 1045 (usadas em todos os testes do TCC) |
| Energia | Bateria LiPo + ESCs (alimentação dos ESCs **não** sai do ESP32) |

A montagem física (solda, pinagem, direção de rotação das hélices) fica a cargo de
quem for replicar.

## Como montar (lado do software)

1. Instale o **Arduino IDE 2.x**.
2. Instale o suporte à placa: *Boards Manager* → procure **"esp32 by Espressif Systems"** → instale.
3. Instale as bibliotecas (*Library Manager*):
   - **ESP32Servo**
   - **ESP Async WebServer** (ESP32Async) e **AsyncTCP**
   - **MPU6050** (I2Cdev / Jeff Rowberg — fornece o `<MPU6050.h>`)
4. Abra o sketch: `cascata-v17/cascata-v17.ino`.
5. *(Opcional)* Troque a rede Wi-Fi nas linhas `ssid`/`password` (padrão: **Drone** / **12345678**).
6. Selecione a placa: *Tools → Board → ESP32 Dev Module*, escolha a porta COM.
7. Clique em **Verificar** e depois em **Enviar**.

## Como usar

1. Ligue o drone na bateria. Abra o **Monitor Serial** (115200 bauds): a calibração da
   IMU roda em ~4 s — **não mova o drone** durante esse tempo.
2. No celular ou no PC, conecte-se à rede Wi-Fi **Drone** (senha `12345678`).
   O ESP32 é o ponto de acesso: o endereço padrão é **http://192.168.4.1**
3. Abra o endereço no navegador. A página mostra a telemetria e os controles
   (Armar / Desarmar / Calibrar / limite de acelerador / ganhos do PID).
4. **Regras de segurança:** só arme com o drone nivelado; solte o controle e ele
   desarma sozinho em 500 ms; teste sempre com o drone preso a um suporte.

Endpoints HTTP disponíveis: `/` (interface), `/arm`, `/disarm`, `/calibrar`,
`/resetstats`, `/diag.txt` (relatório de diagnóstico).

## Ganhos

Valores **padrão no código** (linhas `PAngle`/`PRate`/`IRate`/`DRate`):

```
PAngle = 2.5   PRate = 1.5   IRate = 0.5   DRate = 0.01   yaw: Kp 1.5 / Ki 0
```

Sintonia usada no **Teste 8** dos ensaios do TCC (ajustada pela página web):
`PAngle = 1.50`, `PRate = 1.20`, `IRate = 0.05`, `DRate = 0.05`.

Proteções do laço: anti-windup (`ITERM_LIM = 250`), limite de taxa desejada
(`RATE_LIM = 500 °/s`), termo D com passa-baixa e teto (`clamp`) na saída final.
O mixer trava cada motor em **1100–2000 µs** — nenhum valor do PID ultrapassa isso.

## Créditos e licença

- **MIT © 2026 André Prieto França Filho** (ver [`LICENSE`](LICENSE)).
- Baseado em [pratikPhadte/ESP32-Flight-controller-](https://github.com/pratikPhadte/ESP32-Flight-controller-) — licença **MIT** (obrigatório manter o crédito; está aqui).
- A arquitetura de cascata foi validada em bancada contra
  [dRehmFlight](https://github.com/nickrehm/dRehmFlight) de Nicholas Rehm (**GPLv3**) —
  o port usado na comparação está no repositório
  [`dRehmFlight-ESP32`](../dRehmFlight-ESP32) (quando publicado, o link será o do GitHub).
