/* =====================================================================
   ESP32_DRONE_DIAGNOSTICO - Flight Controller (ESP32 + MPU6050)
   Baseado em: pratikPhadte/ESP32-Flight-controller-
   ---------------------------------------------------------------------
   HARDWARE: ESP32, MPU6050 (0x68, I2C 21/22), 4 ESC + 4 brushless.
   SEGURANCA:
     - Failsafe por heartbeat: sem mensagem do navegador por 500 ms
       (WiFi caiu, aba fechada, celular dormiu) => DESARM automatico.
     - ARM so com IMU calibrada, sensor respondendo e drone nivelado
       (|roll| e |pitch| <= 5 graus).
     - Disarm zera TODOS os integradores/erros do PID.
   DIAGNOSTICO (prova de que o ESP32 aguenta o loop):
     - cicloUs / cicloMaxUs: tempo de execucao do loop (esperado < 4000).
     - jitterUs: maior atraso ja medido sobre a agenda de 250 Hz.
     - estouros: quantas vezes o loop passou de 4000 us.
   ===================================================================== */

#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <math.h>
#include <esp_system.h>   // esp_reset_reason() - o core nao tem mais ESP.getResetReason()

// ----------------------------- REDE -----------------------------
const char* ssid = "Drone";
const char* password = "12345678";

// ----------------------------- PINOS / ESC -----------------------------
constexpr int ESC_A = 19;
constexpr int ESC_B = 18;
constexpr int ESC_C = 33;
constexpr int ESC_D = 32;

constexpr int MOTOR_MIN  = 1000;
constexpr int MOTOR_IDLE = 1100;  // piso: so mantem as helices girando em vazio
constexpr int MOTOR_MAX  = 2000;  // teto real do ESC (a biblioteca limita a 1000..2000)
constexpr int THROTTLE_BASE = 1000; // valor AO LIGAR = parado (subir no slider antes de armar)
constexpr int ESC_FREQ = 400;       // Hz do PWM do ESC (mude p/ 500 se o seu ESC aceitar)

// ----------------------------- TIMING -----------------------------
constexpr uint32_t PERIOD_US     = 4000;   // 250 Hz
constexpr uint32_t TELEMETRY_US  = 50000;  // 20 Hz no WebSocket
constexpr uint32_t FAILSAFE_MS   = 500;    // sem heartbeat => disarm
constexpr uint32_t HEARTBEAT_JS  = 200;    // navegador manda "HB" a cada 200 ms

// ----------------------------- VERSAO (fonte unica) -----------------------------
// Usada no DIAGNOSTICO (linha 3 do arquivo) e no cracha do SITE (canto superior
// direito, enviado no connect). Trocou a versao? Mude AQUI - os dois lugares
// mudam juntos.
const char FW_VER[] = "v17 teto ABS na saida; D filtrado; I esvazia";

// ----------------------------- SEGURANCA / GANHOS -----------------------------
constexpr float MAX_ANGLE_ARM = 5.0f;  // graus, condicao minima para armar
constexpr float YAW_SIGN      = 1.0f;  // inverte se o yaw reagir ao contrario

// PID em CASCATA (mesma arquitetura da referencia): angulo -> taxa -> µs
//   PAngle: graus/s de taxa desejada por grau de erro de angulo  (laço externo)
//   PRate : µs de motor por grau/s de erro de taxa               (laço interno)
//   IRate : integral do laço de taxa (corrige erro persistente)
//   DRate : amortecedor (reage a aceleracao angular)
// SEM limite de saida: o PID pode usar toda a faixa disponivel - o unico limite
// que resta e o proprio hardware (o mixer final trava cada motor em 1100..2000 µs).
// O clamp antigo de ±400 µs travava a correcao e impedia o drone de voltar ao nivel.
float PAngle = 2.5f, PRate = 1.5f, IRate = 0.5f, DRate = 0.01f;
float KpYaw = 1.5f, KiYaw = 0.0f;  // yaw: laço de taxa puro (sem magnetometro)

constexpr float ITERM_LIM = 250.0f; // anti-windup: menos energia guardada ao segurar inclinado
constexpr float RATE_LIM  = 500.0f; // taxa desejada maxima (graus/s)

// ----------------------------- OBJETOS -----------------------------
Servo motorA, motorB, motorC, motorD;
MPU6050 mpu(0x68);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ----------------------------- ESTADO -----------------------------
volatile bool armado = false;
volatile bool pedirCalibracao = false;
bool sensorOK = false;
bool calibrado = false;
bool initialized = false;
volatile int throttleBase = THROTTLE_BASE; // slider da web: 1000..2000 µs
volatile int throttleMax = 1500;           // teto AO LIGAR (seguranca; slider "Max. throttle" web)

// ----------------------------- IMU -----------------------------
int32_t gxOffset = 0, gyOffset = 0, gzOffset = 0;
int32_t accXOffset = 0, accYOffset = 0, accZOffset = 0;
constexpr float LSB_GYRO  = 131.0f;   // +/-250 dps
constexpr float LSB_ACCEL = 16384.0f; // +/-2g

float roll = 0, pitch = 0;
float gyroXf = 0, gyroYf = 0, gyroZf = 0;
int falhasLeitura = 0;               // leituras I2C falhando em sequencia
uint32_t ultimaTentativaIMU = 0;     // auto-recuperacao da IMU (loop)

// Vibracao estrutural: desvio da magnitude da aceleracao em relacao a 1 g
// (parado ideal = 0). RMS via EMA alpha=0.01 por amostra + pico desde o
// ultimo reset. MESMA definicao usada no dRehmFlight_ESP32 para comparacao.
float vibRms = 0, vibPeak = 0;

// ----------------------------- PID (cascata) -----------------------------
float erroRoll = 0, erroPitch = 0;               // erro de ANGULO (graus)
float intRoll = 0, intPitch = 0, intYaw = 0;     // integral do laço de TAXA
float prevErrRateRoll = 0, prevErrRatePitch = 0; // estado do termo D
float dFiltRoll = 0, dFiltPitch = 0;            // D apos passa-baixa (anti auto-excitacao)
float pidRoll = 0, pidPitch = 0, pidYaw = 0;      // saida final (µs)

// ----------------------------- MOTORES -----------------------------
int motorSpeedA = MOTOR_MIN, motorSpeedB = MOTOR_MIN;
int motorSpeedC = MOTOR_MIN, motorSpeedD = MOTOR_MIN;

// ----------------------------- TIMING / DIAGNOSTICO -----------------------------
uint32_t nextTick = 0, previousSample = 0, lastTelemetry = 0;
uint32_t cicloUs = 0, cicloMaxUs = 0, jitterUs = 0, estouros = 0;
uint32_t ultimoHeartbeat = 0;

// ============================ DIAGNOSTICO ============================
// Log de eventos (anel): o que aconteceu durante o teste, com timestamp
struct Evento { uint32_t ms; char txt[56]; };
constexpr int MAX_EVENTOS = 32;
Evento eventosLog[MAX_EVENTOS];
int eventoIdx = 0;              // proxima posicao do anel
uint32_t nEventos = 0;          // total ja registrados (pode passar de 32)

// Contadores de sessao (mostrados no /diag.txt)
uint32_t contCiclos = 0, contArmamentos = 0, contFailsafes = 0;
uint32_t contReconexoesIMU = 0, contCalibOk = 0, contCalibErro = 0;
uint32_t contWsConect = 0, contWsDesconect = 0, contHeartbeats = 0;
uint32_t contAjustePID = 0, contAjusteTHR = 0;

// Perfil de tempo por trecho do ciclo (para achar para onde vao os ~5 ms)
uint32_t tCicloSoma = 0, tCicloN = 0;    // ciclos "normais" (exclui o da calibracao)
uint32_t tIMUmax = 0, tIMUsum = 0;       // leitura I2C da IMU
uint32_t tCtrlmax = 0, tCtrlsum = 0;     // PID + mixer + ESC + gravacao
uint32_t tEscmax = 0, tEscsum = 0;       // so as 4 escritas de ESC (parte do ctrl)
uint32_t benchEscUs = 0;                 // bench de escrita no setup (antes do Wi-Fi)
char escVerif[110] = "nao executado";    // duty_rd (hardware) vs ticks esperados
uint32_t tTelmax = 0, tTelsum = 0;       // snprintf + envio WebSocket
bool cicloCalibrando = false;            // ciclo contem calibracao de 4 s: nao conta

// Gravacao de voo: 25 Hz POR TEMPO enquanto armado, guarda a ULTIMA subida
// 1500 amostras x 28 bytes = 42 KB = 60 s de voo (agenda de 40 ms, independe
// da taxa real do loop)
struct Amostra {
  uint16_t tms;                 // ms desde o inicio da gravacao
  int16_t roll, pitch;          // centesimos de grau
  int16_t gx, gy, gz;           // decimos de graus/s
  int16_t pidR, pidP, pidY;     // us
  int16_t mA, mB, mC, mD;       // us
  int16_t vib;                  // mili-g do RMS de vibracao (|a|-1g)
};
constexpr int MAX_AMOSTRAS = 1500;
Amostra vooLog[MAX_AMOSTRAS];
uint16_t vooN = 0;              // amostras gravadas
bool vooAtivo = false;          // gravando agora
bool vooTruncado = false;       // encheu o buffer (60 s)
constexpr uint32_t VOO_AMOSTRA_US = 40000;  // 40 ms = 25 Hz => 1500 amostras = 60 s
uint32_t vooUltima = 0;         // agenda da proxima amostra (avanca 40 ms por amostra)
uint32_t vooT0 = 0;

void logEvento(const char* txt) {
  Evento& e = eventosLog[eventoIdx];
  e.ms = millis();
  strncpy(e.txt, txt, sizeof(e.txt) - 1);
  e.txt[sizeof(e.txt) - 1] = '\0';
  eventoIdx = (eventoIdx + 1) % MAX_EVENTOS;
  nEventos++;
}

// =====================================================================
//                              FUNCOES BASE
// =====================================================================
void setMotors(int v) {
  motorA.writeMicroseconds(v);
  motorB.writeMicroseconds(v);
  motorC.writeMicroseconds(v);
  motorD.writeMicroseconds(v);
}

void desarmar(const char* motivo = "manual") {
  bool estava = armado;
  armado = false;
  intRoll = intPitch = intYaw = 0;
  erroRoll = erroPitch = 0;
  pidRoll = pidPitch = pidYaw = 0;
  motorSpeedA = motorSpeedB = motorSpeedC = motorSpeedD = MOTOR_MIN;
  setMotors(MOTOR_MIN);
  if (estava) {
    vooAtivo = false;                 // congela a gravacao (mantem os dados)
    char b[70];
    snprintf(b, sizeof(b), "DESARM: %s", motivo);
    logEvento(b);
  }
}

const char* podeArmar() {
  if (!sensorOK)                    return "MPU6050 indisponivel";
  if (!calibrado)                   return "Calibre o IMU antes (botao CALIBRAR)";
  if (!initialized)                 return "Aguardando primeiro angulo valido";
  if (fabsf(roll)  > MAX_ANGLE_ARM) return "Nao nivelado: roll > 5 graus";
  if (fabsf(pitch) > MAX_ANGLE_ARM) return "Nao nivelado: pitch > 5 graus";
  return nullptr; // ok
}

// =====================================================================
//                 LEITURA DIRETA DA IMU (com resultado EXPLICITO)
//   Por que nao usar mpu.getMotion6()? Porque a biblioteca guarda o buffer
//   como membro do objeto: quando a transacao I2C FALHA ela nao zera nada e
//   devolve os VALORES ANTIGOS como se fossem novos (o testConnection() faz
//   idem - por isso ele "da erro e depois funciona", e por isso a verificacao
//   sozinha nao e' confiavel). Aqui: falha => false e nada e' usado.
// =====================================================================
constexpr uint8_t MPU_END = 0x68;     // AD0 em GND

bool lerIMU(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(MPU_END);
  Wire.write(0x3B);                              // ACCEL_XOUT_H: bloco de 14 bytes
  if (Wire.endTransmission(false) != 0) return false;   // NACK = sensor nao respondeu
  if (Wire.requestFrom((uint8_t)MPU_END, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  *ax = (int16_t)((b[0] << 8) | b[1]);
  *ay = (int16_t)((b[2] << 8) | b[3]);
  *az = (int16_t)((b[4] << 8) | b[5]);
  *gx = (int16_t)((b[8] << 8) | b[9]);
  *gy = (int16_t)((b[10] << 8) | b[11]);
  *gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

// Prova de vida REAL: leitura respondeu E a gravidade aparece em algum eixo.
// Em qualquer posicao parada, >= ~0,58 g (9460 counts em +-2 g) fica em um eixo.
bool imuViva() {
  int16_t ax, ay, az, gx, gy, gz;
  if (!lerIMU(&ax, &ay, &az, &gx, &gy, &gz)) return false;
  return abs(ax) > 8000 || abs(ay) > 8000 || abs(az) > 8000;
}

// Conexao com repeticao: logo apos o boot a 1a leitura costuma falhar
// (sensor/barramento ainda subindo) - por isso tentamos varias vezes.
bool conectarIMU(int tentativas) {
  for (int t = 0; t < tentativas; t++) {
    if (imuViva()) return true;
    delay(50);
  }
  return false;
}

// =====================================================================
//                     CALIBRACAO COMPLETA DA IMU
//   Deixe o drone PARADO e em superficie nivelada.
//   Gyro: zera drift (x, y, z). Accel: zera bias (x, y) e ancoraz em 1 g.
// =====================================================================
void calibrarIMU() {
  const int N = 2000;
  int64_t sx = 0, sy = 0, sz = 0, sax = 0, say = 0, saz = 0;
  int ok = 0;                              // amostras que realmente chegaram

  desarmar();
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (lerIMU(&ax, &ay, &az, &gx, &gy, &gz)) {
      sx += gx; sy += gy; sz += gz;
      sax += ax; say += ay; saz += az;
      ok++;
    }
    delay(2);
  }

  if (ok == 0) {                           // IMU morta: nao calibra nada
    sensorOK = false; calibrado = false;
    contCalibErro++;
    logEvento("Calibracao FALHOU: IMU nao respondeu");
    Serial.println("CALIBRACAO FALHOU: IMU nao respondeu");
    return;
  }

  gxOffset = sx / ok;  gyOffset = sy / ok;  gzOffset = sz / ok;
  accXOffset = sax / ok;
  accYOffset = say / ok;
  accZOffset = saz / ok - 16384; // em repouso o eixo Z deve ler +1 g

  // Prova de vida da calibracao em si: em repouso ALGUM eixo carrega gravidade.
  // (substitui o antigo testConnection() daqui - era ele que derrubava o
  //  sensorOK logo apos "ser detectado")
  int64_t gX = sax > 0 ? sax : -sax;
  int64_t gY = say > 0 ? say : -say;
  int64_t gZ = saz > 0 ? saz : -saz;
  sensorOK = (ok >= (N * 9) / 10) && (gX > 8000LL * ok || gY > 8000LL * ok || gZ > 8000LL * ok);
  calibrado = sensorOK;
  initialized = false;          // refaz o angulo inicial
  roll = pitch = 0;
  gyroXf = gyroYf = gyroZf = 0;
  intRoll = intPitch = intYaw = 0;
  falhasLeitura = 0;

  Serial.printf("CALIBRADO  gyro[ld]: %d %d %d | accel[ld]: %d %d %d | amostras=%d/%d vivo=%d\n",
                (int)gxOffset, (int)gyOffset, (int)gzOffset,
                (int)accXOffset, (int)accYOffset, (int)accZOffset, ok, N, sensorOK);
  if (sensorOK) { contCalibOk++;    logEvento("IMU calibrada OK"); }
  else          { contCalibErro++;  logEvento("Calibracao REPROVADA"); }
}

// =====================================================================
//                         WEBSOCKET / HTTP
// =====================================================================
void onWebSocketEvent(AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type,
                      void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT: {
      ultimoHeartbeat = millis();
      contWsConect++;
      logEvento("Navegador conectou");
      // sincroniza a pagina com o estado real (throttle ja ajustado + teto)
      char syn[26];
      snprintf(syn, sizeof(syn), "THRATUAL,%d", (int)throttleBase);
      client->text(syn);
      snprintf(syn, sizeof(syn), "THRMAXATUAL,%d", (int)throttleMax);
      client->text(syn);
      char ver[64];                       // cracha da versao no canto da pagina
      snprintf(ver, sizeof(ver), "VER,%s", FW_VER);
      client->text(ver);
      break;
    }

    case WS_EVT_DISCONNECT:
      // Cliente caiu: deixa o failsafe do loop desarmar (logica so num lugar).
      ultimoHeartbeat = millis() - FAILSAFE_MS - 1;
      contWsDesconect++;
      logEvento("Navegador desconectou");
      break;

    case WS_EVT_DATA: {
      ultimoHeartbeat = millis();
      AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
      if (!info || !info->final || info->index != 0 || info->len != len ||
          info->opcode != WS_TEXT || len >= 80) return;

      char msg[80];
      memcpy(msg, data, len);
      msg[len] = '\0';

      if (strcmp(msg, "HB") == 0) { contHeartbeats++; return; } // heartbeat puro

      // THR,<us> - throttle base (pode ser ajustado tambem armado)
      if (strncmp(msg, "THR,", 4) == 0) {
        int v = atoi(msg + 4);
        if (v < 1000) v = 1000;
        if (v > 2000) v = 2000;
        if (v > throttleMax) v = throttleMax;   // respeita o teto do "Max. throttle"
        throttleBase = v;
        contAjusteTHR++;
        char ev[40];
        snprintf(ev, sizeof(ev), "THR = %d us", v);
        logEvento(ev);
        char reply[24];
        snprintf(reply, sizeof(reply), "THRATUAL,%d", (int)throttleBase);
        client->text(reply);
        return;
      }

      // THRMAX,<us> - teto do acelerador: o throttle nunca passa disso
      // (1000 = sem margem, 2000 = livre; ex. 1500 = varia so +500 acima do minimo)
      if (strncmp(msg, "THRMAX,", 7) == 0) {
        int v = atoi(msg + 7);
        if (v < 1000) v = 1000;
        if (v > 2000) v = 2000;
        throttleMax = v;
        if (throttleBase > throttleMax) throttleBase = throttleMax; // puxa pra baixo na hora
        char ev[44];
        snprintf(ev, sizeof(ev), "THR max = %d us (var +%d)", v, v - 1000);
        logEvento(ev);
        char reply[26];
        snprintf(reply, sizeof(reply), "THRMAXATUAL,%d", (int)throttleMax);
        client->text(reply);
        return;
      }

      float pa, pr, ir, dr, kpy, kiy;
      int n = sscanf(msg, "PID,%f,%f,%f,%f,%f,%f", &pa, &pr, &ir, &dr, &kpy, &kiy);
      if (n >= 4) {
        if (!armado) {
          PAngle = pa; PRate = pr; IRate = ir; DRate = dr;
          if (n >= 6) { KpYaw = kpy; KiYaw = kiy; }
          contAjustePID++;
          char ev[56];
          snprintf(ev, sizeof(ev), "PID: Pa%.2f Pt%.2f I%.2f D%.3f",
                   PAngle, PRate, IRate, DRate);
          logEvento(ev);
        }
        char reply[120];
        snprintf(reply, sizeof(reply), "PIDATUAL,%.3f,%.3f,%.3f,%.4f,%.3f,%.4f",
                 PAngle, PRate, IRate, DRate, KpYaw, KiYaw);
        client->text(reply);
      }
      break;
    }
    default: break;
  }
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{background:black;color:#00ffcc;font-family:Arial;text-align:center;}
h1{font-size:26px;}
canvas{background:#001010;border:2px solid #00ffcc;}
.pid-box{margin-top:15px;padding:12px;border:2px solid #00ffcc;border-radius:15px;width:320px;margin-left:auto;margin-right:auto;}
.pid-row{display:flex;justify-content:space-between;align-items:center;margin:8px 0;}
.pid-row input{width:100px;font-size:15px;text-align:center;}
button{background:#00ffcc;color:black;font-weight:bold;padding:10px;margin:5px;cursor:pointer;border:none;border-radius:6px;}
button.warn{background:#ff5555;color:white;}
.dashboard{display:flex;flex-wrap:wrap;justify-content:center;gap:12px;margin-top:15px;}
.card{background:#001010;border:2px solid #00ffcc;border-radius:15px;padding:12px;width:210px;}
.linha{display:flex;justify-content:space-between;margin:5px 0;font-size:15px;}
#msg{min-height:22px;margin-top:10px;color:#ffcc00;font-weight:bold;}
.dica{font-size:12px;color:#7fbb9f;margin:6px 0;text-align:left;}
.pid-row label{font-size:14px;}
h3{margin:4px 0;}
</style>
</head>
<body>
<div id="fwver" style="position:fixed;top:6px;right:10px;font-size:12px;color:#7fbb9f;background:#001010;border:1px solid #00ffcc;border-radius:8px;padding:3px 8px;z-index:10;">FW ...</div>
<h1>DRONE TELEMETRIA</h1>
<canvas id="grafico" width="350" height="180"></canvas>
<div>
  <button onclick="arm()">ARM</button>
  <button class="warn" onclick="disarm()">DISARM</button>
  <button onclick="calibrar()">CALIBRAR IMU</button>
  <button onclick="resetStats()">RESET STATS</button>
  <button onclick="baixarDiag()">BAIXAR DIAGNÓSTICO</button>
</div>
<div id="msg">Desarmado</div>

<div class="pid-box">
  <h2>PID CASCATA</h2>
  <div class="dica">Ângulo → taxa (°/s) → µs. Sem limite no PID — o ESC segura 1100–2000 µs.</div>
  <div class="pid-row"><label>P ângulo (°/s por °)</label><input type="number" id="pa" step="0.1" value="2.5"></div>
  <div class="pid-row"><label>P taxa (µs por °/s)</label><input type="number" id="pr" step="0.05" value="1.5"></div>
  <div class="pid-row"><label>I taxa</label><input type="number" id="ir" step="0.05" value="0.5"></div>
  <div class="pid-row"><label>D taxa</label><input type="number" id="dr" step="0.001" value="0.01"></div>
  <div class="pid-row"><label>Kp yaw</label><input type="number" id="kpy" step="0.1" value="1.5"></div>
  <div class="pid-row"><label>Ki yaw</label><input type="number" id="kiy" step="0.001" value="0"></div>
  <button onclick="enviarPID()">APLICAR PID</button>
</div>

<div class="pid-box">
  <h2>THROTTLE BASE</h2>
  <div class="pid-row"><label>Acelerador</label><b><span id="thrval">1000</span> µs</b></div>
  <input type="range" id="thr" min="1000" max="2000" step="10" value="1000"
         style="width:100%" oninput="thrInput(this.value)" onchange="thrSend(this.value)">
  <div class="dica">AO LIGAR comeca em 1000 (parado) - suba antes de armar. F450 + 1045: hover 1350–1450 µs.</div>
  <div class="pid-row"><label>Máx. throttle</label><b><span id="thrmaxval">1500</span> µs <span id="thrmaxspan" style="opacity:.7">(variação +500)</span></b></div>
  <input type="range" id="thrmax" min="1000" max="2000" step="10" value="1500"
         style="width:100%" oninput="thrMaxInput(this.value)" onchange="thrMaxSend(this.value)">
  <div class="dica">Teto do Acelerador: ao ligar = 1500 (seguranca). Ex.: 1400 = acelerador chega em 1400. Vale na hora e o slider de cima para no teto.</div>
</div>

<div class="dashboard">
  <div class="card">
    <h3>ATITUDE</h3>
    <div class="linha"><span>ROLL</span><span id="roll">0</span></div>
    <div class="linha"><span>PITCH</span><span id="pitch">0</span></div>
    <div class="linha"><span>YAW rate</span><span id="yawr">0</span></div>
  </div>
  <div class="card">
    <h3>PID OUT</h3>
    <div class="linha"><span>ROLL</span><span id="prr">0</span></div>
    <div class="linha"><span>PITCH</span><span id="ppr">0</span></div>
    <div class="linha"><span>YAW</span><span id="pyr">0</span></div>
  </div>
  <div class="card">
    <h3>MOTORES</h3>
    <div class="linha"><span>A</span><span id="ma">1000</span></div>
    <div class="linha"><span>B</span><span id="mb">1000</span></div>
    <div class="linha"><span>C</span><span id="mc">1000</span></div>
    <div class="linha"><span>D</span><span id="md">1000</span></div>
    <div class="linha"><span>BASE</span><span id="thrbase">1000</span></div>
  </div>
  <div class="card">
    <h3>DIAGNOSTICO</h3>
    <div class="linha"><span>CICLO</span><span id="ciclo">0 us</span></div>
    <div class="linha"><span>CICLO MAX</span><span id="cmax">0 us</span></div>
    <div class="linha"><span>JITTER MAX</span><span id="jit">0 us</span></div>
    <div class="linha"><span>ESTOUROS</span><span id="est">0</span></div>
    <div class="linha"><span>VIB RMS</span><span id="vibrms">0.000</span></div>
    <div class="linha"><span>VIB PICO</span><span id="vibpico">0.000</span></div>
    <div class="linha"><span>LIMITE</span><span>4000 us</span></div>
  </div>
</div>

<script>
let ws = new WebSocket(`ws://${location.host}/ws`);
let canvas = document.getElementById("grafico");
let ctx = canvas.getContext("2d");
let rollData = [], pitchData = [];
let armClick = 0;

function msg(t){ document.getElementById("msg").innerText = t; }

ws.onopen  = function(){ msg("Conectado - desarmado"); };
ws.onclose = function(){ msg("WebSocket caiu (failsafe deve desarmar)"); };

// Heartbeat: mantem o drone armado so enquanto esta pagina estiver viva
setInterval(function(){
  if (ws.readyState === WebSocket.OPEN) ws.send("HB");
}, 200);

ws.onmessage = function(event){
  // Responde o heartbeat a cada telemetria recebida (20 Hz). Isso mantem o
  // failsafe vivo mesmo se o navegador adiar os timers em aba em segundo plano.
  if (ws.readyState === WebSocket.OPEN) ws.send("HB");

  let data = event.data.split(",");
  if (data[0] == "VER") {
    document.getElementById("fwver").innerText = "FW " + event.data.substring(4);
    return;
  }
  if (data[0] == "PIDATUAL") {
    document.getElementById("pa").value = data[1];
    document.getElementById("pr").value = data[2];
    document.getElementById("ir").value = data[3];
    document.getElementById("dr").value = data[4];
    if (data.length >= 7) {
      document.getElementById("kpy").value = data[5];
      document.getElementById("kiy").value = data[6];
    }
    return;
  }
  if (data[0] == "THRATUAL") {
    let tv = document.getElementById("thr");
    tv.value = data[1];
    document.getElementById("thrval").innerText = data[1];
    msg("Throttle = " + data[1] + " us"); return;
  }
  if (data[0] == "THRMAXATUAL") {
    let tm = document.getElementById("thrmax");
    tm.value = data[1];
    thrMaxInput(data[1]);
    return;
  }
  if (data[0] !== "T" || data.length < 18) return;

  let r = parseFloat(data[1]), p = parseFloat(data[2]);
  document.getElementById("roll").innerHTML  = r.toFixed(2);
  document.getElementById("pitch").innerHTML = p.toFixed(2);
  document.getElementById("yawr").innerHTML  = parseFloat(data[16]).toFixed(2);
  document.getElementById("prr").innerHTML   = parseFloat(data[5]).toFixed(1);
  document.getElementById("ppr").innerHTML   = parseFloat(data[6]).toFixed(1);
  document.getElementById("pyr").innerHTML   = parseFloat(data[17]).toFixed(1);
  document.getElementById("ma").innerHTML = data[7];
  document.getElementById("mb").innerHTML = data[8];
  document.getElementById("mc").innerHTML = data[9];
  document.getElementById("md").innerHTML = data[10];
  document.getElementById("ciclo").innerHTML = data[12] + " us";
  document.getElementById("cmax").innerHTML  = data[13] + " us";
  document.getElementById("jit").innerHTML   = data[14] + " us";
  document.getElementById("est").innerHTML   = data[15];
  if (data.length >= 19) document.getElementById("thrbase").innerHTML = data[18];
  if (data.length >= 21) {
    document.getElementById("vibrms").innerHTML  = parseFloat(data[19]).toFixed(3);
    document.getElementById("vibpico").innerHTML = parseFloat(data[20]).toFixed(3);
  }

  rollData.push(r); pitchData.push(p);
  if (rollData.length > 50) { rollData.shift(); pitchData.shift(); }
  drawGraph();
};

function enviarPID(){
  let pa = document.getElementById("pa").value;
  let pr = document.getElementById("pr").value;
  let ir = document.getElementById("ir").value;
  let dr = document.getElementById("dr").value;
  let kpy = document.getElementById("kpy").value;
  let kiy = document.getElementById("kiy").value;
  if (ws.readyState === WebSocket.OPEN) ws.send(`PID,${pa},${pr},${ir},${dr},${kpy},${kiy}`);
}

function thrInput(v){ document.getElementById("thrval").innerText = v; }
function thrSend(v){
  if (ws.readyState === WebSocket.OPEN) ws.send("THR," + v);
}
function thrMaxInput(v){
  v = parseInt(v, 10);
  document.getElementById("thrmaxval").innerText = v;
  document.getElementById("thrmaxspan").innerText = "(variação +" + (v - 1000) + ")";
  // slider do Acelerador passa a ir ate o teto (e nunca alem)
  let t = document.getElementById("thr");
  t.max = v;
  if (parseInt(t.value, 10) > v) {
    t.value = v;
    document.getElementById("thrval").innerText = v;
  }
}
function thrMaxSend(v){
  if (ws.readyState === WebSocket.OPEN) ws.send("THRMAX," + v);
}

// ARM em dois cliques (evita clique acidental)
async function arm(){
  let t = Date.now();
  if (t - armClick > 3000) { armClick = t; msg("Confirmar: clique ARM novamente"); return; }
  armClick = 0;
  let r = await fetch("/arm");
  msg(await r.text());
}
async function disarm(){ let r = await fetch("/disarm"); msg(await r.text()); }
async function calibrar(){ let r = await fetch("/calibrar"); msg(await r.text()); }
async function resetStats(){ let r = await fetch("/resetstats"); msg(await r.text()); }
function baixarDiag(){ window.open("/diag.txt", "_blank"); }

function drawGraph(){
  ctx.clearRect(0,0,350,180);
  ctx.beginPath(); ctx.moveTo(0,90); ctx.lineTo(350,90); ctx.strokeStyle="white"; ctx.stroke();
  ctx.beginPath();
  for (let i=0; i<rollData.length; i++){
    let x = i * 7, y = 90 - (rollData[i] * 2);
    if (i==0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.strokeStyle="red"; ctx.lineWidth=2; ctx.stroke();
  ctx.beginPath();
  for (let i=0; i<pitchData.length; i++){
    let x = i * 7, y = 90 - (pitchData[i] * 2);
    if (i==0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.strokeStyle="cyan"; ctx.lineWidth=2; ctx.stroke();
}
</script>
</body>
</html>
)rawliteral";

// =====================================================================
//                  DIAGNOSTICO EM TEXTO  (/diag.txt)
//   Botao "BAIXAR DIAGNOSTICO" na pagina: gera um .txt com config,
//   IMU, loop, contadores, eventos e a gravacao da ultima subida.
// =====================================================================
const char* motivoReset() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "energia ligada";
    case ESP_RST_EXT:       return "externo (pino EN)";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "PANIC (crash)";
    case ESP_RST_INT_WDT:   return "watchdog de interrupcao";
    case ESP_RST_TASK_WDT:  return "watchdog de tarefa";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (tensao baixa! confira a bateria)";
    default:                return "desconhecido";
  }
}

int formatarLinhaVoo(int idx, char* l, size_t tam) {
  const Amostra& a = vooLog[idx];   // evita struct na assinatura (prototipos do Arduino)
  return snprintf(l, tam, "%u,%.2f,%.2f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d\n",
                  a.tms, a.roll / 100.0f, a.pitch / 100.0f,
                  a.gx / 10.0f, a.gy / 10.0f, a.gz / 10.0f,
                  a.pidR, a.pidP, a.pidY, a.mA, a.mB, a.mC, a.mD, a.vib);
}

String montarDiagTopo() {
  char b[256];
  String s;
  s.reserve(5200);
  s += "=== DIAGNOSTICO ESP32 DRONE ===\n";
  snprintf(b, sizeof(b), "gerado: %lu ms apos boot | tempo ligado: %.1f min\n",
           (unsigned long)millis(), millis() / 60000.0f);
  s += b;
  s += "firmware: ESP32_Drone_Diagnostico - PID em cascata, SEM clamp do PID | ";
  s += FW_VER;
  s += "\n";

  snprintf(b, sizeof(b), "chip: %s | %u MHz | flash: %.1f MB | heap livre: %u B\n",
           ESP.getChipModel(), ESP.getCpuFreqMHz(),
           ESP.getFlashChipSize() / 1048576.0f, (unsigned)ESP.getFreeHeap());
  s += b;
  snprintf(b, sizeof(b), "reset: %s | AP WiFi: %s | IP: %s | clientes WS: %u | pontos AP: %u\n",
           motivoReset(), ssid, WiFi.softAPIP().toString().c_str(),
           (unsigned)ws.count(), (unsigned)WiFi.softAPgetStationNum());
  s += b;

  snprintf(b, sizeof(b), "\n--- ESTADO ---\narmado=%d sensorOK=%d calibrado=%d inicializado=%d\n",
           armado ? 1 : 0, sensorOK ? 1 : 0, calibrado ? 1 : 0, initialized ? 1 : 0);
  s += b;
  snprintf(b, sizeof(b), "roll=%.2f pitch=%.2f | gyro f: x=%.1f y=%.1f z=%.1f dps\n",
           roll, pitch, gyroXf, gyroYf, gyroZf);
  s += b;
  snprintf(b, sizeof(b), "vibracao (|a|-1g): rms=%.3f g | pico=%.3f g\n",
           vibRms, vibPeak);
  s += b;
  snprintf(b, sizeof(b), "pid: roll=%.1f pitch=%.1f yaw=%.1f us | erro roll=%.2f pitch=%.2f\n",
           pidRoll, pidPitch, pidYaw, erroRoll, erroPitch);
  s += b;
  snprintf(b, sizeof(b), "motores: A=%d B=%d C=%d D=%d | throttleBase=%d\n",
           motorSpeedA, motorSpeedB, motorSpeedC, motorSpeedD, (int)throttleBase);
  s += b;
  snprintf(b, sizeof(b), "heartbeat: ultimo ha %lu ms (failsafe em %lu ms)\n",
           (unsigned long)(millis() - ultimoHeartbeat), (unsigned long)FAILSAFE_MS);
  s += b;

  s += "\n--- CONFIG (ganhos) ---\n";
  snprintf(b, sizeof(b),
           "PAngle=%.2f PRate=%.2f IRate=%.2f DRate=%.4f | KpYaw=%.2f KiYaw=%.4f\n"
           "ITERM_LIM=%.0f RATE_LIM=%.0f | clamp de saida do PID: NENHUM\n",
           PAngle, PRate, IRate, DRate, KpYaw, KiYaw, ITERM_LIM, RATE_LIM);
  s += b;
  snprintf(b, sizeof(b), "motores: MIN=%d IDLE=%d MAX=%d (ESC %d Hz, faixa real 1000..2000 us)\n",
           MOTOR_MIN, MOTOR_IDLE, MOTOR_MAX, ESC_FREQ);
  s += b;
  snprintf(b, sizeof(b), "pinos: ESC A=%d B=%d C=%d D=%d | I2C SDA=21 SCL=22 (MPU 0x68)\n",
           ESC_A, ESC_B, ESC_C, ESC_D);
  s += b;
  snprintf(b, sizeof(b),
           "timing: loop %lu Hz (%lu us) | telemetria 20 Hz | HB %lu ms | failsafe %lu ms\n"
           "YAW_SIGN=%.1f | MAX_ANGLE_ARM=%.1f graus | THROTTLE_BASE=%d (padrao ao LIGAR; web 1000..2000)\n"
           "THR_MAX=%d (teto do acelerador, var +%d)\n",
           (unsigned long)(1000000UL / PERIOD_US), (unsigned long)PERIOD_US,
           (unsigned long)HEARTBEAT_JS, (unsigned long)FAILSAFE_MS,
           YAW_SIGN, MAX_ANGLE_ARM, THROTTLE_BASE,
           (int)throttleMax, (int)throttleMax - 1000);
  s += b;

  s += "\n--- IMU (MPU6050) ---\n";
  snprintf(b, sizeof(b), "sensorOK=%d calibrado=%d | falhas de leitura seguidas=%d\n",
           sensorOK ? 1 : 0, calibrado ? 1 : 0, falhasLeitura);
  s += b;
  snprintf(b, sizeof(b), "offsets gyro: gx=%ld gy=%ld gz=%ld | accel: ax=%ld ay=%ld az=%ld\n",
           (long)gxOffset, (long)gyOffset, (long)gzOffset,
           (long)accXOffset, (long)accYOffset, (long)accZOffset);
  s += b;

  s += "\n--- LOOP (esperado < 4000 us) ---\n";
  snprintf(b, sizeof(b), "ciclo atual=%lu us | max=%lu us | jitter=%lu us | estouros=%lu | ciclos totais=%lu\n",
           (unsigned long)cicloUs, (unsigned long)cicloMaxUs, (unsigned long)jitterUs,
           (unsigned long)estouros, (unsigned long)contCiclos);
  s += b;

  s += "\n--- PERFIL DO CICLO (us; medias desde a ultima calibracao/zerar) ---\n";
  unsigned long nPerf = tCicloN ? tCicloN : 1;
  snprintf(b, sizeof(b), "ciclo: media=%lu max=%lu | IMU (I2C): media=%lu max=%lu\n",
           (unsigned long)(tCicloSoma / nPerf), (unsigned long)cicloMaxUs,
           (unsigned long)(tIMUsum / nPerf), (unsigned long)tIMUmax);
  s += b;
  snprintf(b, sizeof(b), "controle+ESC: media=%lu max=%lu | telemetria: media=%lu max=%lu\n",
           (unsigned long)(tCtrlsum / nPerf), (unsigned long)tCtrlmax,
           (unsigned long)(tTelsum / nPerf), (unsigned long)tTelmax);
  s += b;
  snprintf(b, sizeof(b), "escrita ESC (4x): media=%lu max=%lu | bench setup: %lu us/escrita\n",
           (unsigned long)(tEscsum / nPerf), (unsigned long)tEscmax,
           (unsigned long)benchEscUs);
  s += b;
  snprintf(b, sizeof(b), "duty_rd (escrita no HW): %s\n", escVerif);
  s += b;
  long imprev = (long)(tCicloSoma / nPerf) - (long)(tIMUsum / nPerf)
              - (long)(tCtrlsum / nPerf) - (long)(tTelsum / nPerf);
  if (imprev < 0) imprev = 0;
  snprintf(b, sizeof(b), "imprevisto (Wi-Fi/preempcao): media=%lu | amostras n=%lu\n",
           (unsigned long)imprev, (unsigned long)tCicloN);
  s += b;

  s += "\n--- CONTADORES ---\n";
  snprintf(b, sizeof(b), "armamentos=%lu | failsafes=%lu | calibracoes ok=%lu falha=%lu | reconexoes IMU=%lu\n",
           (unsigned long)contArmamentos, (unsigned long)contFailsafes,
           (unsigned long)contCalibOk, (unsigned long)contCalibErro,
           (unsigned long)contReconexoesIMU);
  s += b;
  snprintf(b, sizeof(b), "WS conectou=%lu desconectou=%lu | HB recebidos=%lu | ajustes PID=%lu throttle=%lu\n",
           (unsigned long)contWsConect, (unsigned long)contWsDesconect,
           (unsigned long)contHeartbeats, (unsigned long)contAjustePID,
           (unsigned long)contAjusteTHR);
  s += b;

  s += "\n--- EVENTOS (ms apos boot) ---\n";
  if (nEventos == 0) {
    s += "(nenhum evento registrado)\n";
  } else {
    int total = nEventos < MAX_EVENTOS ? (int)nEventos : MAX_EVENTOS;
    int ini = (int)((nEventos - total) % MAX_EVENTOS);  // primeiro valido do anel
    for (int i = 0; i < total; i++) {
      const Evento& e = eventosLog[(ini + i) % MAX_EVENTOS];
      snprintf(b, sizeof(b), "[%8lu] %s\n", (unsigned long)e.ms, e.txt);
      s += b;
    }
  }

  s += "\n--- DADOS DE VOO (ultima subida; 25 Hz) ---\n";
  if (vooN == 0) {
    s += "(nenhuma gravacao ainda - arme o drone e faca um teste)\n";
  } else {
    snprintf(b, sizeof(b), "amostras=%u de %d | duracao=%.1f s | %s\n",
             vooN, MAX_AMOSTRAS, vooLog[vooN - 1].tms / 1000.0f,
             vooTruncado ? "TRUNCADO (encheu o buffer)" : "completo");
    s += b;
    s += "t_ms,roll_grau,pitch_grau,gx_dps,gy_dps,gz_dps,pidRoll_us,pidPitch_us,pidYaw_us,"
         "mA_us,mB_us,mC_us,mD_us,vib_mg\n";
  }
  return s;
}

// Estado do download do /diag.txt (funciona 1 download por vez)
String diagTopo;
size_t diagPos = 0;
uint16_t diagVooI = 0;
uint8_t diagFase = 0;   // 0=topo 1=dados 2=rodape 3=fim

// =====================================================================
//                                 SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // ESCs primeiro: ja comecam recebendo 1000 us (init/arming do ESC)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motorA.setPeriodHertz(ESC_FREQ); motorA.attach(ESC_A, 1000, 2000);
  motorB.setPeriodHertz(ESC_FREQ); motorB.attach(ESC_B, 1000, 2000);
  motorC.setPeriodHertz(ESC_FREQ); motorC.attach(ESC_C, 1000, 2000);
  motorD.setPeriodHertz(ESC_FREQ); motorD.attach(ESC_D, 1000, 2000);

  // ---- BENCH ESC: custo real de UMA escrita de PWM (antes do Wi-Fi ligar) ----
  // 200 voltas x 4 canais = 800 escritas alternando 1000/1002 us (faixa minima do ESC,
  // o motor nao gira) — mede o caminho de escrita puro, sem preempcao de tarefas.
  uint32_t tb0 = micros();
  for (int i = 0; i < 200; i++) {
    int f = i & 1;
    motorA.writeMicroseconds(1000 + 2 * f);
    motorB.writeMicroseconds(1002 - 2 * f);
    motorC.writeMicroseconds(1000 + 2 * f);
    motorD.writeMicroseconds(1002 - 2 * f);
  }
  benchEscUs = (micros() - tb0) / 800;
  setMotors(MOTOR_MIN);
  delay(30);                          // deixa o duty chegar ao hardware (borda do PWM)

  // ---- Verificacao duty_rd: le o duty EFETIVO do registrador do LEDC ----
  // ticks de 1000 us em 10 bits / 400 Hz: 1000 * 1024 / 2500 = 409,6 -> 409
  uint32_t esp = (1000UL * 1024UL) / 2500UL;
  uint32_t lA = ledcRead(ESC_A), lB = ledcRead(ESC_B), lC = ledcRead(ESC_C), lD = ledcRead(ESC_D);
  long dA = (long)lA - (long)esp, dB = (long)lB - (long)esp;
  long dC = (long)lC - (long)esp, dD = (long)lD - (long)esp;
  bool okHw = (dA >= -2 && dA <= 2 && dB >= -2 && dB <= 2 &&
               dC >= -2 && dC <= 2 && dD >= -2 && dD <= 2);
  snprintf(escVerif, sizeof(escVerif), "%s esp=%lu A=%lu B=%lu C=%lu D=%lu",
           okHw ? "OK" : "FALHOU", (unsigned long)esp,
           (unsigned long)lA, (unsigned long)lB, (unsigned long)lC, (unsigned long)lD);
  Serial.printf("BENCH ESC: %lu us/escrita | duty_rd %s (esp=%lu A=%lu B=%lu C=%lu D=%lu)\n",
                (unsigned long)benchEscUs, okHw ? "OK" : "FALHOU", (unsigned long)esp,
                (unsigned long)lA, (unsigned long)lB, (unsigned long)lC, (unsigned long)lD);

  desarmar();

  // IMU
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpu.initialize();                       // acorda e configura (+-250 dps / +-2 g)
  Serial.printf("MPU6050 WHO_AM_I=%d (34=MPU6050) test=%d\n",
                mpu.getDeviceID(), mpu.testConnection());

  // A 1a leitura logo apos ligar costuma falhar (sensor/barramento subindo):
  // por isso repetimos com prova de vida real em vez de confiar numa leitura so.
  sensorOK = conectarIMU(20);             // ~1 s de tentativas
  Serial.printf("MPU6050 detectada: %d\n", sensorOK);

  // Calibracao obrigatoria (drone parado e nivelado) - leva ~4 s
  if (sensorOK) calibrarIMU();
  else          Serial.println("MPU6050 NAO DETECTADA - confira SDA=21, SCL=22, AD0=GND");

  // Rede + servidor
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ r->send_P(200, "text/html; charset=utf-8", index_html); });

  server.on("/arm", HTTP_GET, [](AsyncWebServerRequest* r){
    const char* motivo = podeArmar();
    if (motivo) {
      char ev[70];
      snprintf(ev, sizeof(ev), "ARM recusado: %s", motivo);
      logEvento(ev);
      r->send(409, "text/plain", motivo); return;
    }
    intRoll = intPitch = intYaw = 0;
    ultimoHeartbeat = millis();
    armado = true;
    contArmamentos++;
    char evA[40];
    snprintf(evA, sizeof(evA), "ARM (thr=%d teto=%d)", (int)throttleBase, (int)throttleMax);
    logEvento(evA);
    // (re)inicia a gravacao da subida: guarda a ULTIMA ate o proximo arm
    vooN = 0; vooTruncado = false; vooUltima = micros() - VOO_AMOSTRA_US;
    vooT0 = millis(); vooAtivo = true;
    char okMsg[40];
    snprintf(okMsg, sizeof(okMsg), "ARMADO - throttle %d", (int)throttleBase);
    r->send(200, "text/plain", okMsg);
  });

  server.on("/disarm", HTTP_GET, [](AsyncWebServerRequest* r){
    desarmar("botao web");
    r->send(200, "text/plain", "DESARMADO");
  });

  server.on("/calibrar", HTTP_GET, [](AsyncWebServerRequest* r){
    if (armado) { r->send(409, "text/plain", "Desarme antes de calibrar"); return; }
    pedirCalibracao = true; // executado no loop (nao bloqueia o servidor)
    r->send(200, "text/plain", "Calibracao iniciada - mantenha o drone parado");
  });

  server.on("/resetstats", HTTP_GET, [](AsyncWebServerRequest* r){
    cicloMaxUs = 0; jitterUs = 0; estouros = 0; contCiclos = 0;
    tCicloSoma = tCicloN = 0;
    tIMUmax = tIMUsum = tCtrlmax = tCtrlsum = tEscmax = tEscsum = tTelmax = tTelsum = 0;
    vibRms = 0; vibPeak = 0;
    r->send(200, "text/plain", "Estatisticas zeradas");
  });

  // Diagnostico para baixar como .txt (enviado em blocos p/ nao estourar a RAM)
  server.on("/diag.txt", HTTP_GET, [](AsyncWebServerRequest* r){
    diagTopo = montarDiagTopo();
    diagPos = 0; diagVooI = 0; diagFase = 0;
    AsyncWebServerResponse* resp = r->beginChunkedResponse(
      "text/plain; charset=utf-8",
      [](uint8_t* out, size_t maxLen, size_t index) -> size_t {
        (void)index;
        size_t n = 0;
        if (diagFase == 0) {                        // 1) topo (status/eventos)
          size_t tam = diagTopo.length();
          if (diagPos < tam) {
            n = tam - diagPos;
            if (n > maxLen) n = maxLen;
            memcpy(out, (const uint8_t*)diagTopo.c_str() + diagPos, n);
            diagPos += n;
            return n;
          }
          diagFase = 1;
        }
        if (diagFase == 1) {                        // 2) linhas de voo
          while (diagVooI < vooN) {
            char lin[128];
            int len = formatarLinhaVoo(diagVooI, lin, sizeof(lin));
            if (len <= 0) { diagVooI++; continue; }
            if (n + (size_t)len > maxLen) break;    // nao coube: proxima chamada
            memcpy(out + n, lin, len);
            n += len;
            diagVooI++;
          }
          if (diagVooI >= vooN) diagFase = 2;
          else return n;
        }
        if (diagFase == 2) {                        // 3) rodape
          const char* fim = "\n--- FIM ---\n";
          size_t len = strlen(fim);
          if (n + len <= maxLen) {
            memcpy(out + n, fim, len);
            n += len;
            diagFase = 3;
            diagTopo = String();                    // libera a memoria
          }
          return n;
        }
        return n;                                   // fase 3: 0 bytes = fim
      });
    resp->addHeader("Content-Disposition", "attachment; filename=diagnostico.txt");
    r->send(resp);
  });

  server.begin();

  nextTick = micros();
  previousSample = nextTick;
  ultimoHeartbeat = millis();
}

// =====================================================================
//                            TELEMETRIA (20 Hz)
// =====================================================================
void enviarTelemetria(uint32_t now) {
  if ((uint32_t)(now - lastTelemetry) < TELEMETRY_US) return;
  lastTelemetry = now;

  uint32_t tTel0 = micros();
  char buf[240];
  snprintf(buf, sizeof(buf),
    "T,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%.2f,%.2f,%d,%.3f,%.3f",
    roll, pitch, erroRoll, erroPitch, pidRoll, pidPitch,
    motorSpeedA, motorSpeedB, motorSpeedC, motorSpeedD, armado ? 1 : 0,
    (unsigned long)cicloUs, (unsigned long)cicloMaxUs,
    (unsigned long)jitterUs, (unsigned long)estouros,
    gyroZf, pidYaw, (int)throttleBase, vibRms, vibPeak);
  ws.textAll(buf);
  ws.cleanupClients();
  uint32_t tTelus = micros() - tTel0;
  if (tTelus > tTelmax) tTelmax = tTelus;
  tTelsum += tTelus;
}

// =====================================================================
//                                  LOOP (250 Hz)
// =====================================================================
void loop() {
  uint32_t now = micros();

  // Agendamento estrito de ciclo
  if ((int32_t)(now - nextTick) < 0) return;

  int32_t atraso = (int32_t)(now - nextTick);
  nextTick += PERIOD_US;
  if ((uint32_t)atraso > jitterUs) jitterUs = (uint32_t)atraso;
  if (atraso > (int32_t)(2 * PERIOD_US)) nextTick = now + PERIOD_US; // estourou: descarta ticks perdidos (evita rajada)

  if (throttleBase > throttleMax) throttleBase = throttleMax; // teto do slider "Max."
  uint32_t tInicio = micros();
  contCiclos++;

  float dt = (now - previousSample) * 1.0e-6f;
  previousSample = now;
  if (dt <= 0 || dt > 0.02f) dt = 0.004f;

  // Failsafe: sem heartbeat recente => desarma (logica so aqui embaixo)
  if (armado && (millis() - ultimoHeartbeat) > FAILSAFE_MS) {
    contFailsafes++;
    Serial.println("FAILSAFE: heartbeat perdido -> DISARM");
    desarmar("FAILSAFE: heartbeat perdido");
  }

  // Calibracao pedida pela web (so quando desarmado)
  if (pedirCalibracao && !armado) {
    pedirCalibracao = false;
    calibrarIMU();                    // leva ~4 s dentro do loop: isolamos o ciclo
    cicloCalibrando = true;           // para nao contaminar max/jitter/estouros
    nextTick = micros();              // reengata a agenda (evita jitter de 4 s)
    cicloMaxUs = 0; estouros = 0; contCiclos = 0;
    tCicloSoma = tCicloN = 0;
    tIMUmax = tIMUsum = tCtrlmax = tCtrlsum = tEscmax = tEscsum = tTelmax = tTelsum = 0;
    vibRms = 0; vibPeak = 0;
  }

  if (!sensorOK) {
    if (armado) desarmar("IMU falhou em voo");  // falha da IMU em voo => corte imediato

    // AUTO-RECUPERACAO: tenta reconectar 1x por segundo e recalibra sozinho
    // (nao precisa mais reiniciar nem apontar verificacao de cabeca pra baixo)
    if ((int32_t)(millis() - ultimaTentativaIMU) > 1000) {
      ultimaTentativaIMU = millis();
      Serial.println("IMU: tentando reconectar...");
      if (conectarIMU(3)) {
        calibrarIMU();                // reconectou => recalibra (drone parado)
        nextTick = micros();          // calibracao levou ~4 s: reengata a agenda
        Serial.println(sensorOK ? "IMU: RECUPERADA + recalibrada"
                                : "IMU: conectou mas calibracao reprovada");
        if (sensorOK) { contReconexoesIMU++; logEvento("IMU RECUPERADA + recalibrada"); }
      }
    }

    cicloUs = micros() - tInicio;
    enviarTelemetria(now);           // mantem a tela viva mostrando a falha
    return;
  }

  // ------------------------- LEITURA DA IMU -------------------------
  int16_t ax, ay, az, gx, gy, gz;
  uint32_t tIMU0 = micros();
  if (!lerIMU(&ax, &ay, &az, &gx, &gy, &gz)) {
    // falha EXPLICITA do barramento (antes a biblioteca devolvia dados velhos
    // e a falha passava despercebida)
    if (++falhasLeitura > 10) {       // ~40 ms seguidos sem resposta
      sensorOK = false;
      Serial.println("IMU: leitura falhou (barramento)");
    }
    cicloUs = micros() - tInicio;
    enviarTelemetria(now);
    return;                           // ciclo: nao integra com dado ruim
  }
  falhasLeitura = 0;
  uint32_t tIMUus = micros() - tIMU0;
  if (tIMUus > tIMUmax) tIMUmax = tIMUus;
  tIMUsum += tIMUus;

  float gyroX = ((int32_t)gx - gxOffset) / LSB_GYRO;
  float gyroY = ((int32_t)gy - gyOffset) / LSB_GYRO;
  float gyroZ = ((int32_t)gz - gzOffset) / LSB_GYRO;

  // Low-pass leve no gyro (suaviza o termo D)
  gyroXf = 0.8f * gyroXf + 0.2f * gyroX;
  gyroYf = 0.8f * gyroYf + 0.2f * gyroY;
  gyroZf = 0.8f * gyroZf + 0.2f * gyroZ;

  float axg = ((int32_t)ax - accXOffset) / LSB_ACCEL;
  float ayg = ((int32_t)ay - accYOffset) / LSB_ACCEL;
  float azg = ((int32_t)az - accZOffset) / LSB_ACCEL;

  // Vibracao: |a| - 1 g a cada ciclo (offsets ja subtraidos acima)
  float vibDev = fabsf(sqrtf(axg*axg + ayg*ayg + azg*azg) - 1.0f);
  vibRms += (vibDev - vibRms) * 0.01f;
  if (vibDev > vibPeak) vibPeak = vibDev;

  float accRoll  = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 57.2957795f;
  float accPitch = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 57.2957795f;

  if (!initialized) {
    roll = accRoll; pitch = accPitch;
    initialized = true;
  }

  // Filtro complementar (98% gyro / 2% accel)
  roll  = 0.98f * (roll  + gyroXf * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gyroYf * dt) + 0.02f * accPitch;

  // ------------------------- CONTROLE (PID em CASCATA) -------------------------
  uint32_t tCtrl0 = micros();
  if (!armado) {
    intRoll = intPitch = intYaw = 0;
    prevErrRateRoll = prevErrRatePitch = 0;
    dFiltRoll = dFiltPitch = 0;
    pidRoll = pidPitch = pidYaw = 0;
    motorSpeedA = motorSpeedB = motorSpeedC = motorSpeedD = MOTOR_MIN;
    uint32_t tEsc0 = micros();
    setMotors(MOTOR_MIN);
    uint32_t tEscus = micros() - tEsc0;
    if (tEscus > tEscmax) tEscmax = tEscus;
    tEscsum += tEscus;
  } else {
    // ---- LAÇO EXTERNO: erro de ANGULO -> taxa desejada (graus/s) ----
    // setpoint 0 = manter nivelado
    erroRoll  = -roll;
    erroPitch = -pitch;
    float taxaRoll  = constrain(PAngle * erroRoll,  -RATE_LIM, RATE_LIM);
    float taxaPitch = constrain(PAngle * erroPitch, -RATE_LIM, RATE_LIM);

    // ---- LAÇO INTERNO: erro de TAXA -> deslocamento de motor (µs) ----
    float errTaxaRoll  = taxaRoll  - gyroXf;
    float errTaxaPitch = taxaPitch - gyroYf;

    if (IRate > 0) {                                  // integral + anti-windup
      // Volta ao ZERO e parou (demo de nivelamento terminada): esvazia o acumulo
      // com calma (meia-vida ~1,4 s a 250 Hz). Sem isso, o I carregado enquanto voce
      // segura o drone inclinado fica alto depois de soltar e os motores continuam
      // empurrando ("descontrole" apos retornar ao nivel). Em voo/hover normal a
      // condicao raramente fica 1 s satisfeita, entao o trim de equilibrio nao perde.
      if (fabsf(roll) < 0.5f && fabsf(pitch) < 0.5f &&
          fabsf(gyroXf) < 2.0f && fabsf(gyroYf) < 2.0f) {
        intRoll  *= 0.998f;
        intPitch *= 0.998f;
      }
      intRoll  = constrain(intRoll  + errTaxaRoll  * dt, -ITERM_LIM, ITERM_LIM);
      intPitch = constrain(intPitch + errTaxaPitch * dt, -ITERM_LIM, ITERM_LIM);
    } else {
      intRoll = intPitch = 0;                         // I desligado: zera o acumulo
    }

    // Passa-baixa do D (alfa 0.25 a 250 Hz ~ corte 11 Hz). Sem ela o D amplifica o
    // ruido de gyro da VIBRACAO: no voo de 60 s do diagnostico (4) ele cuspia picos
    // de +700 us a cada salto de gyro, os motores estouravam nos rails 1100/2000 e a
    // vibracao realimentava o laco (cresce ate saturar = "indo de um lado pro outro").
    float dRawRoll  = DRate * (errTaxaRoll  - prevErrRateRoll)  / dt;
    float dRawPitch = DRate * (errTaxaPitch - prevErrRatePitch) / dt;
    dFiltRoll  = 0.75f * dFiltRoll  + 0.25f * dRawRoll;
    dFiltPitch = 0.75f * dFiltPitch + 0.25f * dRawPitch;
    float derRoll  = dFiltRoll;
    float derPitch = dFiltPitch;
    prevErrRateRoll  = errTaxaRoll;
    prevErrRatePitch = errTaxaPitch;

    // Sem clamp de saida: autoridade total (o mixer garante os 1100..2000 µs do ESC)
    pidRoll  = PRate * errTaxaRoll  + IRate * intRoll  + derRoll;
    pidPitch = PRate * errTaxaPitch + IRate * intPitch + derPitch;

    // Yaw: laço de taxa puro (sem magnetometro, nao ha angulo absoluto)
    float erroYaw = -gyroZf;
    if (KiYaw > 0)
      intYaw = constrain(intYaw + erroYaw * dt, -ITERM_LIM, ITERM_LIM);
    pidYaw = YAW_SIGN * (KpYaw * erroYaw + KiYaw * intYaw);

    // Mixer X com redistribuicao de headroom (throttle vem do slider da web)
    float mA = throttleBase - pidRoll - pidPitch + pidYaw;
    float mB = throttleBase + pidRoll - pidPitch - pidYaw;
    float mC = throttleBase - pidRoll + pidPitch - pidYaw;
    float mD = throttleBase + pidRoll + pidPitch + pidYaw;

    float mx = fmaxf(fmaxf(mA, mB), fmaxf(mC, mD));
    if (mx > MOTOR_MAX) {                       // saturou em cima: desloca todos p/ baixo
      float d = mx - MOTOR_MAX;
      mA -= d; mB -= d; mC -= d; mD -= d;
      mx = MOTOR_MAX;                           // recalcula o topo antes do teste do piso
    }
    float mn = fminf(fminf(mA, mB), fminf(mC, mD));
    if (mn < MOTOR_IDLE) {                      // respeita o piso do ESC
      float d = fminf((float)(MOTOR_IDLE - mn), (float)(MOTOR_MAX - mx));
      mA += d; mB += d; mC += d; mD += d;
    }

    motorSpeedA = constrain((int)lroundf(mA), MOTOR_IDLE, MOTOR_MAX);
    motorSpeedB = constrain((int)lroundf(mB), MOTOR_IDLE, MOTOR_MAX);
    motorSpeedC = constrain((int)lroundf(mC), MOTOR_IDLE, MOTOR_MAX);
    motorSpeedD = constrain((int)lroundf(mD), MOTOR_IDLE, MOTOR_MAX);

    // TETO ABSOLUTO do slider "Max. throttle" - vale pra SAIDA dos motores, nao
    // so pro base. O PID nao tem clamp por design (o mixer era o limite), entao na
    // oscilacao ele mandava 2000 direto e o drone "decidia acelerar ao maximo"
    // mesmo com o teto baixo (diagnostico (5)). Agora NENHUM motor passa do teto.
    if (throttleMax < motorSpeedA) motorSpeedA = throttleMax;
    if (throttleMax < motorSpeedB) motorSpeedB = throttleMax;
    if (throttleMax < motorSpeedC) motorSpeedC = throttleMax;
    if (throttleMax < motorSpeedD) motorSpeedD = throttleMax;

    uint32_t tEsc0 = micros();
    motorA.writeMicroseconds(motorSpeedA);
    motorB.writeMicroseconds(motorSpeedB);
    motorC.writeMicroseconds(motorSpeedC);
    motorD.writeMicroseconds(motorSpeedD);
    uint32_t tEscus = micros() - tEsc0;
    if (tEscus > tEscmax) tEscmax = tEscus;
    tEscsum += tEscus;

    // GRAVACAO DE VOO (25 Hz POR TEMPO): agenda de 40 ms por amostra — nao
    // depende da taxa real do loop (hoje ele roda a ~200 Hz, nao 250)
    if (vooAtivo) {
      if (now - vooUltima >= VOO_AMOSTRA_US) {
        vooUltima += VOO_AMOSTRA_US;
        if (vooN >= MAX_AMOSTRAS) {
          vooAtivo = false; vooTruncado = true;
          logEvento("Gravacao CHEIA: 60 s de voo");
        } else {
          Amostra& a = vooLog[vooN++];
          uint32_t dtms = millis() - vooT0;
          a.tms   = dtms > 65535 ? 65535 : (uint16_t)dtms;
          a.roll  = (int16_t)constrain(roll  * 100.0f, -32000.0f, 32000.0f);
          a.pitch = (int16_t)constrain(pitch * 100.0f, -32000.0f, 32000.0f);
          a.gx    = (int16_t)constrain(gyroXf * 10.0f, -32000.0f, 32000.0f);
          a.gy    = (int16_t)constrain(gyroYf * 10.0f, -32000.0f, 32000.0f);
          a.gz    = (int16_t)constrain(gyroZf * 10.0f, -32000.0f, 32000.0f);
          a.vib   = (int16_t)constrain(vibRms * 1000.0f, 0.0f, 32000.0f);
          a.pidR  = (int16_t)constrain(pidRoll,  -32000.0f, 32000.0f);
          a.pidP  = (int16_t)constrain(pidPitch, -32000.0f, 32000.0f);
          a.pidY  = (int16_t)constrain(pidYaw,   -32000.0f, 32000.0f);
          a.mA = (int16_t)motorSpeedA;
          a.mB = (int16_t)motorSpeedB;
          a.mC = (int16_t)motorSpeedC;
          a.mD = (int16_t)motorSpeedD;
        }
      }
    }
  }

  uint32_t tCtrlus = micros() - tCtrl0;
  if (tCtrlus > tCtrlmax) tCtrlmax = tCtrlus;
  tCtrlsum += tCtrlus;

  // ------------------------- DIAGNOSTICO DO CICLO -------------------------
  cicloUs = micros() - tInicio;
  if (cicloCalibrando) {
    cicloCalibrando = false;           // ciclo da calibracao (~4 s): nao conta
  } else {
    if (cicloUs > cicloMaxUs) cicloMaxUs = cicloUs;
    if (cicloUs >= PERIOD_US) estouros++;
    tCicloSoma += cicloUs; tCicloN++;
  }

  // ------------------------- TELEMETRIA 20 Hz -------------------------
  enviarTelemetria(now);
}
