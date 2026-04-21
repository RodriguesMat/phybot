// ============================================================
// phybot.ino — Firmware ESP32 do Phybot
//
// O ESP32 hospeda o proprio servidor HTTP + WebSocket (/ws)
// com o mesmo protocolo do simulador Python.  O celular se
// conecta direto ao IP do ESP32 - sem PC intermediario.
//
// Compativel com Arduino ESP32 core 3.x  (API LEDC nova).
//
// Protocolo recebido (web -> ESP32):
//   { "type": "motor",    "left": 120, "right": 120 }
//   { "type": "stop" }
//   { "type": "speed",    "value": 200 }
//   { "type": "navigate" }
//   { "type": "nav_cancel" }
//
// Protocolo enviado (ESP32 -> web):
//   { "type": "hello",      "mode": "hardware", "connected": [...] }
//   { "type": "sensors",    ... }
//   { "type": "nav_status", "active": true/false, "segment": N, ... }
//
// Bibliotecas necessarias (instale via Library Manager):
//   - ESPAsyncWebServer  (ESP32Async)          -- NAO a me-no-dev (arquivada)
//   - AsyncTCP           (ESP32Async)
//   - ArduinoJson        (bblanchon, v7.x)
//   - Adafruit_VL53L0X   (Adafruit)
//
// Arquivos web: copie web/index.html e web/controller.js
//               para a pasta data/ deste sketch e grave via
//               "ESP32 LittleFS Data Upload" (plugin do Arduino IDE).
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

// ============================================================
// CONFIGURACOES - edite antes de gravar
// ============================================================
static const char* WIFI_SSID = "SUA_REDE";
static const char* WIFI_PASS = "SUA_SENHA";

// Hostname no WiFi (fica phybot.local em redes com mDNS)
static const char* WIFI_HOSTNAME = "phybot";

// ============================================================
// PINOS
//   Motores: PWM direto em IN1/IN2 (mesma estrategia do teste
//   validado).  Para girar horario: PWM em IN1, 0 em IN2;
//   anti-horario: 0 em IN1, PWM em IN2.  Jumper EN da ponte-H
//   amarrado em VCC (modo habilitado fixo).
// ============================================================
//  Motor esquerdo (4WD: os 2 motores do lado esquerdo em paralelo)
#define M_L_IN1   19
#define M_L_IN2   18
//  Motor direito (4WD: os 2 motores do lado direito em paralelo)
#define M_R_IN1    5
#define M_R_IN2   17

// Sensores
#define PINO_TRIG  2
#define PINO_ECHO  4
#define MPU_ADDR   0x68

// PWM
#define PWM_FREQ   5000   // Hz
#define PWM_BITS   8      // 0..255

// ============================================================
// PARAMETROS DO ROBO - espelho do MATLAB
// ============================================================
static constexpr float _W         = 0.15f;   // dist. entre rodas [m]
static constexpr float _R_WHEEL   = 0.03f;   // raio da roda [m]
static constexpr float _MAX_OMEGA = 3.0f;    // omega maxima [rad/s]
static constexpr float _VR_MAX    = _MAX_OMEGA * _R_WHEEL; // 0.09 m/s

struct Segment {
  const char*   name;
  float         vr;          // velocidade roda direita [m/s]
  float         vl;          // velocidade roda esquerda [m/s]
  unsigned long durationMs;
};

static Segment TRAJECTORY[4];

static void computeTrajectory() {
  float vr, vl, psi_dot, Rcir;

  // Trecho 01 - Reta 400 mm
  vr = vl = _VR_MAX;
  TRAJECTORY[0] = {"Reta 400 mm", vr, vl,
                   (unsigned long)(0.4f / ((vr + vl) / 2.0f) * 1000.0f)};

  // Trecho 02 - Curva R=200 mm (90 graus)
  Rcir    = 0.2f;
  vr      = _VR_MAX;
  vl      = vr * (Rcir - _W / 2.0f) / (Rcir + _W / 2.0f);
  psi_dot = (vr - vl) / _W;
  TRAJECTORY[1] = {"Curva R=200mm (90 graus)", vr, vl,
                   (unsigned long)(0.5f * M_PI / psi_dot * 1000.0f)};

  // Trecho 03 - Reta 400 mm
  vr = vl = _VR_MAX;
  TRAJECTORY[2] = {"Reta 400 mm", vr, vl,
                   (unsigned long)(0.4f / ((vr + vl) / 2.0f) * 1000.0f)};

  // Trecho 04 - Curva R=600 mm (270 graus)
  Rcir    = 0.6f;
  vr      = _VR_MAX;
  vl      = vr * (Rcir - _W / 2.0f) / (Rcir + _W / 2.0f);
  psi_dot = (vr - vl) / _W;
  TRAJECTORY[3] = {"Curva R=600mm (270 graus)", vr, vl,
                   (unsigned long)(1.5f * M_PI / psi_dot * 1000.0f)};
}

static int velToPWM(float vel) {
  return constrain((int)(fabsf(vel) / _VR_MAX * 255.0f), 0, 255);
}

// ============================================================
// MOTORES
// ============================================================
static void motorsInit() {
  // Core 3.x: ledcAttach(pin, freq, resolution) -- canal auto-alocado
  ledcAttach(M_L_IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(M_L_IN2, PWM_FREQ, PWM_BITS);
  ledcAttach(M_R_IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(M_R_IN2, PWM_FREQ, PWM_BITS);
  ledcWrite(M_L_IN1, 0); ledcWrite(M_L_IN2, 0);
  ledcWrite(M_R_IN1, 0); ledcWrite(M_R_IN2, 0);
}

// left/right: -255..+255 (sinal = sentido)
static void setMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  // Motor esquerdo
  if (left >= 0) { ledcWrite(M_L_IN1, left);  ledcWrite(M_L_IN2, 0); }
  else           { ledcWrite(M_L_IN1, 0);     ledcWrite(M_L_IN2, -left); }

  // Motor direito
  if (right >= 0) { ledcWrite(M_R_IN1, right); ledcWrite(M_R_IN2, 0); }
  else            { ledcWrite(M_R_IN1, 0);     ledcWrite(M_R_IN2, -right); }
}

static void stopMotors() {
  ledcWrite(M_L_IN1, 0); ledcWrite(M_L_IN2, 0);
  ledcWrite(M_R_IN1, 0); ledcWrite(M_R_IN2, 0);
}

// ============================================================
// SENSORES
// ============================================================
static Adafruit_VL53L0X lox;

struct MpuData {
  float accX, accY, accZ;
  float gyrX, gyrY, gyrZ;
  float pitch;  // anguloXZ (inclinacao frente)
  float roll;   // anguloYZ (inclinacao lado)
};

// Ultimas leituras (usadas tanto no broadcast quanto na navegacao)
static float    lastDistCm   = -1.0f;   // HC-SR04   (cm)
static int      lastDistMm   = -1;      // VL53L0X   (mm, -1 se erro)
static MpuData  lastMpu      = {0};
static float    anguloZ      = 0.0f;    // integral do gyroZ
static bool     mpuPresent   = false;
static bool     voxPresent   = false;

static unsigned long tMpuPrev = 0;  // para dt do gyroZ

// ---------- HC-SR04 ----------
static float readHCSR04() {
  digitalWrite(PINO_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PINO_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PINO_TRIG, LOW);
  // timeout 25 ms (~4.3 m) para nao travar o loop
  unsigned long dur = pulseIn(PINO_ECHO, HIGH, 25000UL);
  if (dur == 0) return -1.0f;
  return (dur * 0.0343f) / 2.0f;  // cm
}

// ---------- VL53L0X ----------
static int readVL53L0X() {
  if (!voxPresent) return -1;
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  if (m.RangeStatus != 4) return (int)m.RangeMilliMeter;
  return -1;
}

// ---------- MPU-6050 ----------
static bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);    // sai do sleep
  if (Wire.endTransmission(true) != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x18); // gyro +-2000 dps
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x18); // accel +-16g
  Wire.endTransmission();
  return true;
}

static bool readMPU(MpuData& d) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                   // temp (descartada)
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Fundos de escala: accel +-16g => 2048 LSB/g; gyro +-2000 dps => 16.4 LSB/dps
  const float ACC_S = 1.0f / 2048.0f;   // em "g"
  const float GYR_S = 1.0f / 16.4f;     // em dps

  d.accX = ax * ACC_S * 9.81f;          // m/s^2
  d.accY = ay * ACC_S * 9.81f;
  d.accZ = az * ACC_S * 9.81f;
  d.gyrX = gx * GYR_S;                  // dps
  d.gyrY = gy * GYR_S;
  d.gyrZ = gz * GYR_S;
  d.pitch = atan2f(d.accX, d.accZ) * 180.0f / M_PI;
  d.roll  = atan2f(d.accY, d.accZ) * 180.0f / M_PI;
  return true;
}

// ============================================================
// SERVIDOR HTTP + WEBSOCKET
// ============================================================
static AsyncWebServer httpServer(80);
static AsyncWebSocket ws("/ws");

// ============================================================
// NAVEGACAO - state machine baseada em millis(), sem delay()
// ============================================================
static bool          navActive   = false;
static int           navSegIdx   = 0;
static unsigned long navSegStart = 0;

static void broadcastNavStatus(bool active, int segIdx,
                               const char* name, unsigned long dMs) {
  JsonDocument doc;
  doc["type"]     = "nav_status";
  doc["active"]   = active;
  doc["segment"]  = active ? segIdx + 1 : 0;
  doc["total"]    = 4;
  doc["name"]     = name;
  doc["duration"] = dMs / 1000.0f;
  String payload;
  serializeJson(doc, payload);
  ws.textAll(payload);
}

static void navStart() {
  if (navActive) return;
  navActive   = true;
  navSegIdx   = 0;
  navSegStart = millis();
  Segment& s  = TRAJECTORY[0];
  setMotors(velToPWM(s.vl), velToPWM(s.vr));
  Serial.printf("[NAV] Seg 1/4: %s (%lums)\n", s.name, s.durationMs);
  broadcastNavStatus(true, 0, s.name, s.durationMs);
}

static void navCancel() {
  if (!navActive) return;
  navActive = false;
  stopMotors();
  broadcastNavStatus(false, 0, "Cancelada", 0);
  Serial.println("[NAV] Cancelada");
}

static void navTick() {
  if (!navActive) return;
  Segment& cur = TRAJECTORY[navSegIdx];
  if (millis() - navSegStart < cur.durationMs) return;

  navSegIdx++;
  if (navSegIdx >= 4) {
    navActive = false;
    stopMotors();
    broadcastNavStatus(false, 3, "Concluida", 0);
    Serial.println("[NAV] Concluida");
    return;
  }
  navSegStart = millis();
  Segment& nxt = TRAJECTORY[navSegIdx];
  setMotors(velToPWM(nxt.vl), velToPWM(nxt.vr));
  Serial.printf("[NAV] Seg %d/4: %s (%lums)\n",
                navSegIdx + 1, nxt.name, nxt.durationMs);
  broadcastNavStatus(true, navSegIdx, nxt.name, nxt.durationMs);
}

// ============================================================
// TRATAMENTO DAS MENSAGENS WEBSOCKET
// ============================================================
static void handleMessage(uint8_t* data, size_t len) {
  JsonDocument doc;
  if (deserializeJson(doc, data, len)) return;

  const char* type = doc["type"] | "";

  if (strcmp(type, "motor") == 0) {
    if (navActive) return;                        // bloqueia manual durante nav
    setMotors(doc["left"] | 0, doc["right"] | 0);

  } else if (strcmp(type, "stop") == 0) {
    if (navActive) navCancel();
    else           stopMotors();

  } else if (strcmp(type, "navigate") == 0) {
    navStart();

  } else if (strcmp(type, "nav_cancel") == 0) {
    navCancel();

  } else if (strcmp(type, "speed") == 0) {
    Serial.printf("[WS] speed=%d\n", doc["value"] | 0);
  }
}

static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Cliente %u conectado (%s)\n",
                  client->id(), client->remoteIP().toString().c_str());

    JsonDocument doc;
    doc["type"] = "hello";
    doc["mode"] = "hardware";
    JsonArray conn = doc["connected"].to<JsonArray>();
    conn.add("hcsr04");
    if (voxPresent) conn.add("vl53l0x");
    if (mpuPresent) conn.add("mpu6050");
    conn.add("encoders");
    conn.add("battery");
    String payload;
    serializeJson(doc, payload);
    client->text(payload);

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Cliente %u desconectado\n", client->id());
    // seguranca: se o ultimo cliente saiu, para tudo
    if (ws.count() == 0) {
      if (navActive) navCancel();
      else           stopMotors();
    }

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 &&
        info->len == len && info->opcode == WS_TEXT) {
      handleMessage(data, len);
    }
  }
}

// ============================================================
// BROADCAST DE SENSORES
// ============================================================
static void broadcastSensors() {
  JsonDocument doc;
  doc["type"] = "sensors";

  JsonArray conn = doc["connected"].to<JsonArray>();
  conn.add("hcsr04");
  if (voxPresent) conn.add("vl53l0x");
  if (mpuPresent) conn.add("mpu6050");
  conn.add("encoders");
  conn.add("battery");

  doc["hcsr04"]["dist_cm"]  = (lastDistCm < 0) ? (float)NAN : lastDistCm;
  doc["vl53l0x"]["dist_mm"] = lastDistMm;          // -1 se sem leitura
  doc["mpu6050"]["pitch"]   = lastMpu.pitch;
  doc["mpu6050"]["roll"]    = lastMpu.roll;
  doc["mpu6050"]["ax"]      = lastMpu.accX;
  doc["mpu6050"]["ay"]      = lastMpu.accY;
  doc["mpu6050"]["az"]      = lastMpu.accZ;
  doc["mpu6050"]["gx"]      = lastMpu.gyrX;
  doc["mpu6050"]["gy"]      = lastMpu.gyrY;
  doc["mpu6050"]["gz"]      = lastMpu.gyrZ;
  doc["mpu6050"]["yaw"]     = anguloZ;
  doc["encoders"]["rpm_l"]  = 0;                   // TODO: encoder real
  doc["encoders"]["rpm_r"]  = 0;
  doc["battery"]["voltage"] = 0;                   // TODO: divisor resistivo

  String payload;
  serializeJson(doc, payload);
  ws.textAll(payload);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== PHYBOT boot ===");

  // --- Motores ---
  motorsInit();
  stopMotors();

  // --- Trajetoria ---
  computeTrajectory();
  Serial.println("--- Trajetoria calculada ---");
  for (int i = 0; i < 4; i++) {
    Serial.printf("  Seg %d: %-26s VR=%.4f VL=%.4f t=%.2fs\n",
                  i + 1, TRAJECTORY[i].name,
                  TRAJECTORY[i].vr, TRAJECTORY[i].vl,
                  TRAJECTORY[i].durationMs / 1000.0f);
  }

  // --- HC-SR04 ---
  pinMode(PINO_TRIG, OUTPUT);
  pinMode(PINO_ECHO, INPUT);

  // --- I2C / MPU / VL53L0X ---
  Wire.begin();
  mpuPresent = mpuInit();
  Serial.printf("[MPU6050] %s\n", mpuPresent ? "OK" : "NAO encontrado");

  voxPresent = lox.begin();
  Serial.printf("[VL53L0X] %s\n", voxPresent ? "OK" : "NAO encontrado");

  // --- LittleFS ---
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] Falha ao montar LittleFS");
  } else {
    Serial.println("[FS] LittleFS montado");
  }

  // --- WiFi STA ---
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setSleep(false);                          // latencia menor em WS
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] Conectando a \"%s\"", WIFI_SSID);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] OK  IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    Serial.printf("[WiFi] Abra no celular: http://%s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] FALHOU - seguindo sem rede");
  }

  // --- HTTP + WS ---
  ws.onEvent(onWsEvent);
  httpServer.addHandler(&ws);
  httpServer.serveStatic("/", LittleFS, "/")
            .setDefaultFile("index.html");
  // fallback: se alguem pedir um caminho desconhecido, devolve index
  httpServer.onNotFound([](AsyncWebServerRequest* req) {
    if (LittleFS.exists("/index.html"))
      req->send(LittleFS, "/index.html", "text/html");
    else
      req->send(404, "text/plain", "index.html nao esta no LittleFS");
  });
  httpServer.begin();
  Serial.println("[HTTP] Servidor iniciado na porta 80");

  tMpuPrev = millis();
}

// ============================================================
// LOOP
// ============================================================
static unsigned long tLastMpu    = 0;   // 100 ms
static unsigned long tLastVL     = 0;   // 100 ms
static unsigned long tLastHC     = 0;   // 1000 ms
static unsigned long tLastBcast  = 0;   //  100 ms
static unsigned long tWifiCheck  = 0;   // 5 s

void loop() {
  unsigned long now = millis();

  // --- Reconexao WiFi (nao bloqueante) ---
  if (now - tWifiCheck > 5000) {
    tWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Reconectando...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }

  // --- Sensores ---
  if (mpuPresent && now - tLastMpu >= 100) {
    MpuData d;
    if (readMPU(d)) {
      lastMpu = d;
      // Integral do gyroZ para yaw (com offset do teste validado)
      const float ERRO_GZ = 1.15f;
      float dt = (now - tMpuPrev) / 1000.0f;
      anguloZ += (d.gyrZ - ERRO_GZ) * dt;
    }
    tMpuPrev = now;
    tLastMpu = now;
  }
  if (voxPresent && now - tLastVL >= 100) {
    lastDistMm = readVL53L0X();
    tLastVL    = now;
  }
  if (now - tLastHC >= 1000) {
    lastDistCm = readHCSR04();
    tLastHC    = now;
  }

  // --- Navegacao ---
  navTick();

  // --- Broadcast WS ---
  if (ws.count() > 0 && now - tLastBcast >= 100) {
    tLastBcast = now;
    broadcastSensors();
  }

  ws.cleanupClients();
}
