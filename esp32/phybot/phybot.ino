// ============================================================
// phybot.ino — Firmware ESP32 do Phybot
//
// O ESP32 hospeda o próprio servidor HTTP + WebSocket (/ws)
// com o mesmo protocolo do simulador Python.  O celular se
// conecta direto ao IP do ESP32 — sem PC intermediário.
//
// Protocolo recebido (web → ESP32):
//   { "type": "motor",    "left": 120, "right": 120 }
//   { "type": "stop" }
//   { "type": "speed",    "value": 200 }
//   { "type": "navigate" }
//   { "type": "nav_cancel" }
//
// Protocolo enviado (ESP32 → web):
//   { "type": "hello",      "mode": "hardware", "connected": [...] }
//   { "type": "sensors",    ... }
//   { "type": "nav_status", "active": true/false, "segment": N, ... }
//
// Bibliotecas necessárias (instale no Arduino IDE / PlatformIO):
//   - ESPAsyncWebServer  (me-no-dev/ESPAsyncWebServer)
//   - AsyncTCP           (me-no-dev/AsyncTCP)
//   - ArduinoJson        (bblanchon/ArduinoJson  v7)
//
// Arquivos web: copie web/index.html e web/controller.js
//               para a pasta data/ deste sketch e grave via
//               "ESP32 Sketch Data Upload" (LittleFS).
// ============================================================

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>

// ============================================================
// CONFIGURAÇÕES — edite antes de gravar
// ============================================================
const char* WIFI_SSID = "SUA_REDE";
const char* WIFI_PASS = "SUA_SENHA";

// ============================================================
// PINOS — L298N
// Ajuste conforme a sua fiação
// ============================================================
//  Motor esquerdo
#define MOTOR_L_EN   25   // EN A  (PWM)
#define MOTOR_L_IN1  26   // IN1
#define MOTOR_L_IN2  27   // IN2
//  Motor direito
#define MOTOR_R_EN   14   // EN B  (PWM)
#define MOTOR_R_IN1  12   // IN3
#define MOTOR_R_IN2  13   // IN4

#define LEDC_CH_L    0    // canal LEDC motor esquerdo
#define LEDC_CH_R    1    // canal LEDC motor direito
#define PWM_FREQ     1000 // Hz
#define PWM_BITS     8    // resolução 0-255

// ============================================================
// PARÂMETROS DO ROBÔ — espelho do código MATLAB
// ============================================================
static constexpr float _W         = 0.15f;  // distância entre rodas [m]
static constexpr float _R_WHEEL   = 0.03f;  // raio da roda [m]
static constexpr float _MAX_OMEGA = 3.0f;   // omega máxima das rodas [rad/s]
static constexpr float _VR_MAX    = _MAX_OMEGA * _R_WHEEL; // 0.09 m/s

struct Segment {
    const char* name;
    float vr;        // velocidade roda direita [m/s]
    float vl;        // velocidade roda esquerda [m/s]
    unsigned long durationMs; // duração em milissegundos
};

static Segment TRAJECTORY[4];

void computeTrajectory() {
    float vr, vl, psi_dot, Rcir;

    // Trecho 01 — Reta 400 mm
    vr = vl = _VR_MAX;
    TRAJECTORY[0] = {"Reta 400 mm", vr, vl,
                      (unsigned long)(0.4f / ((vr + vl) / 2.0f) * 1000.0f)};

    // Trecho 02 — Curva R=200 mm (90°)
    Rcir    = 0.2f;
    vr      = _VR_MAX;
    vl      = vr * (Rcir - _W / 2.0f) / (Rcir + _W / 2.0f);
    psi_dot = (vr - vl) / _W;
    TRAJECTORY[1] = {"Curva R=200mm (90 graus)", vr, vl,
                      (unsigned long)(0.5f * M_PI / psi_dot * 1000.0f)};

    // Trecho 03 — Reta 400 mm
    vr = vl = _VR_MAX;
    TRAJECTORY[2] = {"Reta 400 mm", vr, vl,
                      (unsigned long)(0.4f / ((vr + vl) / 2.0f) * 1000.0f)};

    // Trecho 04 — Curva R=600 mm (270°)
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
void motorsInit() {
    ledcSetup(LEDC_CH_L, PWM_FREQ, PWM_BITS);
    ledcSetup(LEDC_CH_R, PWM_FREQ, PWM_BITS);
    ledcAttachPin(MOTOR_L_EN, LEDC_CH_L);
    ledcAttachPin(MOTOR_R_EN, LEDC_CH_R);
    pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);
}

void setMotors(int left, int right) {
    left  = constrain(left,  -255, 255);
    right = constrain(right, -255, 255);

    digitalWrite(MOTOR_L_IN1, left  >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_L_IN2, left  >= 0 ? LOW  : HIGH);
    ledcWrite(LEDC_CH_L, abs(left));

    digitalWrite(MOTOR_R_IN1, right >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_IN2, right >= 0 ? LOW  : HIGH);
    ledcWrite(LEDC_CH_R, abs(right));
}

void stopMotors() {
    ledcWrite(LEDC_CH_L, 0);
    ledcWrite(LEDC_CH_R, 0);
}

// ============================================================
// SERVIDOR HTTP + WEBSOCKET
// ============================================================
AsyncWebServer httpServer(80);
AsyncWebSocket ws("/ws");

// ============================================================
// NAVEGAÇÃO — state machine baseada em millis(), sem delay()
// ============================================================
static bool          navActive   = false;
static int           navSegIdx   = 0;
static unsigned long navSegStart = 0;

void broadcastNavStatus(bool active, int segIdx, const char* name, unsigned long dMs) {
    StaticJsonDocument<256> doc;
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

void navStart() {
    if (navActive) return;
    navActive   = true;
    navSegIdx   = 0;
    navSegStart = millis();

    Segment& s = TRAJECTORY[0];
    setMotors(velToPWM(s.vl), velToPWM(s.vr));
    Serial.printf("[NAV] Seg 1/4: %s (%lums)\n", s.name, s.durationMs);
    broadcastNavStatus(true, 0, s.name, s.durationMs);
}

void navCancel() {
    if (!navActive) return;
    navActive = false;
    stopMotors();
    broadcastNavStatus(false, 0, "Cancelada", 0);
    Serial.println("[NAV] Cancelada");
}

// Chamada a cada iteração do loop() — sem bloquear
void navTick() {
    if (!navActive) return;

    Segment& seg = TRAJECTORY[navSegIdx];
    if (millis() - navSegStart < seg.durationMs) return; // ainda no segmento

    navSegIdx++;
    if (navSegIdx >= 4) {
        navActive = false;
        stopMotors();
        broadcastNavStatus(false, 3, "Concluida", 0);
        Serial.println("[NAV] Concluida");
        return;
    }

    // Avança para o próximo segmento
    navSegStart = millis();
    Segment& next = TRAJECTORY[navSegIdx];
    setMotors(velToPWM(next.vl), velToPWM(next.vr));
    Serial.printf("[NAV] Seg %d/4: %s (%lums)\n",
                  navSegIdx + 1, next.name, next.durationMs);
    broadcastNavStatus(true, navSegIdx, next.name, next.durationMs);
}

// ============================================================
// TRATAMENTO DAS MENSAGENS WEBSOCKET
// ============================================================
void handleMessage(uint8_t* data, size_t len) {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, data, len)) return;

    const char* type = doc["type"] | "";

    if (strcmp(type, "motor") == 0) {
        if (navActive) return;  // bloqueia controle manual durante navegação
        setMotors(doc["left"] | 0, doc["right"] | 0);

    } else if (strcmp(type, "stop") == 0) {
        if (navActive) navCancel();
        else           stopMotors();

    } else if (strcmp(type, "navigate") == 0) {
        navStart();

    } else if (strcmp(type, "nav_cancel") == 0) {
        navCancel();

    } else if (strcmp(type, "speed") == 0) {
        // slider de velocidade — apenas registra (aplicado via motor)
        Serial.printf("[WS] speed=%d\n", doc["value"] | 0);
    }
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Cliente %u conectado\n", client->id());

        // Envia hello com o mesmo formato do simulador
        StaticJsonDocument<256> doc;
        doc["type"] = "hello";
        doc["mode"] = "hardware";
        JsonArray conn = doc.createNestedArray("connected");
        conn.add("hcsr04"); conn.add("vl53l0x"); conn.add("mpu6050");
        conn.add("encoders"); conn.add("battery");
        String payload;
        serializeJson(doc, payload);
        client->text(payload);

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Cliente %u desconectado\n", client->id());
        if (navActive) navCancel();
        else           stopMotors();

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        // Processa apenas frames completos e de texto
        if (info->final && info->index == 0 &&
            info->len == len && info->opcode == WS_TEXT) {
            handleMessage(data, len);
        }
    }
}

// ============================================================
// SENSORES — broadcast periódico
// Substitua os valores estáticos pelas leituras reais
// ============================================================
static unsigned long lastSensorMs = 0;

void broadcastSensors() {
    StaticJsonDocument<512> doc;
    doc["type"] = "sensors";

    JsonArray conn = doc.createNestedArray("connected");
    conn.add("hcsr04"); conn.add("vl53l0x"); conn.add("mpu6050");
    conn.add("encoders"); conn.add("battery");

    // TODO: substitua pelos valores reais dos sensores
    doc["hcsr04"]["dist_cm"]  = 0;
    doc["vl53l0x"]["dist_mm"] = 0;
    doc["mpu6050"]["pitch"]   = 0;
    doc["mpu6050"]["roll"]    = 0;
    doc["mpu6050"]["ax"]      = 0;
    doc["mpu6050"]["ay"]      = 0;
    doc["mpu6050"]["az"]      = 9.81;
    doc["mpu6050"]["gx"]      = 0;
    doc["mpu6050"]["gy"]      = 0;
    doc["mpu6050"]["gz"]      = 0;
    doc["encoders"]["rpm_l"]  = 0;
    doc["encoders"]["rpm_r"]  = 0;
    doc["battery"]["voltage"] = 0;

    String payload;
    serializeJson(doc, payload);
    ws.textAll(payload);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    motorsInit();
    stopMotors();
    computeTrajectory();

    // Log da trajetória calculada
    Serial.println("=== Trajetoria calculada ===");
    for (int i = 0; i < 4; i++) {
        Serial.printf("  Seg %d: %-26s VR=%.4f VL=%.4f  t=%.2fs\n",
                      i + 1, TRAJECTORY[i].name,
                      TRAJECTORY[i].vr, TRAJECTORY[i].vl,
                      TRAJECTORY[i].durationMs / 1000.0f);
    }
    Serial.println("============================");

    // LittleFS — arquivos web (index.html, controller.js)
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] Falha ao montar LittleFS");
    }

    // WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WiFi] Conectando");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.printf("\n[WiFi] Conectado! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] Abra no celular: http://%s\n", WiFi.localIP().toString().c_str());

    // WebSocket
    ws.onEvent(onWsEvent);
    httpServer.addHandler(&ws);

    // Serve arquivos da pasta data/ (index.html, controller.js)
    httpServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    httpServer.begin();
    Serial.println("[HTTP] Servidor iniciado");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    navTick();  // avança a state machine de navegação

    if (ws.count() > 0 && millis() - lastSensorMs >= 100) {
        lastSensorMs = millis();
        broadcastSensors();
    }

    ws.cleanupClients();
}
