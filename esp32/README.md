# ESP32 — Firmware do Phybot

O ESP32 hospeda ele mesmo o servidor HTTP + WebSocket (`/ws`) com o
**mesmo protocolo** do simulador Python. O celular conecta direto ao
IP do ESP32 — sem PC intermediário.

```
Celular (browser)  ←──WiFi──→  ESP32
                               ├── HTTP  → serve index.html + controller.js
                               └── WS /ws → recebe motor/navigate, envia sensors/nav_status
```

---

## Estrutura

```
esp32/
├── phybot/
│   ├── phybot.ino      ← sketch principal
│   └── data/           ← arquivos gravados no LittleFS
│       ├── index.html
│       └── controller.js
└── README.md
```

---

## Bibliotecas necessárias

Instale pelo Arduino IDE (Ferramentas → Gerenciar Bibliotecas) ou PlatformIO:

| Biblioteca | Autor | Versão recomendada |
|---|---|---|
| ESPAsyncWebServer | me-no-dev | última |
| AsyncTCP | me-no-dev | última |
| ArduinoJson | bblanchon | v7 |

> **ESPAsyncWebServer** e **AsyncTCP** não estão no registro oficial.
> Instale pelo `.zip` ou pelo link do repositório no GitHub.

---

## Como gravar

### 1. Preparar os arquivos web

Copie `web/index.html` e `web/controller.js` para `esp32/phybot/data/`:

```bash
cp web/index.html    esp32/phybot/data/
cp web/controller.js esp32/phybot/data/
```

### 2. Configurar credenciais WiFi

Abra `phybot.ino` e edite:

```cpp
const char* WIFI_SSID = "SUA_REDE";
const char* WIFI_PASS = "SUA_SENHA";
```

### 3. Ajustar pinos do L298N

Confira a seção `PINOS` no `phybot.ino` e adapte aos seus GPIOs:

```cpp
#define MOTOR_L_EN   25   // EN A (PWM)
#define MOTOR_L_IN1  26
#define MOTOR_L_IN2  27
#define MOTOR_R_EN   14   // EN B (PWM)
#define MOTOR_R_IN1  12
#define MOTOR_R_IN2  13
```

### 4. Gravar os arquivos web no LittleFS

No Arduino IDE:
- Instale o plugin **ESP32 LittleFS Data Upload**
- Ferramentas → **ESP32 Sketch Data Upload**

### 5. Gravar o firmware

- Selecione a placa **ESP32 Dev Module**
- Clique em **Upload** normalmente

### 6. Usar

- Abra o Monitor Serial (115200 baud)
- Aguarde a mensagem com o IP: `http://192.168.x.x`
- Abra esse IP no celular (mesma rede WiFi)

---

## Protocolo WebSocket

### Recebido (web → ESP32)

```json
{ "type": "motor",    "left": 120, "right": 120 }
{ "type": "stop" }
{ "type": "speed",    "value": 200 }
{ "type": "navigate" }
{ "type": "nav_cancel" }
```

### Enviado (ESP32 → web)

```json
{ "type": "hello", "mode": "hardware", "connected": ["hcsr04", ...] }
{ "type": "sensors", "hcsr04": {"dist_cm": 23.5}, ... }
{ "type": "nav_status", "active": true, "segment": 2, "total": 4, "name": "Curva R=200mm (90 graus)", "duration": 4.8 }
```

---

## Trajetória automática

A trajetória é calculada no `setup()` a partir dos mesmos parâmetros do MATLAB:

| Seg | Descrição | Duração |
|---|---|---|
| 1 | Reta 400 mm | ~4.4 s |
| 2 | Curva R=200 mm (90°) | ~4.8 s |
| 3 | Reta 400 mm | ~4.4 s |
| 4 | Curva R=600 mm (270°) | ~35.3 s |

Botão **Navegar** na interface inicia. Durante a navegação o controle
manual (joystick/dpad) é bloqueado. Botão **Cancelar** ou
**PARADA DE EMERGÊNCIA** interrompem e param os motores.

---

## Adicionando sensores reais

Localize o comentário `// TODO: substitua pelos valores reais` em `broadcastSensors()`
e substitua os zeros pelas leituras das suas bibliotecas (HC-SR04, VL53L0X, MPU-6050, encoders).
