# Interface de Controle do Phybot (Simulação)

Este projeto implementa a interface mobile para controle do Phybot com:

- controle de direção (`Joystick` e `D-pad`)
- controle de velocidade (slider `0..255`)
- parada de emergência
- visualização em tempo real dos sensores
- servidor de simulação no notebook (sem hardware ESP32)

## Estrutura

```text
phybot/
├── web/
│   ├── index.html
│   └── controller.js
└── simulator/
    ├── server.py
    └── requirements.txt
```

## Requisitos

- Python 3.10+
- `uv` instalado

## Como rodar no notebook

1. Entre na pasta da simulação:

```bash
cd simulator
```

2. Crie o ambiente virtual com `uv`:

```bash
uv venv
source .venv/bin/activate
```

3. Instale dependências:

```bash
uv pip install -r requirements.txt
```

4. Inicie o servidor:

```bash
python server.py
```

5. Abra no navegador:

- no notebook: `http://localhost:8000`
- no celular (mesma rede WiFi): `http://IP_DO_NOTEBOOK:8000`

## Como usar

- Ajuste velocidade no slider (`0..255`)
- Alterne modo em `Modo: Joystick` / `Modo: D-pad`
- Use joystick virtual ou botões direcionais
- Use `PARADA DE EMERGENCIA` para enviar `stop`

## Protocolo WebSocket (`/ws`)

Mensagens enviadas pela interface:

```json
{ "type": "motor", "left": 120, "right": 120 }
```

```json
{ "type": "stop" }
```

Mensagem de sensores enviada pela simulação:

```json
{
  "type": "sensors",
  "connected": ["hcsr04", "vl53l0x", "mpu6050", "encoders", "battery"],
  "hcsr04": { "dist_cm": 88.7 },
  "vl53l0x": { "dist_mm": 421 },
  "mpu6050": { "pitch": 3.4, "roll": -1.2, "ax": 0.01, "ay": -0.03, "az": 9.81 },
  "encoders": { "rpm_l": 40, "rpm_r": 38 },
  "battery": { "voltage": 5.79 }
}
```

## Observações

- A URL do WebSocket no cliente usa `ws://${window.location.host}/ws`, então funciona tanto no notebook quanto futuramente no ESP32 sem alterar o `controller.js`.
- Esta fase é somente simulação. Integração com hardware fica para a fase ESP32.
