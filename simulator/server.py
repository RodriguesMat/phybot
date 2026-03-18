import asyncio
import json
import math
import time
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles


BASE_DIR = Path(__file__).resolve().parent
WEB_DIR = BASE_DIR.parent / "web"

# ---------------------------------------------------------------------------
# Cinemática diferencial — parâmetros do robô (espelho do código MATLAB)
# ---------------------------------------------------------------------------
_W = 0.15        # distância entre rodas [m]
_R_WHEEL = 0.03  # raio da roda [m]
_MAX_OMEGA = 3.0 # velocidade angular máxima [rad/s]
_VR_MAX = _MAX_OMEGA * _R_WHEEL  # 0.09 m/s


def _compute_trajectory() -> list[dict]:
    """Retorna os 4 segmentos de trajetória do MATLAB convertidos para Python."""
    segs = []
    VR_MAX = _VR_MAX

    # Trecho 01 — Reta 400 mm
    vr, vl = VR_MAX, VR_MAX
    segs.append({"name": "Reta 400 mm", "vr": vr, "vl": vl,
                  "duration": 0.4 / ((vr + vl) / 2)})

    # Trecho 02 — Curva R=200 mm (90°)
    R = 0.2
    vr = VR_MAX
    vl = vr * (R - _W / 2) / (R + _W / 2)
    segs.append({"name": "Curva R=200 mm (90°)", "vr": vr, "vl": vl,
                  "duration": 0.5 * math.pi / ((vr - vl) / _W)})

    # Trecho 03 — Reta 400 mm
    vr, vl = VR_MAX, VR_MAX
    segs.append({"name": "Reta 400 mm", "vr": vr, "vl": vl,
                  "duration": 0.4 / ((vr + vl) / 2)})

    # Trecho 04 — Curva R=600 mm (270°)
    R = 0.6
    vr = VR_MAX
    vl = vr * (R - _W / 2) / (R + _W / 2)
    segs.append({"name": "Curva R=600 mm (270°)", "vr": vr, "vl": vl,
                  "duration": 1.5 * math.pi / ((vr - vl) / _W)})

    return segs


TRAJECTORY = _compute_trajectory()


def _vel_to_pwm(vel: float) -> int:
    """Converte velocidade linear [m/s] para PWM [-255, 255]."""
    return max(-255, min(255, int(vel / _VR_MAX * 255)))

app = FastAPI(title="Phybot Simulator")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/assets", StaticFiles(directory=WEB_DIR), name="assets")


class RobotState:
    def __init__(self) -> None:
        self.left_cmd = 0
        self.right_cmd = 0
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.battery_v = 5.8
        self.last_cmd_at = time.time()
        self.connected = ["hcsr04", "vl53l0x", "mpu6050", "encoders", "battery"]
        self.nav_active = False
        self.nav_task: Optional[asyncio.Task] = None

    def update_commands(self, left: int, right: int) -> None:
        self.left_cmd = max(-255, min(255, int(left)))
        self.right_cmd = max(-255, min(255, int(right)))
        self.last_cmd_at = time.time()

    def stop(self) -> None:
        self.left_cmd = 0
        self.right_cmd = 0
        self.last_cmd_at = time.time()

    def tick(self, t: float) -> dict:
        # Valores estáticos dos sensores (por enquanto)
        return {
            "type": "sensors",
            "connected": self.connected,
            "hcsr04": {"dist_cm": 23.5},
            "vl53l0x": {"dist_mm": 342},
            "mpu6050": {
                "pitch": 5.2,
                "roll": -1.3,
                "ax": 0.12,
                "ay": -0.03,
                "az": 9.78,
                "gx": 1.2,
                "gy": -0.5,
                "gz": 0.1,
            },
            "encoders": {"rpm_l": 0, "rpm_r": 0},
            "battery": {"voltage": 5.8},
            "sim_time_s": round(t, 2),
        }


state = RobotState()


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


@app.get("/{full_path:path}")
async def web_files(full_path: str) -> FileResponse:
    target = WEB_DIR / full_path
    if target.is_file():
        return FileResponse(target)
    return FileResponse(WEB_DIR / "index.html")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    print("[SIM] Cliente conectado")

    await websocket.send_text(
        json.dumps(
            {
                "type": "hello",
                "mode": "simulation",
                "connected": state.connected,
            }
        )
    )

    sender = asyncio.create_task(_sender_loop(websocket))
    receiver = asyncio.create_task(_receiver_loop(websocket))
    done, pending = await asyncio.wait(
        [sender, receiver], return_when=asyncio.FIRST_COMPLETED
    )

    for task in pending:
        task.cancel()
    for task in done:
        _ = task.exception() if not task.cancelled() else None

    print("[SIM] Cliente desconectado")


async def _run_navigation(state: RobotState, websocket: WebSocket) -> None:
    """Executa os segmentos de trajetória sequencialmente."""
    try:
        state.nav_active = True
        total = len(TRAJECTORY)
        print(f"[NAV] Iniciando trajetória ({total} segmentos)")

        for i, seg in enumerate(TRAJECTORY):
            left_pwm = _vel_to_pwm(seg["vl"])
            right_pwm = _vel_to_pwm(seg["vr"])
            state.update_commands(left_pwm, right_pwm)
            print(f"[NAV] Seg {i+1}/{total}: {seg['name']} "
                  f"({seg['duration']:.1f}s) L={left_pwm} R={right_pwm}")

            await websocket.send_text(json.dumps({
                "type": "nav_status",
                "active": True,
                "segment": i + 1,
                "total": total,
                "name": seg["name"],
                "duration": round(seg["duration"], 1),
            }))

            await asyncio.sleep(seg["duration"])

        state.stop()
        state.nav_active = False
        print("[NAV] Trajetória concluída")
        await websocket.send_text(json.dumps({
            "type": "nav_status",
            "active": False,
            "segment": total,
            "total": total,
            "name": "Concluída",
            "duration": 0,
        }))

    except asyncio.CancelledError:
        state.stop()
        state.nav_active = False
        print("[NAV] Cancelada pelo usuário")
        await websocket.send_text(json.dumps({
            "type": "nav_status",
            "active": False,
            "segment": 0,
            "total": len(TRAJECTORY),
            "name": "Cancelada",
            "duration": 0,
        }))
        raise


async def _sender_loop(websocket: WebSocket) -> None:
    while True:
        payload = state.tick(time.time())
        await websocket.send_text(json.dumps(payload))
        await asyncio.sleep(0.1)


async def _receiver_loop(websocket: WebSocket) -> None:
    while True:
        try:
            message = await websocket.receive_text()
        except WebSocketDisconnect:
            break

        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            print(f"[SIM] JSON invalido: {message}")
            continue

        msg_type = data.get("type")
        if msg_type == "motor":
            if state.nav_active:
                continue  # ignora comandos manuais durante navegação
            left = data.get("left", 0)
            right = data.get("right", 0)
            state.update_commands(left, right)
            print(f"[SIM] MOTOR left={state.left_cmd} right={state.right_cmd}")
        elif msg_type == "speed":
            value = data.get("value", 0)
            print(f"[SIM] SPEED value={value}")
        elif msg_type == "stop":
            if state.nav_active and state.nav_task:
                state.nav_task.cancel()
            else:
                state.stop()
                print("[SIM] STOP")
        elif msg_type == "navigate":
            if state.nav_active:
                print("[NAV] Já em execução, ignorando")
                continue
            state.nav_task = asyncio.create_task(
                _run_navigation(state, websocket)
            )
        elif msg_type == "nav_cancel":
            if state.nav_task and not state.nav_task.done():
                state.nav_task.cancel()
        else:
            print(f"[SIM] Mensagem desconhecida: {data}")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("server:app", host="0.0.0.0", port=8000, reload=False)
