(function () {
  "use strict";

  const els = {
    connDot: document.getElementById("connDot"),
    connText: document.getElementById("connText"),
    sensorGrid: document.getElementById("sensorGrid"),
    speedSlider: document.getElementById("speedSlider"),
    speedValue: document.getElementById("speedValue"),
    modeToggle: document.getElementById("modeToggle"),
    joystickArea: document.getElementById("joystickArea"),
    joystickCanvas: document.getElementById("joystickCanvas"),
    dpadArea: document.getElementById("dpadArea"),
    emergencyBtn: document.getElementById("emergencyBtn"),
    navBtn: document.getElementById("navBtn"),
    navInfo: document.getElementById("navInfo"),
  };

  const state = {
    ws: null,
    isConnected: false,
    reconnectDelay: 1000,
    reconnectTimer: null,
    speed: Number(localStorage.getItem("phybotSpeed") || 150),
    mode: localStorage.getItem("phybotMode") || "joystick",
    lastSendAt: 0,
    sendIntervalMs: 50,
    leftCmd: 0,
    rightCmd: 0,
    joystickActive: false,
    joyNormX: 0,
    joyNormY: 0,
    dpadHold: null,
    cards: {},
    navActive: false,
  };

  const CARD_DEFS = {
    hcsr04: [
      {
        id: "hcsr04_dist_cm",
        title: "Dist. Ultrassom",
        format: (data) => `${num(data?.hcsr04?.dist_cm, 1)} cm`,
      },
    ],
    vl53l0x: [
      {
        id: "vl53l0x_dist_mm",
        title: "Dist. Laser",
        format: (data) => `${num(data?.vl53l0x?.dist_mm, 0)} mm`,
      },
    ],
    mpu6050: [
      {
        id: "mpu_pitch",
        title: "Pitch",
        format: (data) => `${num(data?.mpu6050?.pitch, 1)}°`,
      },
      {
        id: "mpu_roll",
        title: "Roll",
        format: (data) => `${num(data?.mpu6050?.roll, 1)}°`,
      },
    ],
    encoders: [
      {
        id: "enc_rpm_l",
        title: "RPM Esq",
        format: (data) => `${num(data?.encoders?.rpm_l, 0)}`,
      },
      {
        id: "enc_rpm_r",
        title: "RPM Dir",
        format: (data) => `${num(data?.encoders?.rpm_r, 0)}`,
      },
    ],
    battery: [
      {
        id: "battery_v",
        title: "Bateria",
        format: (data) => `${num(data?.battery?.voltage, 2)} V`,
      },
    ],
  };

  init();

  function init() {
    state.speed = clamp(state.speed, 0, 255);
    els.speedSlider.value = String(state.speed);
    els.speedValue.textContent = String(state.speed);
    applyMode(state.mode);
    bindUiEvents();
    connectWebSocket();
    renderEmptyCards();
    setupJoystick();
  }

  function bindUiEvents() {
    els.speedSlider.addEventListener("input", () => {
      state.speed = clamp(Number(els.speedSlider.value), 0, 255);
      localStorage.setItem("phybotSpeed", String(state.speed));
      els.speedValue.textContent = String(state.speed);
      sendJson({ type: "speed", value: state.speed }, true);
      if (state.mode === "joystick" && state.joystickActive) sendFromJoystick();
    });

    els.modeToggle.addEventListener("click", () => {
      const next = state.mode === "joystick" ? "dpad" : "joystick";
      applyMode(next);
    });

    els.emergencyBtn.addEventListener("click", emergencyStop);

    els.navBtn.addEventListener("click", () => {
      if (state.navActive) {
        sendJson({ type: "nav_cancel" }, true);
      } else {
        sendJson({ type: "navigate" }, true);
      }
    });

    const dpadButtons = Array.from(document.querySelectorAll(".dpad-btn"));
    dpadButtons.forEach((btn) => {
      const start = (ev) => {
        ev.preventDefault();
        btn.classList.add("active");
        const dir = btn.dataset.dir;
        state.dpadHold = dir || null;
        sendDpadDirection(dir);
      };
      const end = (ev) => {
        ev.preventDefault();
        btn.classList.remove("active");
        state.dpadHold = null;
        sendStop();
      };

      btn.addEventListener("touchstart", start, { passive: false });
      btn.addEventListener("touchend", end, { passive: false });
      btn.addEventListener("touchcancel", end, { passive: false });
      btn.addEventListener("mousedown", start);
      btn.addEventListener("mouseup", end);
      btn.addEventListener("mouseleave", end);
    });
  }

  function applyMode(mode) {
    state.mode = mode === "dpad" ? "dpad" : "joystick";
    localStorage.setItem("phybotMode", state.mode);

    const isJoy = state.mode === "joystick";
    els.joystickArea.style.display = isJoy ? "grid" : "none";
    els.dpadArea.style.display = isJoy ? "none" : "grid";
    els.modeToggle.textContent = `Modo: ${isJoy ? "Joystick" : "D-pad"}`;

    if (!isJoy) {
      state.joystickActive = false;
      state.joyNormX = 0;
      state.joyNormY = 0;
      drawJoystick();
      sendStop();
    } else if (state.dpadHold) {
      state.dpadHold = null;
      sendStop();
    }
  }

  function connectWebSocket() {
    const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const wsUrl = `${wsProtocol}//${window.location.host}/ws`;

    setConn(false, "Conectando...");
    clearTimeout(state.reconnectTimer);

    try {
      state.ws = new WebSocket(wsUrl);
    } catch (err) {
      scheduleReconnect();
      return;
    }

    state.ws.onopen = () => {
      state.reconnectDelay = 1000;
      setConn(true, "Conectado");
    };

    state.ws.onmessage = (event) => {
      let data = null;
      try {
        data = JSON.parse(event.data);
      } catch (_e) {
        return;
      }

      if (data?.type === "hello" || data?.type === "sensors") {
        updateSensors(data);
      } else if (data?.type === "nav_status") {
        updateNavStatus(data);
      }
    };

    state.ws.onerror = () => {
      setConn(false, "Erro de conexão");
    };

    state.ws.onclose = () => {
      setConn(false, "Desconectado");
      scheduleReconnect();
    };
  }

  function scheduleReconnect() {
    clearTimeout(state.reconnectTimer);
    state.reconnectTimer = setTimeout(connectWebSocket, state.reconnectDelay);
    state.reconnectDelay = Math.min(10000, state.reconnectDelay * 2);
  }

  function setConn(ok, text) {
    state.isConnected = ok;
    els.connText.textContent = text;
    els.connDot.classList.toggle("connected", ok);
  }

  function sendJson(obj, force) {
    const now = Date.now();
    if (!force && now - state.lastSendAt < state.sendIntervalMs) return;
    state.lastSendAt = now;

    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;
    state.ws.send(JSON.stringify(obj));
  }

  function sendMotor(left, right, force) {
    if (state.navActive) return;
    state.leftCmd = clamp(Math.round(left), -255, 255);
    state.rightCmd = clamp(Math.round(right), -255, 255);
    sendJson(
      {
        type: "motor",
        left: state.leftCmd,
        right: state.rightCmd,
      },
      force
    );
  }

  function sendStop() {
    state.leftCmd = 0;
    state.rightCmd = 0;
    sendJson({ type: "stop" }, true);
  }

  function emergencyStop() {
    sendStop();
    if (navigator.vibrate) navigator.vibrate(200);
  }

  function sendDpadDirection(dir) {
    const s = state.speed;
    switch (dir) {
      case "forward":
        sendMotor(s, s, true);
        break;
      case "backward":
        sendMotor(-s, -s, true);
        break;
      case "left":
        sendMotor(-s, s, true);
        break;
      case "right":
        sendMotor(s, -s, true);
        break;
      case "stop":
      default:
        sendStop();
        break;
    }
  }

  function updateSensors(data) {
    const connected = Array.isArray(data.connected) ? data.connected : [];
    ensureCards(connected);
    updateCards(data);
  }

  function renderEmptyCards() {
    const fallback = ["hcsr04", "vl53l0x", "mpu6050", "encoders", "battery"];
    ensureCards(fallback);
  }

  function ensureCards(connectedSensors) {
    const requiredDefs = [];
    connectedSensors.forEach((sensorId) => {
      const defs = CARD_DEFS[sensorId];
      if (defs) requiredDefs.push(...defs);
    });

    if (!requiredDefs.length) return;

    const requiredIds = new Set(requiredDefs.map((d) => d.id));
    Object.keys(state.cards).forEach((id) => {
      if (!requiredIds.has(id)) {
        state.cards[id].remove();
        delete state.cards[id];
      }
    });

    requiredDefs.forEach((def) => {
      if (state.cards[def.id]) return;
      const node = document.createElement("article");
      node.className = "sensor-card";
      node.innerHTML = `<div class="sensor-title">${def.title}</div><div class="sensor-value">--</div>`;
      state.cards[def.id] = node;
      els.sensorGrid.appendChild(node);
    });
  }

  function updateCards(data) {
    Object.keys(CARD_DEFS).forEach((sensorId) => {
      CARD_DEFS[sensorId].forEach((def) => {
        const node = state.cards[def.id];
        if (!node) return;
        const valueNode = node.querySelector(".sensor-value");
        valueNode.textContent = def.format(data);
      });
    });
  }

  function setupJoystick() {
    const canvas = els.joystickCanvas;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    let pointerId = null;

    const getPos = (clientX, clientY) => {
      const rect = canvas.getBoundingClientRect();
      const x = clientX - rect.left;
      const y = clientY - rect.top;
      const cx = rect.width / 2;
      const cy = rect.height / 2;
      const maxR = Math.min(rect.width, rect.height) * 0.36;

      const dx = x - cx;
      const dy = y - cy;
      const dist = Math.sqrt(dx * dx + dy * dy);

      const clamped = dist > maxR ? maxR / dist : 1;
      const nx = (dx * clamped) / maxR;
      const ny = (dy * clamped) / maxR;
      return { nx, ny };
    };

    const onPointerDown = (ev) => {
      if (state.mode !== "joystick") return;
      pointerId = ev.pointerId;
      canvas.setPointerCapture(pointerId);
      state.joystickActive = true;
      const p = getPos(ev.clientX, ev.clientY);
      state.joyNormX = p.nx;
      state.joyNormY = p.ny;
      drawJoystick();
      sendFromJoystick();
    };

    const onPointerMove = (ev) => {
      if (!state.joystickActive || ev.pointerId !== pointerId) return;
      const p = getPos(ev.clientX, ev.clientY);
      state.joyNormX = p.nx;
      state.joyNormY = p.ny;
      drawJoystick();
      sendFromJoystick();
    };

    const onPointerUp = (ev) => {
      if (ev.pointerId !== pointerId) return;
      state.joystickActive = false;
      pointerId = null;
      state.joyNormX = 0;
      state.joyNormY = 0;
      drawJoystick();
      sendStop();
    };

    canvas.addEventListener("pointerdown", onPointerDown);
    canvas.addEventListener("pointermove", onPointerMove);
    canvas.addEventListener("pointerup", onPointerUp);
    canvas.addEventListener("pointercancel", onPointerUp);
    drawJoystick();
  }

  function sendFromJoystick() {
    const deadZone = 0.15;
    const x = Math.abs(state.joyNormX) < deadZone ? 0 : state.joyNormX;
    const y = Math.abs(state.joyNormY) < deadZone ? 0 : state.joyNormY;

    // y negativo no canvas significa frente; invertido para comando.
    const throttle = -y;
    const turn = x;
    const leftMix = clamp(throttle + turn, -1, 1);
    const rightMix = clamp(throttle - turn, -1, 1);

    const left = leftMix * state.speed;
    const right = rightMix * state.speed;
    sendMotor(left, right, false);
  }

  function drawJoystick() {
    const canvas = els.joystickCanvas;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const maxR = Math.min(w, h) * 0.36;
    const knobR = 24;
    const kx = cx + state.joyNormX * maxR;
    const ky = cy + state.joyNormY * maxR;

    ctx.clearRect(0, 0, w, h);

    ctx.fillStyle = "rgba(255,255,255,0.06)";
    ctx.strokeStyle = "rgba(255,255,255,0.25)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, maxR, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.strokeStyle = "rgba(255,255,255,0.18)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx - maxR, cy);
    ctx.lineTo(cx + maxR, cy);
    ctx.moveTo(cx, cy - maxR);
    ctx.lineTo(cx, cy + maxR);
    ctx.stroke();

    ctx.fillStyle = state.joystickActive
      ? "rgba(103,209,255,0.85)"
      : "rgba(255,255,255,0.8)";
    ctx.beginPath();
    ctx.arc(kx, ky, knobR, 0, Math.PI * 2);
    ctx.fill();
  }

  function updateNavStatus(data) {
    state.navActive = data.active === true;

    if (state.navActive) {
      els.navBtn.textContent = "Cancelar";
      els.navBtn.classList.add("cancel");
      els.navInfo.innerHTML =
        `Seg <strong>${data.segment}/${data.total}</strong> — ${data.name} (${data.duration}s)`;
    } else {
      els.navBtn.textContent = "Navegar";
      els.navBtn.classList.remove("cancel");
      els.navInfo.innerHTML = data.name === "Cancelada"
        ? "Trajetória cancelada"
        : data.name === "Concluída"
          ? "Trajetória <strong>concluída</strong>"
          : "Trajetória pronta";
    }

    // bloqueia/desbloqueia controles manuais
    els.modeToggle.disabled = state.navActive;
    els.speedSlider.disabled = state.navActive;
  }

  function clamp(v, min, max) {
    return Math.max(min, Math.min(max, v));
  }

  function num(v, decimals) {
    if (typeof v !== "number" || Number.isNaN(v)) return "--";
    if (decimals <= 0) return String(Math.round(v));
    return String(v.toFixed(decimals));
  }
})();
