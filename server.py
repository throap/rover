#!/usr/bin/env python3
"""
Rover Web Server - Raspberry Pi 4 + L298N H-Bridge + HC-SR04 Ultrasonic Sensor
Hosts a web control panel on the Pi's local network.
Access from any browser at http://<PI_IP>:5000

Install deps:  pip3 install flask
Run:           python3 rover_server.py

Ultrasonic Sensor Wiring (HC-SR04):
  VCC  → 5V  (Pin 2 or 4)
  GND  → GND (any ground)
  TRIG → GPIO 23 (Pin 16)
  ECHO → GPIO 24 (Pin 18) via voltage divider:
           ECHO → 1kΩ → GPIO24 → 2kΩ → GND
"""

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from flask import Flask, request, jsonify, render_template_string
from gpiozero import LED, PWMOutputDevice, DigitalOutputDevice
from time import sleep, time
import threading

# ── Pin Definitions (BCM) ─────────────────────────────────────────────────────
STATUS_LED        = 25
MOTOR_LEFT_ENA    = 12
MOTOR_LEFT_IN1    = 27
MOTOR_LEFT_IN2    = 17
MOTOR_RIGHT_ENB   = 19
MOTOR_RIGHT_IN3   = 6
MOTOR_RIGHT_IN4   = 5
ULTRASONIC_TRIG   = 23
ULTRASONIC_ECHO   = 24

DEFAULT_SPEED      = 0.75
OBSTACLE_DISTANCE  = 25.0   # cm — stop and avoid if closer than this
AVOID_BACKUP_TIME  = 0.6    # seconds to back up
AVOID_TURN_TIME    = 0.45   # seconds to turn right after backing up

# ── Device Setup ──────────────────────────────────────────────────────────────
led       = LED(STATUS_LED)
left_ena  = PWMOutputDevice(MOTOR_LEFT_ENA,  initial_value=0)
left_in1  = DigitalOutputDevice(MOTOR_LEFT_IN1,  initial_value=False)
left_in2  = DigitalOutputDevice(MOTOR_LEFT_IN2,  initial_value=False)
right_enb = PWMOutputDevice(MOTOR_RIGHT_ENB, initial_value=0)
right_in3 = DigitalOutputDevice(MOTOR_RIGHT_IN3, initial_value=False)
right_in4 = DigitalOutputDevice(MOTOR_RIGHT_IN4, initial_value=False)
trig      = DigitalOutputDevice(ULTRASONIC_TRIG, initial_value=False)

# Echo uses gpiozero's Button-like input — we read it manually via lgpio
import lgpio
_gpio_handle = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(_gpio_handle, ULTRASONIC_ECHO)

# ── Motor Control ─────────────────────────────────────────────────────────────

def set_motors(left: float, right: float) -> None:
    left_in1.value  = left > 0
    left_in2.value  = left < 0
    left_ena.value  = abs(left)
    right_in3.value = right > 0
    right_in4.value = right < 0
    right_enb.value = abs(right)

def stop() -> None:
    set_motors(0.0, 0.0)
    led.off()

def cleanup() -> None:
    stop()
    lgpio.gpiochip_close(_gpio_handle)
    for device in (left_ena, left_in1, left_in2,
                   right_enb, right_in3, right_in4, led, trig):
        device.close()

# ── Ultrasonic Sensor ─────────────────────────────────────────────────────────
_sonar_lock = threading.Lock()

def get_distance_cm() -> float:
    """Returns distance in cm, or 999 on timeout/error."""
    with _sonar_lock:
        # Send 10µs trigger pulse
        trig.off()
        sleep(0.000002)
        trig.on()
        sleep(0.00001)
        trig.off()

        # Wait for echo HIGH (with timeout)
        timeout = time() + 0.04
        while lgpio.gpio_read(_gpio_handle, ULTRASONIC_ECHO) == 0:
            if time() > timeout:
                return 999.0
        pulse_start = time()

        # Wait for echo LOW (with timeout)
        timeout = time() + 0.04
        while lgpio.gpio_read(_gpio_handle, ULTRASONIC_ECHO) == 1:
            if time() > timeout:
                return 999.0
        pulse_end = time()

    duration = pulse_end - pulse_start
    distance = (duration * 34300) / 2  # speed of sound in cm/s
    return round(distance, 1)

# ── Avoidance State ───────────────────────────────────────────────────────────
avoidance_active = False       # True while the avoid sequence is running
avoidance_lock   = threading.Lock()
latest_distance  = 999.0      # updated by the sensor thread

def run_avoidance_sequence():
    """Backs up then turns right. Runs in its own thread."""
    global avoidance_active, current_command

    try:
        led.on()

        # 1. Stop briefly
        stop()
        sleep(0.15)

        # 2. Back up
        set_motors(-DEFAULT_SPEED, -DEFAULT_SPEED)
        sleep(AVOID_BACKUP_TIME)

        # 3. Turn right (left motor forward, right motor backward)
        set_motors(DEFAULT_SPEED, -DEFAULT_SPEED)
        sleep(AVOID_TURN_TIME)

        # 4. Full stop — hand control back to user
        stop()
        current_command = 'stop'
    finally:
        with avoidance_lock:
            avoidance_active = False

# ── Watchdog + Sensor Loop ────────────────────────────────────────────────────
COMMAND_TIMEOUT  = 0.3
last_command_time = time()
current_command  = 'stop'

def watchdog_and_sensor():
    """Combined thread: enforces command timeout and reads sonar."""
    global current_command, latest_distance, avoidance_active

    while True:
        # Read distance every ~100 ms
        dist = get_distance_cm()
        latest_distance = dist

        with avoidance_lock:
            already_avoiding = avoidance_active

        # Trigger avoidance if rover is moving forward and too close
        if (not already_avoiding
                and current_command == 'forward'
                and dist < OBSTACLE_DISTANCE):
            with avoidance_lock:
                avoidance_active = True
            t = threading.Thread(target=run_avoidance_sequence, daemon=True)
            t.start()

        # Watchdog: stop if client goes quiet
        if (not already_avoiding
                and current_command != 'stop'
                and (time() - last_command_time) > COMMAND_TIMEOUT):
            stop()
            current_command = 'stop'

        sleep(0.1)

wt = threading.Thread(target=watchdog_and_sensor, daemon=True)
wt.start()

# ── Flask App ─────────────────────────────────────────────────────────────────
app = Flask(__name__)

COMMANDS = {
    'forward':     ( DEFAULT_SPEED,  DEFAULT_SPEED),
    'backward':    (-DEFAULT_SPEED, -DEFAULT_SPEED),
    'left':        (-DEFAULT_SPEED,  DEFAULT_SPEED),
    'right':       ( DEFAULT_SPEED, -DEFAULT_SPEED),
    'pivot_left':  ( 0.0,            DEFAULT_SPEED),
    'pivot_right': ( DEFAULT_SPEED,  0.0          ),
    'stop':        ( 0.0,            0.0          ),
}

@app.route('/command', methods=['POST'])
def command():
    global last_command_time, current_command

    # Block all movement commands while avoidance is running
    with avoidance_lock:
        if avoidance_active:
            return jsonify({'status': 'busy', 'message': 'avoidance sequence running'}), 423

    data  = request.get_json()
    cmd   = data.get('command', 'stop')
    speed = float(data.get('speed', DEFAULT_SPEED))

    if cmd in COMMANDS:
        left, right = COMMANDS[cmd]
        if cmd != 'stop':
            left  = (left  / DEFAULT_SPEED) * speed
            right = (right / DEFAULT_SPEED) * speed
            led.on()
        else:
            led.off()

        set_motors(left, right)
        last_command_time = time()
        current_command   = cmd
        return jsonify({'status': 'ok', 'command': cmd})

    return jsonify({'status': 'error', 'message': 'unknown command'}), 400

@app.route('/status')
def status():
    with avoidance_lock:
        avoiding = avoidance_active
    return jsonify({
        'command':          current_command,
        'speed':            DEFAULT_SPEED,
        'distance_cm':      latest_distance,
        'avoidance_active': avoiding,
    })

@app.route('/')
def index():
    return render_template_string(HTML)

# ── HTML UI ───────────────────────────────────────────────────────────────────
HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Rover Control</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=DM+Mono:wght@300;400;500&family=DM+Sans:wght@300;400;500&display=swap');

  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg:        #f5f4f0;
    --surface:   #ffffff;
    --border:    #e2e0d8;
    --text:      #1a1916;
    --muted:     #9e9b92;
    --accent:    #1a1916;
    --active-bg: #1a1916;
    --active-fg: #f5f4f0;
    --warn:      #c0392b;
    --warn-bg:   #fdf0ef;
    --radius:    10px;
  }

  body {
    font-family: 'DM Sans', sans-serif;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 32px;
    padding: 40px 20px;
  }

  header { text-align: center; }

  header h1 {
    font-family: 'DM Mono', monospace;
    font-weight: 400;
    font-size: 1.1rem;
    letter-spacing: 0.12em;
    text-transform: uppercase;
  }

  header p {
    font-size: 0.78rem;
    color: var(--muted);
    margin-top: 6px;
    font-family: 'DM Mono', monospace;
    letter-spacing: 0.04em;
  }

  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 28px;
    width: 100%;
    max-width: 400px;
  }

  .section-label {
    font-family: 'DM Mono', monospace;
    font-size: 0.68rem;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--muted);
    margin-bottom: 16px;
  }

  /* D-pad grid */
  .dpad {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    grid-template-rows: repeat(3, 1fr);
    gap: 8px;
    aspect-ratio: 1;
    max-width: 240px;
    margin: 0 auto;
    transition: opacity 0.2s;
  }

  .dpad.locked { opacity: 0.35; pointer-events: none; }

  .btn {
    display: flex;
    align-items: center;
    justify-content: center;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    cursor: pointer;
    font-family: 'DM Mono', monospace;
    font-size: 0.75rem;
    color: var(--text);
    letter-spacing: 0.05em;
    user-select: none;
    transition: background 0.1s, color 0.1s, border-color 0.1s, transform 0.08s;
    -webkit-tap-highlight-color: transparent;
    touch-action: none;
  }

  .btn svg {
    width: 18px; height: 18px;
    stroke: currentColor; fill: none;
    stroke-width: 1.8;
    stroke-linecap: round; stroke-linejoin: round;
  }

  .btn:active, .btn.pressed {
    background: var(--active-bg);
    color: var(--active-fg);
    border-color: var(--active-bg);
    transform: scale(0.96);
  }

  .btn.empty {
    background: transparent;
    border: none;
    pointer-events: none;
  }

  /* Speed slider */
  .speed-section { margin-top: 24px; }

  .speed-row {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-top: 10px;
  }

  input[type=range] {
    flex: 1;
    height: 4px;
    -webkit-appearance: none;
    background: var(--border);
    border-radius: 2px;
    outline: none;
  }

  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 18px; height: 18px;
    border-radius: 50%;
    background: var(--accent);
    cursor: pointer;
    border: 3px solid var(--surface);
    box-shadow: 0 0 0 1px var(--border);
  }

  .speed-value {
    font-family: 'DM Mono', monospace;
    font-size: 0.8rem;
    min-width: 36px;
    text-align: right;
  }

  /* Status bar */
  .status-bar {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-top: 24px;
    padding-top: 20px;
    border-top: 1px solid var(--border);
  }

  .status-dot {
    width: 7px; height: 7px;
    border-radius: 50%;
    background: var(--border);
    transition: background 0.2s;
    flex-shrink: 0;
  }

  .status-dot.active  { background: #3d9970; }
  .status-dot.warning { background: var(--warn); }

  .status-text {
    font-family: 'DM Mono', monospace;
    font-size: 0.72rem;
    color: var(--muted);
    letter-spacing: 0.05em;
    text-transform: uppercase;
    flex: 1;
  }

  /* Sensor readout */
  .sensor-row {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-top: 14px;
    padding: 10px 12px;
    border-radius: 7px;
    background: var(--bg);
    border: 1px solid var(--border);
    transition: background 0.2s, border-color 0.2s;
  }

  .sensor-row.near {
    background: var(--warn-bg);
    border-color: #e8b4b0;
  }

  .sensor-icon {
    font-size: 0.9rem;
    flex-shrink: 0;
  }

  .sensor-label {
    font-family: 'DM Mono', monospace;
    font-size: 0.68rem;
    color: var(--muted);
    letter-spacing: 0.06em;
    text-transform: uppercase;
    flex: 1;
  }

  .sensor-value {
    font-family: 'DM Mono', monospace;
    font-size: 0.85rem;
    font-weight: 500;
    color: var(--text);
    letter-spacing: 0.02em;
    min-width: 52px;
    text-align: right;
  }

  .sensor-value.near { color: var(--warn); }

  /* Avoidance banner */
  .avoid-banner {
    display: none;
    align-items: center;
    justify-content: center;
    gap: 8px;
    padding: 10px 14px;
    margin-top: 14px;
    border-radius: 7px;
    background: var(--warn-bg);
    border: 1px solid #e8b4b0;
    font-family: 'DM Mono', monospace;
    font-size: 0.72rem;
    color: var(--warn);
    letter-spacing: 0.05em;
    text-transform: uppercase;
  }

  .avoid-banner.visible { display: flex; }

  /* Keyboard hint */
  .hint {
    font-family: 'DM Mono', monospace;
    font-size: 0.68rem;
    color: var(--muted);
    text-align: center;
    letter-spacing: 0.04em;
  }

  kbd {
    display: inline-block;
    padding: 1px 5px;
    border: 1px solid var(--border);
    border-radius: 4px;
    font-family: 'DM Mono', monospace;
    font-size: 0.68rem;
    background: var(--surface);
    color: var(--text);
  }
</style>
</head>
<body>

<header>
  <h1>Rover Control</h1>
  <p id="ip-display">loading...</p>
</header>

<div class="card">
  <div class="section-label">Direction</div>

  <div class="dpad" id="dpad">
    <div class="btn empty"></div>
    <div class="btn" id="btn-forward"  data-cmd="forward">
      <svg viewBox="0 0 24 24"><polyline points="18 15 12 9 6 15"/></svg>
    </div>
    <div class="btn empty"></div>

    <div class="btn" id="btn-left"     data-cmd="left">
      <svg viewBox="0 0 24 24"><polyline points="15 18 9 12 15 6"/></svg>
    </div>
    <div class="btn" id="btn-stop"     data-cmd="stop">
      <svg viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="2"/></svg>
    </div>
    <div class="btn" id="btn-right"    data-cmd="right">
      <svg viewBox="0 0 24 24"><polyline points="9 18 15 12 9 6"/></svg>
    </div>

    <div class="btn empty"></div>
    <div class="btn" id="btn-backward" data-cmd="backward">
      <svg viewBox="0 0 24 24"><polyline points="6 9 12 15 18 9"/></svg>
    </div>
    <div class="btn empty"></div>
  </div>

  <!-- Avoidance banner -->
  <div class="avoid-banner" id="avoid-banner">
    ⚠ obstacle — avoiding…
  </div>

  <!-- Sensor readout -->
  <div class="sensor-row" id="sensor-row">
    <span class="sensor-icon">◎</span>
    <span class="sensor-label">Distance</span>
    <span class="sensor-value" id="sensor-value">— cm</span>
  </div>

  <div class="speed-section">
    <div class="section-label">Speed</div>
    <div class="speed-row">
      <input type="range" id="speed-slider" min="10" max="100" value="75" step="5">
      <span class="speed-value" id="speed-label">75%</span>
    </div>
  </div>

  <div class="status-bar">
    <div class="status-dot" id="status-dot"></div>
    <span class="status-text" id="status-text">stopped</span>
  </div>
</div>

<div class="hint">
  keyboard: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> to move &nbsp;·&nbsp; <kbd>space</kbd> to stop
</div>

<script>
  let currentCmd  = 'stop';
  let sendInterval = null;
  let speed        = 0.75;
  let isLocked     = false;   // true while avoidance is running

  const KEY_MAP = { w:'forward', s:'backward', a:'left', d:'right', ' ':'stop' };

  document.getElementById('ip-display').textContent = window.location.host;

  // ── Speed slider ───────────────────────────────────────────────────────────
  const slider     = document.getElementById('speed-slider');
  const speedLabel = document.getElementById('speed-label');
  slider.addEventListener('input', () => {
    speed = slider.value / 100;
    speedLabel.textContent = slider.value + '%';
  });

  // ── Send command ───────────────────────────────────────────────────────────
  async function sendCommand(cmd) {
    if (isLocked && cmd !== 'stop') return;
    try {
      const res = await fetch('/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: cmd, speed }),
      });
      const data = await res.json();
      // Server says avoidance is running — lock UI immediately
      if (data.status === 'busy') setLocked(true);
    } catch(e) {}
  }

  // ── Repeat / stop helpers ──────────────────────────────────────────────────
  function startCommand(cmd) {
    if (isLocked || currentCmd === cmd) return;
    stopCommand(false);
    currentCmd = cmd;
    sendCommand(cmd);
    if (cmd !== 'stop') sendInterval = setInterval(() => sendCommand(cmd), 100);
    updateUI(cmd);
  }

  function stopCommand(send = true) {
    if (sendInterval) { clearInterval(sendInterval); sendInterval = null; }
    currentCmd = 'stop';
    if (send) sendCommand('stop');
    updateUI('stop');
  }

  // ── Lock / unlock controls ─────────────────────────────────────────────────
  const dpad        = document.getElementById('dpad');
  const avoidBanner = document.getElementById('avoid-banner');

  function setLocked(val) {
    isLocked = val;
    dpad.classList.toggle('locked', val);
    avoidBanner.classList.toggle('visible', val);
    if (val) {
      // Clear any ongoing movement command locally
      if (sendInterval) { clearInterval(sendInterval); sendInterval = null; }
      currentCmd = 'stop';
      updateUI('stop');
    }
  }

  // ── Status polling (includes distance + avoidance flag) ────────────────────
  const statusDot   = document.getElementById('status-dot');
  const statusText  = document.getElementById('status-text');
  const sensorRow   = document.getElementById('sensor-row');
  const sensorValue = document.getElementById('sensor-value');

  const LABELS = {
    forward:'forward', backward:'backward',
    left:'spinning left', right:'spinning right',
    pivot_left:'pivoting left', pivot_right:'pivoting right',
    stop:'stopped',
  };

  async function pollStatus() {
    try {
      const res  = await fetch('/status');
      const data = await res.json();

      // Avoidance state
      const avoiding = data.avoidance_active;
      setLocked(avoiding);

      // Distance display
      const dist     = data.distance_cm;
      const isNear   = dist < 25 && dist < 999;
      sensorValue.textContent = dist >= 999 ? '— cm' : dist.toFixed(1) + ' cm';
      sensorValue.classList.toggle('near', isNear);
      sensorRow.classList.toggle('near', isNear);

      // Status dot / text (only update if not avoiding)
      if (!avoiding) {
        const moving = data.command !== 'stop';
        statusDot.className  = 'status-dot' + (moving ? ' active' : '');
        statusText.textContent = LABELS[data.command] || data.command;
      } else {
        statusDot.className  = 'status-dot warning';
        statusText.textContent = 'avoiding obstacle';
      }
    } catch(e) {}
  }

  setInterval(pollStatus, 200);

  // ── UI button highlight ────────────────────────────────────────────────────
  function updateUI(cmd) {
    document.querySelectorAll('.btn[data-cmd]').forEach(b => {
      b.classList.toggle('pressed', b.dataset.cmd === cmd && cmd !== 'stop');
    });
    if (!isLocked) {
      const moving = cmd !== 'stop';
      statusDot.className   = 'status-dot' + (moving ? ' active' : '');
      statusText.textContent = LABELS[cmd] || cmd;
    }
  }

  // ── Button events ──────────────────────────────────────────────────────────
  document.querySelectorAll('.btn[data-cmd]').forEach(btn => {
    const cmd = btn.dataset.cmd;
    const press   = (e) => { e.preventDefault(); cmd === 'stop' ? stopCommand() : startCommand(cmd); };
    const release = (e) => { e.preventDefault(); if (cmd !== 'stop') stopCommand(); };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('touchstart', press,   { passive: false });
    btn.addEventListener('mouseup',    release);
    btn.addEventListener('mouseleave', release);
    btn.addEventListener('touchend',   release, { passive: false });
  });

  // ── Keyboard events ────────────────────────────────────────────────────────
  const pressedKeys = new Set();

  document.addEventListener('keydown', (e) => {
    const cmd = KEY_MAP[e.key.toLowerCase()];
    if (!cmd || pressedKeys.has(e.key.toLowerCase())) return;
    e.preventDefault();
    pressedKeys.add(e.key.toLowerCase());
    cmd === 'stop' ? stopCommand() : startCommand(cmd);
  });

  document.addEventListener('keyup', (e) => {
    const cmd = KEY_MAP[e.key.toLowerCase()];
    if (!cmd) return;
    pressedKeys.delete(e.key.toLowerCase());
    if (cmd !== 'stop') stopCommand();
  });
</script>
</body>
</html>"""

# ── Entry Point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    print("=" * 40)
    print("  ROVER WEB SERVER")
    print("=" * 40)
    print(f"  Open in browser:")
    print(f"  http://{local_ip}:5000")
    print(f"  Obstacle threshold: {OBSTACLE_DISTANCE} cm")
    print("=" * 40)
    print("  Ctrl+C to stop\n")

    for _ in range(3):
        led.on();  sleep(0.2)
        led.off(); sleep(0.2)

    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        cleanup()
        print("\nServer stopped. GPIO released.")
