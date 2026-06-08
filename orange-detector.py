#!/usr/bin/env python3
"""
Rover Web Server — Raspberry Pi 4 + L298N + HC-SR04 + USB Camera
Adds orange-cone detection and autonomous "seek & approach" mode via OpenCV.

Install deps:  pip3 install flask opencv-python --break-system-packages
               sudo apt install python3-opencv -y
Run:           python3 rover_server.py
"""

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from flask import Flask, request, jsonify, render_template_string, Response
from gpiozero import LED, PWMOutputDevice, DigitalOutputDevice
from time import sleep, time
import threading
import lgpio
import cv2
import numpy as np

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
OBSTACLE_DISTANCE  = 3.0   # cm — tightened: was 25 cm, rover was stopping way too early
AVOID_BACKUP_TIME  = 0.6
AVOID_TURN_TIME    = 0.45

# ── Orange-cone HSV tuning ─────────────────────────────────────────────────────
# Narrowed to true traffic-cone orange (hue 5-20).
# Removed the 160-180 wrap-around band — that was the main source of red confusion.
# Raised saturation floor to 180 and value floor to 120 so dim/dark reds are rejected.
# If your cone still isn't detected, lower ORANGE_LOWER1[1] toward 150.
ORANGE_LOWER1 = np.array([  3, 130, 110])
ORANGE_UPPER1 = np.array([ 25, 255, 255])
ORANGE_LOWER2 = np.array([  3, 130, 110])
ORANGE_UPPER2 = np.array([ 25, 255, 255])

MIN_CONE_AREA = 1500
FRAME_W         = 640
FRAME_H         = 480

# Cone-tracking PID-ish constants
CENTRE_DEADBAND    = 8      # px — if cone cx is within this of frame centre, go straight
APPROACH_STOP_AREA = 120000   # px² — was 22000; rover now drives until cone is large/close
SEARCH_TURN_SPEED  = 0.18
APPROACH_SPEED     = 0.55
STEER_CORRECTION   = 0.20   # speed delta applied to slow the inner wheel

# ── Device Setup ──────────────────────────────────────────────────────────────
led       = LED(STATUS_LED)
left_ena  = PWMOutputDevice(MOTOR_LEFT_ENA,  initial_value=0)
left_in1  = DigitalOutputDevice(MOTOR_LEFT_IN1,  initial_value=False)
left_in2  = DigitalOutputDevice(MOTOR_LEFT_IN2,  initial_value=False)
right_enb = PWMOutputDevice(MOTOR_RIGHT_ENB, initial_value=0)
right_in3 = DigitalOutputDevice(MOTOR_RIGHT_IN3, initial_value=False)
right_in4 = DigitalOutputDevice(MOTOR_RIGHT_IN4, initial_value=False)
trig      = DigitalOutputDevice(ULTRASONIC_TRIG, initial_value=False)

_gpio_handle = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(_gpio_handle, ULTRASONIC_ECHO)

# ── Camera Setup ──────────────────────────────────────────────────────────────
_camera = cv2.VideoCapture(0)
_camera.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
_camera.set(cv2.CAP_PROP_FPS,          30)

_frame_lock    = threading.Lock()
_latest_frame  = None
_debug_frame   = None

_cone_lock   = threading.Lock()
_cone_data   = {
    'detected': False,
    'cx': 0,
    'cy': 0,
    'area': 0,
}

def detect_cone(frame):
    """
    Returns (detected, cx, cy, area, annotated_frame).
    Uses HSV colour thresholding to find the largest orange blob.
    """
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, ORANGE_LOWER1, ORANGE_UPPER1),
        cv2.inRange(hsv, ORANGE_LOWER2, ORANGE_UPPER2),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    annotated = frame.copy()
    cx_frame = FRAME_W // 2
    cv2.line(annotated, (cx_frame, 0), (cx_frame, FRAME_H), (50, 50, 50), 1)

    if not contours:
        return False, 0, 0, 0, annotated

    largest = max(contours, key=cv2.contourArea)
    area    = cv2.contourArea(largest)

    if area < MIN_CONE_AREA:
        return False, 0, 0, 0, annotated

    x, y, w, h = cv2.boundingRect(largest)
    cx = x + w // 2
    cy = y + h // 2
    box_area = w * h

    cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 140, 255), 2)
    cv2.circle(annotated, (cx, cy), 5, (0, 200, 255), -1)
    label = f"CONE  {box_area}px²"
    cv2.putText(annotated, label, (x, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 1, cv2.LINE_AA)

    return True, cx, cy, box_area, annotated


def camera_capture_loop():
    global _latest_frame, _debug_frame
    while True:
        ok, frame = _camera.read()
        if not ok:
            sleep(0.05)
            continue

        detected, cx, cy, area, annotated = detect_cone(frame)

        with _cone_lock:
            _cone_data['detected'] = detected
            _cone_data['cx']       = cx
            _cone_data['cy']       = cy
            _cone_data['area']     = area

        _, jpeg_raw = cv2.imencode('.jpg', frame,      [cv2.IMWRITE_JPEG_QUALITY, 70])
        _, jpeg_dbg = cv2.imencode('.jpg', annotated,  [cv2.IMWRITE_JPEG_QUALITY, 70])

        with _frame_lock:
            _latest_frame = jpeg_raw.tobytes()
            _debug_frame  = jpeg_dbg.tobytes()


cam_thread = threading.Thread(target=camera_capture_loop, daemon=True)
cam_thread.start()


def _get_latest(which='normal'):
    with _frame_lock:
        return _debug_frame if which == 'debug' else _latest_frame


def generate_mjpeg(which='normal'):
    while True:
        frame = _get_latest(which)
        if frame:
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
            )
        sleep(0.033)

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
    _camera.release()
    lgpio.gpiochip_close(_gpio_handle)
    for device in (left_ena, left_in1, left_in2,
                   right_enb, right_in3, right_in4, led, trig):
        device.close()

# ── Ultrasonic Sensor ─────────────────────────────────────────────────────────
_sonar_lock = threading.Lock()


def get_distance_cm() -> float:
    with _sonar_lock:
        trig.off(); sleep(0.000002)
        trig.on();  sleep(0.00001)
        trig.off()

        timeout = time() + 0.04
        while lgpio.gpio_read(_gpio_handle, ULTRASONIC_ECHO) == 0:
            if time() > timeout: return 999.0
        pulse_start = time()

        timeout = time() + 0.04
        while lgpio.gpio_read(_gpio_handle, ULTRASONIC_ECHO) == 1:
            if time() > timeout: return 999.0
        pulse_end = time()

    return round(((pulse_end - pulse_start) * 34300) / 2, 1)

# ── Avoidance (manual-mode only) ──────────────────────────────────────────────
avoidance_active = False
avoidance_lock   = threading.Lock()
latest_distance  = 999.0


def run_avoidance_sequence():
    global avoidance_active, current_command
    try:
        led.on(); stop(); sleep(0.4)
        set_motors(-DEFAULT_SPEED, -DEFAULT_SPEED); sleep(AVOID_BACKUP_TIME)
        set_motors( DEFAULT_SPEED, -DEFAULT_SPEED); sleep(AVOID_TURN_TIME)
        stop(); current_command = 'stop'
    finally:
        with avoidance_lock:
            avoidance_active = False

# ── Autonomous cone-seek state machine ───────────────────────────────────────
_auto_lock       = threading.Lock()
_auto_state      = 'idle'
_auto_thread     = None
_auto_stop_event = threading.Event()


def autonomous_loop():
    global _auto_state, latest_distance
    _auto_stop_event.clear()

    SEARCH_SPIN_INTERVAL = 0.3
    LOST_TIMEOUT         = 0.6

    last_seen = 0.0

    while not _auto_stop_event.is_set():
        with _cone_lock:
            detected = _cone_data['detected']
            cx       = _cone_data['cx']
            area     = _cone_data['area']

        dist = latest_distance

        if _auto_state in ('locking', 'approaching'):
            if dist < OBSTACLE_DISTANCE or area >= APPROACH_STOP_AREA:
                stop()
                with _auto_lock:
                    _auto_state = 'arrived'
                led.on()
                for _ in range(3):
                    led.off(); sleep(0.2)
                    led.on();  sleep(0.2)
                led.off()
                _auto_stop_event.set()
                break

        if detected:
            last_seen = time()
            error = cx - (FRAME_W // 2)

            if abs(error) <= CENTRE_DEADBAND:
                with _auto_lock:
                    _auto_state = 'approaching'
                set_motors(APPROACH_SPEED, APPROACH_SPEED)
            else:
                with _auto_lock:
                    _auto_state = 'locking'
                correction = STEER_CORRECTION * (error / (FRAME_W / 2))
                left_spd  = APPROACH_SPEED + correction
                right_spd = APPROACH_SPEED - correction
                left_spd  = max(-1.0, min(1.0, left_spd))
                right_spd = max(-1.0, min(1.0, right_spd))
                set_motors(left_spd, right_spd)

        else:
            if (time() - last_seen) > LOST_TIMEOUT or last_seen == 0.0:
                with _auto_lock:
                    _auto_state = 'searching'
                set_motors(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED)
                sleep(SEARCH_SPIN_INTERVAL)
                stop()
                sleep(0.4)

        sleep(0.05)

    stop()
    with _auto_lock:
        if _auto_state not in ('arrived',):
            _auto_state = 'idle'


def start_autonomous():
    global _auto_thread, _auto_state
    _auto_stop_event.set()
    if _auto_thread and _auto_thread.is_alive():
        _auto_thread.join(timeout=1.0)
    with _auto_lock:
        _auto_state = 'searching'
    _auto_thread = threading.Thread(target=autonomous_loop, daemon=True)
    _auto_thread.start()


def stop_autonomous():
    global _auto_state
    _auto_stop_event.set()
    stop()
    with _auto_lock:
        _auto_state = 'idle'


def is_autonomous() -> bool:
    with _auto_lock:
        return _auto_state != 'idle'

# ── Watchdog + Sensor Loop ────────────────────────────────────────────────────
COMMAND_TIMEOUT   = 0.3
last_command_time = time()
current_command   = 'stop'


def watchdog_and_sensor():
    global current_command, latest_distance, avoidance_active
    while True:
        dist = get_distance_cm()
        latest_distance = dist

        if is_autonomous():
            sleep(0.1)
            continue

        with avoidance_lock:
            already_avoiding = avoidance_active

        if (not already_avoiding
                and current_command == 'forward'
                and dist < OBSTACLE_DISTANCE):
            with avoidance_lock:
                avoidance_active = True
            threading.Thread(target=run_avoidance_sequence, daemon=True).start()

        if (not already_avoiding
                and current_command != 'stop'
                and (time() - last_command_time) > COMMAND_TIMEOUT):
            stop()
            current_command = 'stop'

        sleep(0.1)


threading.Thread(target=watchdog_and_sensor, daemon=True).start()

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


@app.route('/stream')
def stream():
    return Response(generate_mjpeg('normal'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream/debug')
def stream_debug():
    return Response(generate_mjpeg('debug'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/command', methods=['POST'])
def command():
    global last_command_time, current_command

    if is_autonomous():
        stop_autonomous()

    with avoidance_lock:
        if avoidance_active:
            return jsonify({'status': 'busy', 'message': 'avoidance running'}), 423

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


@app.route('/auto/start', methods=['POST'])
def auto_start():
    global current_command
    current_command = 'stop'
    start_autonomous()
    return jsonify({'status': 'ok', 'mode': 'autonomous'})


@app.route('/auto/stop', methods=['POST'])
def auto_stop():
    stop_autonomous()
    return jsonify({'status': 'ok', 'mode': 'manual'})


@app.route('/status')
def status():
    with avoidance_lock:
        avoiding = avoidance_active
    with _auto_lock:
        auto_state = _auto_state
    with _cone_lock:
        cone = dict(_cone_data)

    return jsonify({
        'command':          current_command,
        'speed':            DEFAULT_SPEED,
        'distance_cm':      latest_distance,
        'avoidance_active': avoiding,
        'auto_state':       auto_state,
        'cone':             cone,
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
    --auto-bg:   #ff7700;
    --auto-fg:   #ffffff;
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
    gap: 24px;
    padding: 28px 16px;
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

  /* ── Camera ── */
  .camera-wrap {
    width: 100%; max-width: 640px;
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .stream-toggle {
    display: flex;
    gap: 6px;
    align-self: flex-end;
  }
  .stream-btn {
    font-family: 'DM Mono', monospace;
    font-size: 0.65rem;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    padding: 4px 10px;
    border: 1px solid var(--border);
    border-radius: 20px;
    background: var(--surface);
    color: var(--muted);
    cursor: pointer;
    transition: all 0.15s;
  }
  .stream-btn.active {
    background: var(--accent);
    color: var(--active-fg);
    border-color: var(--accent);
  }
  .camera-card {
    background: #0d0d0c;
    border: 1px solid #2a2927;
    border-radius: var(--radius);
    width: 100%;
    overflow: hidden;
    position: relative;
    aspect-ratio: 4/3;
  }
  .camera-card img { width:100%; height:100%; object-fit:cover; display:block; }
  .cam-badge {
    position: absolute; top:10px; left:12px;
    display:flex; align-items:center; gap:6px;
    background: rgba(0,0,0,.55); backdrop-filter:blur(6px);
    border-radius:20px; padding:4px 10px;
    font-family:'DM Mono',monospace; font-size:.65rem;
    color:#e0ddd4; letter-spacing:.08em; text-transform:uppercase;
  }
  .cam-dot { width:6px;height:6px;border-radius:50%;background:#e74c3c;animation:pulse 1.4s infinite; }
  @keyframes pulse { 0%,100%{opacity:1}50%{opacity:.3} }
  .cam-dist {
    position:absolute; top:10px; right:12px;
    background:rgba(0,0,0,.55); backdrop-filter:blur(6px);
    border-radius:20px; padding:4px 12px;
    font-family:'DM Mono',monospace; font-size:.72rem;
    color:#e0ddd4; letter-spacing:.05em; transition:color .2s;
  }
  .cam-dist.near { color:#e74c3c; }
  .cam-cone-state {
    position:absolute; bottom:10px; left:12px;
    background:rgba(255,119,0,.85); backdrop-filter:blur(6px);
    border-radius:20px; padding:4px 12px;
    font-family:'DM Mono',monospace; font-size:.68rem;
    color:#fff; letter-spacing:.06em; text-transform:uppercase;
    display:none;
  }
  .cam-cone-state.visible { display:block; }

  /* ── Autonomous mode banner ──
     Uses min-height + visibility instead of display:none/flex so
     the banner always occupies space — no more layout jump when it
     appears or the text changes.                                    */
  .auto-banner {
    width:100%; max-width:640px;
    display: flex;                      /* always in flow */
    align-items:center; justify-content:space-between;
    gap:12px;
    padding:12px 18px;
    background:var(--auto-bg);
    border-radius:var(--radius);
    font-family:'DM Mono',monospace;
    font-size:.78rem;
    color:#fff;
    letter-spacing:.05em;
    /* hidden by default via visibility + zero height */
    visibility: hidden;
    min-height: 0;
    padding-top: 0;
    padding-bottom: 0;
    overflow: hidden;
    transition: min-height 0.2s ease, padding 0.2s ease;
  }
  .auto-banner.visible {
    visibility: visible;
    min-height: 48px;
    padding-top: 12px;
    padding-bottom: 12px;
  }
  .auto-banner-text { display:flex;align-items:center;gap:10px; }
  .auto-spinner {
    width:12px;height:12px;border-radius:50%;
    border:2px solid rgba(255,255,255,.35);
    border-top-color:#fff;
    animation:spin .8s linear infinite;
    flex-shrink:0;
  }
  @keyframes spin{to{transform:rotate(360deg)}}
  .btn-abort {
    font-family:'DM Mono',monospace;
    font-size:.7rem;
    letter-spacing:.06em;
    text-transform:uppercase;
    padding:5px 14px;
    border:1px solid rgba(255,255,255,.5);
    border-radius:20px;
    background:transparent;
    color:#fff;
    cursor:pointer;
    transition:background .15s;
    white-space:nowrap;
  }
  .btn-abort:hover { background:rgba(255,255,255,.15); }

  /* ── Cards ── */
  .card {
    background:var(--surface);
    border:1px solid var(--border);
    border-radius:var(--radius);
    padding:24px;
    width:100%; max-width:400px;
  }
  .section-label {
    font-family:'DM Mono',monospace;
    font-size:.68rem; letter-spacing:.1em;
    text-transform:uppercase; color:var(--muted);
    margin-bottom:14px;
  }

  /* ── Autonomous trigger button ── */
  .btn-auto {
    width:100%;
    display:flex; align-items:center; justify-content:center; gap:10px;
    padding:14px;
    background:var(--auto-bg);
    color:#fff;
    border:none;
    border-radius:var(--radius);
    font-family:'DM Mono',monospace;
    font-size:.85rem; letter-spacing:.08em; text-transform:uppercase;
    cursor:pointer;
    transition:opacity .15s, transform .08s;
    margin-bottom:20px;
  }
  .btn-auto:hover  { opacity:.9; }
  .btn-auto:active { transform:scale(0.97); }
  .btn-auto:disabled { opacity:.4; cursor:not-allowed; }
  .cone-icon { font-size:1.1rem; }

  /* ── D-pad ── */
  .dpad {
    display:grid;
    grid-template-columns:repeat(3,1fr);
    grid-template-rows:repeat(3,1fr);
    gap:8px; aspect-ratio:1;
    max-width:220px; margin:0 auto;
    transition:opacity .2s;
  }
  .dpad.locked { opacity:.3; pointer-events:none; }
  .btn {
    display:flex; align-items:center; justify-content:center;
    background:var(--bg); border:1px solid var(--border);
    border-radius:var(--radius); cursor:pointer;
    font-family:'DM Mono',monospace; font-size:.75rem;
    color:var(--text); user-select:none;
    transition:background .1s,color .1s,border-color .1s,transform .08s;
    -webkit-tap-highlight-color:transparent; touch-action:none;
  }
  .btn svg { width:18px;height:18px;stroke:currentColor;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round; }
  .btn:active,.btn.pressed { background:var(--active-bg);color:var(--active-fg);border-color:var(--active-bg);transform:scale(0.96); }
  .btn.empty { background:transparent;border:none;pointer-events:none; }

  /* ── Speed ── */
  .speed-section { margin-top:22px; }
  .speed-row { display:flex;align-items:center;gap:12px;margin-top:10px; }
  input[type=range] {
    flex:1;height:4px;-webkit-appearance:none;
    background:var(--border);border-radius:2px;outline:none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance:none;width:18px;height:18px;border-radius:50%;
    background:var(--accent);cursor:pointer;
    border:3px solid var(--surface);box-shadow:0 0 0 1px var(--border);
  }
  .speed-value { font-family:'DM Mono',monospace;font-size:.8rem;min-width:36px;text-align:right; }

  /* ── Status ── */
  .status-bar {
    display:flex;align-items:center;gap:10px;
    margin-top:22px;padding-top:18px;
    border-top:1px solid var(--border);
  }
  .status-dot { width:7px;height:7px;border-radius:50%;background:var(--border);transition:background .2s;flex-shrink:0; }
  .status-dot.active  { background:#3d9970; }
  .status-dot.warning { background:var(--warn); }
  .status-dot.auto    { background:var(--auto-bg); }
  .status-text { font-family:'DM Mono',monospace;font-size:.72rem;color:var(--muted);letter-spacing:.05em;text-transform:uppercase;flex:1; }

  .avoid-banner {
    display:none;align-items:center;justify-content:center;gap:8px;
    padding:10px 14px;margin-top:12px;border-radius:7px;
    background:var(--warn-bg);border:1px solid #e8b4b0;
    font-family:'DM Mono',monospace;font-size:.72rem;color:var(--warn);
    letter-spacing:.05em;text-transform:uppercase;
  }
  .avoid-banner.visible { display:flex; }

  .hint {
    font-family:'DM Mono',monospace;font-size:.68rem;
    color:var(--muted);text-align:center;letter-spacing:.04em;
  }
  kbd {
    display:inline-block;padding:1px 5px;
    border:1px solid var(--border);border-radius:4px;
    font-family:'DM Mono',monospace;font-size:.68rem;
    background:var(--surface);color:var(--text);
  }
</style>
</head>
<body>

<header>
  <h1>Rover Control</h1>
  <p id="ip-display">loading...</p>
</header>

<!-- ── Autonomous banner ── -->
<div class="auto-banner" id="auto-banner">
  <div class="auto-banner-text">
    <div class="auto-spinner"></div>
    <span id="auto-state-label">searching for cone…</span>
  </div>
  <button class="btn-abort" onclick="abortAuto()">Abort</button>
</div>

<!-- ── Camera ── -->
<div class="camera-wrap">
  <div class="stream-toggle">
    <button class="stream-btn active" id="btn-raw"   onclick="setStream('normal')">Normal</button>
    <button class="stream-btn"        id="btn-debug" onclick="setStream('debug')">Detection</button>
  </div>
  <div class="camera-card">
    <img id="cam-img" src="/stream" alt="Camera feed">
    <div class="cam-badge"><span class="cam-dot"></span>Live</div>
    <div class="cam-dist" id="cam-dist">— cm</div>
    <div class="cam-cone-state" id="cam-cone-state">🟠 cone locked</div>
  </div>
</div>

<!-- ── Controls ── -->
<div class="card">
  <div class="section-label">Autonomous</div>
  <button class="btn-auto" id="btn-auto-seek" onclick="startAuto()">
    <span class="cone-icon">🔶</span> Seek Orange Cone
  </button>

  <div class="section-label">Manual</div>
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

  <div class="avoid-banner" id="avoid-banner">⚠ obstacle — avoiding…</div>

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
  keyboard: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> &nbsp;·&nbsp; <kbd>space</kbd> stop &nbsp;·&nbsp; <kbd>T</kbd> seek cone
</div>

<script>
  let currentCmd   = 'stop';
  let sendInterval = null;
  let speed        = 0.75;
  let isLocked     = false;
  let autoMode     = false;

  const KEY_MAP = { w:'forward', s:'backward', a:'left', d:'right', ' ':'stop' };

  document.getElementById('ip-display').textContent = window.location.host;

  const slider     = document.getElementById('speed-slider');
  const speedLabel = document.getElementById('speed-label');
  slider.addEventListener('input', () => {
    speed = slider.value / 100;
    speedLabel.textContent = slider.value + '%';
  });

  // ── Stream toggle ─────────────────────────────────────────────────────────
  function setStream(mode) {
    document.getElementById('cam-img').src = mode === 'debug' ? '/stream/debug' : '/stream';
    document.getElementById('btn-raw').classList.toggle('active',   mode === 'normal');
    document.getElementById('btn-debug').classList.toggle('active', mode === 'debug');
  }

  // ── Command helpers ───────────────────────────────────────────────────────
  async function sendCommand(cmd) {
    if (isLocked && cmd !== 'stop') return;
    try {
      const res  = await fetch('/command', {
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({ command: cmd, speed }),
      });
      const data = await res.json();
      if (data.status === 'busy') setLocked(true);
    } catch(e) {}
  }

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

  // ── Autonomous ─────────────────────────────────────────────────────────────
  async function startAuto() {
    try {
      await fetch('/auto/start', { method:'POST' });
      setAutoMode(true);
    } catch(e) {}
  }

  async function abortAuto() {
    try {
      await fetch('/auto/stop', { method:'POST' });
      setAutoMode(false);
    } catch(e) {}
  }

  // setAutoMode only touches the DOM when the value actually changes,
  // preventing the banner from re-rendering (and jumping) on every poll tick.
  function setAutoMode(val) {
    if (autoMode === val) return;   // ← no-op if unchanged — fixes the flashing
    autoMode = val;
    document.getElementById('auto-banner').classList.toggle('visible', val);
    document.getElementById('dpad').classList.toggle('locked', val);
    document.getElementById('btn-auto-seek').disabled = val;
    if (val) {
      stopCommand(false);
      isLocked = false;
    }
  }

  const AUTO_LABELS = {
    searching:   '🔍 searching for cone…',
    locking:     '🟠 cone spotted — aligning…',
    approaching: '➡ approaching cone…',
    arrived:     '✅ arrived at cone!',
    idle:        'autonomous off',
  };

  // ── Status polling ────────────────────────────────────────────────────────
  const dpad        = document.getElementById('dpad');
  const avoidBanner = document.getElementById('avoid-banner');
  const statusDot   = document.getElementById('status-dot');
  const statusText  = document.getElementById('status-text');
  const camDist     = document.getElementById('cam-dist');
  const camConeState= document.getElementById('cam-cone-state');
  const autoLabel   = document.getElementById('auto-state-label');

  async function pollStatus() {
    try {
      const res  = await fetch('/status');
      const data = await res.json();

      const dist   = data.distance_cm;
      const isNear = dist < 15 && dist < 999;
      camDist.textContent = dist >= 999 ? '— cm' : dist.toFixed(1) + ' cm';
      camDist.classList.toggle('near', isNear);

      const coneDetected = data.cone && data.cone.detected;
      camConeState.classList.toggle('visible', coneDetected);

      const aState = data.auto_state || 'idle';
      const inAuto  = aState !== 'idle';

      // Only call setAutoMode when the mode actually flips
      setAutoMode(inAuto);

      if (inAuto) {
        // Update label text without touching layout
        const newLabel = AUTO_LABELS[aState] || aState;
        if (autoLabel.textContent !== newLabel) autoLabel.textContent = newLabel;

        statusDot.className   = 'status-dot auto';
        statusText.textContent = aState;

        if (aState === 'arrived') {
          setTimeout(() => { setAutoMode(false); abortAuto(); }, 1500);
        }
        return;
      }

      const avoiding = data.avoidance_active;
      if (avoiding !== isLocked) setLocked(avoiding);

      if (!avoiding) {
        const moving = data.command !== 'stop';
        statusDot.className    = 'status-dot' + (moving ? ' active' : '');
        statusText.textContent = data.command || 'stopped';
      } else {
        statusDot.className    = 'status-dot warning';
        statusText.textContent = 'avoiding obstacle';
      }
    } catch(e) {}
  }

  setInterval(pollStatus, 200);

  function setLocked(val) {
    isLocked = val;
    avoidBanner.classList.toggle('visible', val);
    if (val) {
      if (sendInterval) { clearInterval(sendInterval); sendInterval = null; }
      currentCmd = 'stop'; updateUI('stop');
    }
  }

  const LABELS = {
    forward:'forward', backward:'backward',
    left:'spinning left', right:'spinning right',
    stop:'stopped',
  };

  function updateUI(cmd) {
    document.querySelectorAll('.btn[data-cmd]').forEach(b => {
      b.classList.toggle('pressed', b.dataset.cmd === cmd && cmd !== 'stop');
    });
    if (!isLocked && !autoMode) {
      const moving = cmd !== 'stop';
      statusDot.className    = 'status-dot' + (moving ? ' active' : '');
      statusText.textContent = LABELS[cmd] || cmd;
    }
  }

  // ── D-pad touch/mouse ─────────────────────────────────────────────────────
  document.querySelectorAll('.btn[data-cmd]').forEach(btn => {
    const cmd     = btn.dataset.cmd;
    const press   = (e) => { e.preventDefault(); if (autoMode) return; cmd === 'stop' ? stopCommand() : startCommand(cmd); };
    const release = (e) => { e.preventDefault(); if (autoMode) return; if (cmd !== 'stop') stopCommand(); };
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('touchstart', press,   { passive:false });
    btn.addEventListener('mouseup',    release);
    btn.addEventListener('mouseleave', release);
    btn.addEventListener('touchend',   release, { passive:false });
  });

  // ── Keyboard ──────────────────────────────────────────────────────────────
  const pressedKeys = new Set();
  document.addEventListener('keydown', (e) => {
    if (e.key.toLowerCase() === 't') { startAuto(); return; }
    const cmd = KEY_MAP[e.key.toLowerCase()];
    if (!cmd || pressedKeys.has(e.key.toLowerCase()) || autoMode) return;
    e.preventDefault();
    pressedKeys.add(e.key.toLowerCase());
    cmd === 'stop' ? stopCommand() : startCommand(cmd);
  });
  document.addEventListener('keyup', (e) => {
    const cmd = KEY_MAP[e.key.toLowerCase()];
    if (!cmd) return;
    pressedKeys.delete(e.key.toLowerCase());
    if (cmd !== 'stop' && !autoMode) stopCommand();
  });
</script>
</body>
</html>"""

# ── Entry Point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    if not _camera.isOpened():
        print("WARNING: Could not open camera on /dev/video0.")
        print("         Try VideoCapture(1) or VideoCapture(2).")

    print("=" * 44)
    print("  ROVER WEB SERVER + CONE TRACKING")
    print("=" * 44)
    print(f"  Open: http://{local_ip}:5000")
    print(f"  Obstacle / arrive threshold: {OBSTACLE_DISTANCE} cm")
    print(f"  Cone area 'arrived' threshold: {APPROACH_STOP_AREA} px²")
    print("=" * 44)
    print("  Ctrl+C to stop\n")

    for _ in range(3):
        led.on();  sleep(0.2)
        led.off(); sleep(0.2)

    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        cleanup()
        print("\nServer stopped. GPIO released.")
