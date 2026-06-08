#!/usr/bin/env python3
"""
Rover Dashboard - System diagnostics web server for Raspberry Pi rover
Shows IP, CPU temp, USB devices, CPU/RAM/disk usage, GPIO state, uptime, and more.

Run:  python3 rover_dashboard.py
Access at http://<PI_IP>:5001  (different port so it doesn't clash with rover_server.py)

No extra libraries needed beyond what the rover already uses.
"""

import os, subprocess, socket, re, json, threading, time
from flask import Flask, jsonify, render_template_string

app = Flask(__name__)

# ── Data collectors ────────────────────────────────────────────────────────────

def get_ip_addresses():
    ips = {}
    try:
        result = subprocess.check_output(['ip', '-o', 'addr', 'show'], text=True)
        for line in result.splitlines():
            parts = line.split()
            iface = parts[1]
            if 'inet ' in line and iface != 'lo':
                ip = parts[3].split('/')[0]
                ips[iface] = ip
    except Exception:
        pass
    # Also grab hostname
    try:
        ips['hostname'] = socket.gethostname()
    except Exception:
        pass
    return ips

def get_cpu_temp():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return round(int(f.read().strip()) / 1000, 1)
    except Exception:
        return None

def get_gpu_temp():
    try:
        out = subprocess.check_output(['vcgencmd', 'measure_temp'], text=True)
        match = re.search(r'[\d.]+', out)
        return float(match.group()) if match else None
    except Exception:
        return None

def get_cpu_usage():
    try:
        # Read /proc/stat twice 200ms apart for accurate reading
        def read_stat():
            with open('/proc/stat') as f:
                line = f.readline()
            vals = list(map(int, line.split()[1:]))
            idle = vals[3]
            total = sum(vals)
            return idle, total
        i1, t1 = read_stat()
        time.sleep(0.2)
        i2, t2 = read_stat()
        return round(100 * (1 - (i2 - i1) / (t2 - t1)), 1)
    except Exception:
        return None

def get_memory():
    try:
        with open('/proc/meminfo') as f:
            lines = f.readlines()
        mem = {}
        for line in lines:
            key, val = line.split(':')
            mem[key.strip()] = int(val.strip().split()[0])
        total = mem.get('MemTotal', 0)
        available = mem.get('MemAvailable', 0)
        used = total - available
        pct = round(100 * used / total, 1) if total else 0
        return {
            'total_mb':     round(total / 1024, 1),
            'used_mb':      round(used / 1024, 1),
            'available_mb': round(available / 1024, 1),
            'percent':      pct,
        }
    except Exception:
        return {}

def get_disk():
    try:
        result = subprocess.check_output(['df', '-h', '/'], text=True)
        parts = result.splitlines()[1].split()
        return {
            'total':   parts[1],
            'used':    parts[2],
            'free':    parts[3],
            'percent': parts[4],
        }
    except Exception:
        return {}

def get_usb_devices():
    devices = []
    try:
        result = subprocess.check_output(['lsusb'], text=True)
        for line in result.splitlines():
            # Format: Bus 001 Device 002: ID abcd:1234 Description
            match = re.match(r'Bus (\d+) Device (\d+): ID ([\w:]+)\s+(.*)', line)
            if match:
                bus, dev, uid, desc = match.groups()
                # Skip root hubs — they're internal
                if 'root hub' in desc.lower():
                    continue
                devices.append({
                    'bus':  bus,
                    'device': dev,
                    'id':   uid,
                    'name': desc.strip() or 'Unknown device',
                })
    except Exception:
        pass
    return devices

def get_uptime():
    try:
        with open('/proc/uptime') as f:
            secs = float(f.read().split()[0])
        days  = int(secs // 86400)
        hours = int((secs % 86400) // 3600)
        mins  = int((secs % 3600) // 60)
        parts = []
        if days:  parts.append(f'{days}d')
        if hours: parts.append(f'{hours}h')
        parts.append(f'{mins}m')
        return ' '.join(parts)
    except Exception:
        return 'unknown'

def get_cpu_freq():
    try:
        with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq') as f:
            return round(int(f.read().strip()) / 1_000_000, 2)  # GHz
    except Exception:
        return None

def get_throttle_state():
    """Returns human-readable throttle/voltage warnings from vcgencmd."""
    try:
        out  = subprocess.check_output(['vcgencmd', 'get_throttled'], text=True)
        match = re.search(r'0x([0-9a-fA-F]+)', out)
        if not match:
            return []
        val  = int(match.group(1), 16)
        flags = []
        if val & 0x1:     flags.append('Under-voltage detected')
        if val & 0x2:     flags.append('Arm frequency capped')
        if val & 0x4:     flags.append('Currently throttled')
        if val & 0x8:     flags.append('Soft temperature limit')
        if val & 0x10000: flags.append('Under-voltage occurred')
        if val & 0x20000: flags.append('Arm freq cap occurred')
        if val & 0x40000: flags.append('Throttling occurred')
        return flags
    except Exception:
        return []

def get_network_stats():
    try:
        stats = {}
        with open('/proc/net/dev') as f:
            lines = f.readlines()[2:]
        for line in lines:
            parts = line.split()
            iface = parts[0].rstrip(':')
            if iface == 'lo':
                continue
            rx_mb = round(int(parts[1]) / 1_048_576, 2)
            tx_mb = round(int(parts[9]) / 1_048_576, 2)
            stats[iface] = {'rx_mb': rx_mb, 'tx_mb': tx_mb}
        return stats
    except Exception:
        return {}

def get_camera_devices():
    cams = []
    try:
        for i in range(5):
            path = f'/dev/video{i}'
            if os.path.exists(path):
                name = 'Unknown'
                try:
                    result = subprocess.check_output(
                        ['v4l2-ctl', '--device', path, '--info'],
                        text=True, stderr=subprocess.DEVNULL
                    )
                    for line in result.splitlines():
                        if 'Card type' in line:
                            name = line.split(':', 1)[1].strip()
                except Exception:
                    name = 'USB Camera'
                cams.append({'device': path, 'name': name})
    except Exception:
        pass
    return cams

def get_gpio_pins():
    """Returns state of the rover's known GPIO pins via /sys or lgpio."""
    PIN_NAMES = {
        25: 'Status LED',
        12: 'Motor L ENA',
        27: 'Motor L IN1',
        17: 'Motor L IN2',
        19: 'Motor R ENB',
        6:  'Motor R IN3',
        5:  'Motor R IN4',
        23: 'Ultrasonic TRIG',
        24: 'Ultrasonic ECHO',
    }
    pins = []
    for bcm, name in PIN_NAMES.items():
        path = f'/sys/class/gpio/gpio{bcm}/value'
        val = None
        try:
            with open(path) as f:
                val = int(f.read().strip())
        except Exception:
            pass
        pins.append({'bcm': bcm, 'name': name, 'value': val})
    return pins

def get_processes():
    """Top 5 CPU-consuming processes."""
    try:
        result = subprocess.check_output(
            ['ps', 'aux', '--sort=-%cpu'],
            text=True
        )
        procs = []
        for line in result.splitlines()[1:6]:
            parts = line.split(None, 10)
            if len(parts) >= 11:
                procs.append({
                    'pid':  parts[1],
                    'cpu':  parts[2],
                    'mem':  parts[3],
                    'name': parts[10][:60],
                })
        return procs
    except Exception:
        return []

# ── API route ──────────────────────────────────────────────────────────────────

@app.route('/api/stats')
def api_stats():
    return jsonify({
        'timestamp':    time.strftime('%Y-%m-%d %H:%M:%S'),
        'uptime':       get_uptime(),
        'ips':          get_ip_addresses(),
        'cpu_temp_c':   get_cpu_temp(),
        'gpu_temp_c':   get_gpu_temp(),
        'cpu_freq_ghz': get_cpu_freq(),
        'cpu_percent':  get_cpu_usage(),
        'memory':       get_memory(),
        'disk':         get_disk(),
        'usb_devices':  get_usb_devices(),
        'cameras':      get_camera_devices(),
        'network':      get_network_stats(),
        'throttle':     get_throttle_state(),
        'gpio_pins':    get_gpio_pins(),
        'top_processes':get_processes(),
    })

@app.route('/')
def index():
    return render_template_string(HTML)

# ── HTML Dashboard ─────────────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Rover Diagnostics</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Barlow:wght@300;400;600;700&display=swap');

*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

:root {
  --bg:       #0a0c0f;
  --panel:    #0f1318;
  --border:   #1c2330;
  --glow:     #00e5ff;
  --glow2:    #00ff9d;
  --warn:     #ffb300;
  --danger:   #ff3d3d;
  --text:     #c8d8e8;
  --muted:    #4a6070;
  --mono:     'Share Tech Mono', monospace;
  --sans:     'Barlow', sans-serif;
}

html, body { height: 100%; }

body {
  font-family: var(--sans);
  background: var(--bg);
  color: var(--text);
  min-height: 100vh;
  padding: 0;
  overflow-x: hidden;
}

/* Scanline overlay */
body::before {
  content: '';
  position: fixed;
  inset: 0;
  background: repeating-linear-gradient(
    0deg,
    transparent,
    transparent 2px,
    rgba(0,229,255,0.015) 2px,
    rgba(0,229,255,0.015) 4px
  );
  pointer-events: none;
  z-index: 1000;
}

/* ── Header ── */
.header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 18px 32px;
  border-bottom: 1px solid var(--border);
  background: linear-gradient(90deg, rgba(0,229,255,0.04) 0%, transparent 60%);
  position: sticky;
  top: 0;
  z-index: 100;
  backdrop-filter: blur(8px);
}

.header-left {
  display: flex;
  align-items: baseline;
  gap: 16px;
}

.logo {
  font-family: var(--mono);
  font-size: 1.1rem;
  color: var(--glow);
  text-shadow: 0 0 12px rgba(0,229,255,0.6);
  letter-spacing: 0.12em;
}

.logo-sub {
  font-family: var(--mono);
  font-size: 0.65rem;
  color: var(--muted);
  letter-spacing: 0.15em;
  text-transform: uppercase;
}

.header-right {
  display: flex;
  align-items: center;
  gap: 20px;
}

.live-dot {
  display: flex;
  align-items: center;
  gap: 7px;
  font-family: var(--mono);
  font-size: 0.65rem;
  color: var(--glow2);
  letter-spacing: 0.1em;
}

.live-dot::before {
  content: '';
  width: 7px; height: 7px;
  border-radius: 50%;
  background: var(--glow2);
  box-shadow: 0 0 8px var(--glow2);
  animation: blink 1.6s infinite;
}

@keyframes blink {
  0%,100% { opacity: 1; }
  50%      { opacity: 0.2; }
}

.timestamp {
  font-family: var(--mono);
  font-size: 0.65rem;
  color: var(--muted);
  letter-spacing: 0.06em;
}

/* ── Grid layout ── */
.grid {
  display: grid;
  grid-template-columns: repeat(12, 1fr);
  gap: 16px;
  padding: 24px 32px;
  max-width: 1600px;
  margin: 0 auto;
}

/* span helpers */
.col-4  { grid-column: span 4; }
.col-3  { grid-column: span 3; }
.col-6  { grid-column: span 6; }
.col-12 { grid-column: span 12; }

/* ── Panel ── */
.panel {
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 6px;
  padding: 20px;
  position: relative;
  overflow: hidden;
}

.panel::before {
  content: '';
  position: absolute;
  top: 0; left: 0; right: 0;
  height: 1px;
  background: linear-gradient(90deg, transparent, var(--glow), transparent);
  opacity: 0.3;
}

.panel-title {
  font-family: var(--mono);
  font-size: 0.6rem;
  letter-spacing: 0.18em;
  text-transform: uppercase;
  color: var(--muted);
  margin-bottom: 16px;
  display: flex;
  align-items: center;
  gap: 8px;
}

.panel-title::before {
  content: '';
  display: inline-block;
  width: 3px; height: 10px;
  background: var(--glow);
  box-shadow: 0 0 6px var(--glow);
}

/* ── Big stat ── */
.big-stat {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.big-val {
  font-family: var(--mono);
  font-size: 2.4rem;
  line-height: 1;
  color: var(--glow);
  text-shadow: 0 0 20px rgba(0,229,255,0.4);
  letter-spacing: -0.02em;
}

.big-unit {
  font-family: var(--mono);
  font-size: 0.75rem;
  color: var(--muted);
  letter-spacing: 0.1em;
}

/* ── Gauge bar ── */
.gauge-wrap { margin-top: 10px; }

.gauge-label {
  display: flex;
  justify-content: space-between;
  font-family: var(--mono);
  font-size: 0.62rem;
  color: var(--muted);
  margin-bottom: 5px;
  letter-spacing: 0.06em;
}

.gauge-track {
  height: 5px;
  background: var(--border);
  border-radius: 3px;
  overflow: hidden;
}

.gauge-fill {
  height: 100%;
  border-radius: 3px;
  background: var(--glow);
  box-shadow: 0 0 8px var(--glow);
  transition: width 0.6s ease, background 0.3s;
}

.gauge-fill.warn  { background: var(--warn);   box-shadow: 0 0 8px var(--warn);   }
.gauge-fill.hot   { background: var(--danger);  box-shadow: 0 0 8px var(--danger);  }

/* ── IP list ── */
.ip-list { list-style: none; display: flex; flex-direction: column; gap: 10px; }

.ip-item {
  display: flex;
  justify-content: space-between;
  align-items: baseline;
  gap: 12px;
  padding-bottom: 8px;
  border-bottom: 1px solid var(--border);
}

.ip-item:last-child { border-bottom: none; padding-bottom: 0; }

.ip-iface {
  font-family: var(--mono);
  font-size: 0.65rem;
  color: var(--muted);
  letter-spacing: 0.08em;
  flex-shrink: 0;
}

.ip-addr {
  font-family: var(--mono);
  font-size: 0.9rem;
  color: var(--glow2);
  text-shadow: 0 0 10px rgba(0,255,157,0.3);
  letter-spacing: 0.04em;
}

/* ── USB list ── */
.device-list { list-style: none; display: flex; flex-direction: column; gap: 8px; }

.device-item {
  display: flex;
  align-items: flex-start;
  gap: 10px;
  padding: 10px 12px;
  background: rgba(0,229,255,0.03);
  border: 1px solid var(--border);
  border-radius: 4px;
}

.device-icon {
  font-size: 0.8rem;
  margin-top: 2px;
  flex-shrink: 0;
}

.device-name {
  font-family: var(--sans);
  font-size: 0.8rem;
  font-weight: 400;
  color: var(--text);
  line-height: 1.4;
}

.device-id {
  font-family: var(--mono);
  font-size: 0.6rem;
  color: var(--muted);
  margin-top: 2px;
  letter-spacing: 0.06em;
}

.empty-msg {
  font-family: var(--mono);
  font-size: 0.7rem;
  color: var(--muted);
  letter-spacing: 0.06em;
  padding: 8px 0;
}

/* ── GPIO table ── */
.gpio-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(170px, 1fr));
  gap: 8px;
}

.gpio-pin {
  display: flex;
  align-items: center;
  gap: 10px;
  padding: 8px 10px;
  border: 1px solid var(--border);
  border-radius: 4px;
  background: rgba(255,255,255,0.01);
}

.gpio-light {
  width: 8px; height: 8px;
  border-radius: 50%;
  flex-shrink: 0;
  background: var(--muted);
  transition: background 0.3s, box-shadow 0.3s;
}

.gpio-light.on {
  background: var(--glow2);
  box-shadow: 0 0 8px var(--glow2);
}

.gpio-light.unknown {
  background: var(--border);
}

.gpio-info { flex: 1; min-width: 0; }

.gpio-name {
  font-family: var(--sans);
  font-size: 0.72rem;
  color: var(--text);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.gpio-bcm {
  font-family: var(--mono);
  font-size: 0.58rem;
  color: var(--muted);
  letter-spacing: 0.06em;
}

/* ── Process table ── */
.proc-table { width: 100%; border-collapse: collapse; }

.proc-table th {
  font-family: var(--mono);
  font-size: 0.58rem;
  letter-spacing: 0.12em;
  text-transform: uppercase;
  color: var(--muted);
  text-align: left;
  padding: 0 8px 10px;
  border-bottom: 1px solid var(--border);
}

.proc-table td {
  font-family: var(--mono);
  font-size: 0.72rem;
  color: var(--text);
  padding: 8px;
  border-bottom: 1px solid rgba(28,35,48,0.5);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  max-width: 300px;
}

.proc-table td.num { color: var(--glow); text-align: right; }

/* ── Throttle warnings ── */
.warn-list { display: flex; flex-direction: column; gap: 6px; }

.warn-item {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 12px;
  border-radius: 4px;
  background: rgba(255,61,61,0.07);
  border: 1px solid rgba(255,61,61,0.2);
  font-family: var(--mono);
  font-size: 0.7rem;
  color: var(--danger);
  letter-spacing: 0.04em;
}

.warn-item::before { content: '⚠'; }

.ok-msg {
  font-family: var(--mono);
  font-size: 0.7rem;
  color: var(--glow2);
  letter-spacing: 0.06em;
  display: flex;
  align-items: center;
  gap: 8px;
}

.ok-msg::before { content: '✓'; }

/* ── Network stats ── */
.net-list { list-style: none; display: flex; flex-direction: column; gap: 8px; }

.net-item {
  display: grid;
  grid-template-columns: 80px 1fr 1fr;
  align-items: center;
  gap: 12px;
  padding: 8px 10px;
  border: 1px solid var(--border);
  border-radius: 4px;
}

.net-iface {
  font-family: var(--mono);
  font-size: 0.65rem;
  color: var(--muted);
  letter-spacing: 0.08em;
}

.net-val {
  font-family: var(--mono);
  font-size: 0.75rem;
  color: var(--text);
}

.net-val span { color: var(--muted); font-size: 0.6rem; }

/* ── Dual stat row ── */
.stat-row {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 14px;
  margin-top: 4px;
}

.stat-cell { display: flex; flex-direction: column; gap: 4px; }

.stat-label {
  font-family: var(--mono);
  font-size: 0.58rem;
  color: var(--muted);
  letter-spacing: 0.1em;
  text-transform: uppercase;
}

.stat-val {
  font-family: var(--mono);
  font-size: 1.1rem;
  color: var(--text);
}

/* ── Camera section ── */
.cam-list { display: flex; flex-direction: column; gap: 8px; }

.cam-item {
  display: flex;
  align-items: center;
  gap: 10px;
  padding: 10px 12px;
  background: rgba(0,255,157,0.03);
  border: 1px solid rgba(0,255,157,0.12);
  border-radius: 4px;
}

.cam-led {
  width: 8px; height: 8px;
  border-radius: 50%;
  background: var(--glow2);
  box-shadow: 0 0 8px var(--glow2);
  flex-shrink: 0;
}

.cam-info { flex: 1; }
.cam-name { font-size: 0.8rem; color: var(--text); }
.cam-dev  { font-family: var(--mono); font-size: 0.6rem; color: var(--muted); margin-top: 2px; }

/* ── Uptime badge ── */
.uptime-val {
  font-family: var(--mono);
  font-size: 1.6rem;
  color: var(--glow2);
  text-shadow: 0 0 16px rgba(0,255,157,0.3);
  letter-spacing: 0.04em;
  margin-top: 4px;
}

/* ── Responsive ── */
@media (max-width: 900px) {
  .col-4, .col-3 { grid-column: span 6; }
  .col-6 { grid-column: span 12; }
  .grid { padding: 16px; gap: 12px; }
}

@media (max-width: 560px) {
  .col-4, .col-3, .col-6 { grid-column: span 12; }
  .header { padding: 14px 16px; }
  .logo-sub { display: none; }
}
</style>
</head>
<body>

<header class="header">
  <div class="header-left">
    <span class="logo">ROVER // DIAG</span>
    <span class="logo-sub">System Diagnostics</span>
  </div>
  <div class="header-right">
    <span class="live-dot">LIVE</span>
    <span class="timestamp" id="ts">—</span>
  </div>
</header>

<div class="grid">

  <!-- IP Addresses -->
  <div class="panel col-4" id="panel-ip">
    <div class="panel-title">Network Addresses</div>
    <ul class="ip-list" id="ip-list"><li class="empty-msg">loading…</li></ul>
  </div>

  <!-- Uptime -->
  <div class="panel col-4">
    <div class="panel-title">Uptime</div>
    <div class="uptime-val" id="uptime">—</div>
  </div>

  <!-- CPU Temp -->
  <div class="panel col-4">
    <div class="panel-title">CPU Temperature</div>
    <div class="big-stat">
      <div class="big-val" id="cpu-temp">—</div>
      <div class="big-unit">°C  (GPU: <span id="gpu-temp">—</span> °C)</div>
    </div>
    <div class="gauge-wrap">
      <div class="gauge-label"><span>0°C</span><span>85°C</span></div>
      <div class="gauge-track"><div class="gauge-fill" id="temp-bar" style="width:0%"></div></div>
    </div>
  </div>

  <!-- CPU Usage -->
  <div class="panel col-3">
    <div class="panel-title">CPU Usage</div>
    <div class="big-stat">
      <div class="big-val" id="cpu-pct">—</div>
      <div class="big-unit">%  @ <span id="cpu-freq">—</span> GHz</div>
    </div>
    <div class="gauge-wrap">
      <div class="gauge-label"><span>0%</span><span>100%</span></div>
      <div class="gauge-track"><div class="gauge-fill" id="cpu-bar" style="width:0%"></div></div>
    </div>
  </div>

  <!-- RAM -->
  <div class="panel col-3">
    <div class="panel-title">Memory</div>
    <div class="big-stat">
      <div class="big-val" id="ram-pct">—</div>
      <div class="big-unit">% used</div>
    </div>
    <div class="gauge-wrap">
      <div class="gauge-label">
        <span id="ram-used">— MB</span>
        <span id="ram-total">— MB</span>
      </div>
      <div class="gauge-track"><div class="gauge-fill" id="ram-bar" style="width:0%"></div></div>
    </div>
  </div>

  <!-- Disk -->
  <div class="panel col-3">
    <div class="panel-title">Disk (SD Card)</div>
    <div class="big-stat">
      <div class="big-val" id="disk-pct">—</div>
      <div class="big-unit">used</div>
    </div>
    <div class="stat-row" style="margin-top:12px;">
      <div class="stat-cell">
        <span class="stat-label">Used</span>
        <span class="stat-val" id="disk-used">—</span>
      </div>
      <div class="stat-cell">
        <span class="stat-label">Free</span>
        <span class="stat-val" id="disk-free">—</span>
      </div>
    </div>
  </div>

  <!-- Throttle warnings -->
  <div class="panel col-3">
    <div class="panel-title">Power / Throttle</div>
    <div id="throttle-content"><span class="empty-msg">loading…</span></div>
  </div>

  <!-- USB Devices -->
  <div class="panel col-6">
    <div class="panel-title">USB Devices</div>
    <ul class="device-list" id="usb-list"><li class="empty-msg">loading…</li></ul>
  </div>

  <!-- Cameras -->
  <div class="panel col-6">
    <div class="panel-title">Cameras Detected</div>
    <div class="cam-list" id="cam-list"><span class="empty-msg">loading…</span></div>
  </div>

  <!-- Network I/O -->
  <div class="panel col-6">
    <div class="panel-title">Network I/O (total since boot)</div>
    <ul class="net-list" id="net-list"><li class="empty-msg">loading…</li></ul>
  </div>

  <!-- GPIO Pins -->
  <div class="panel col-6">
    <div class="panel-title">Rover GPIO Pins</div>
    <div class="gpio-grid" id="gpio-grid"><span class="empty-msg">loading…</span></div>
  </div>

  <!-- Top Processes -->
  <div class="panel col-12">
    <div class="panel-title">Top Processes (by CPU)</div>
    <table class="proc-table">
      <thead>
        <tr>
          <th>PID</th>
          <th>CPU %</th>
          <th>MEM %</th>
          <th style="width:100%">Command</th>
        </tr>
      </thead>
      <tbody id="proc-body"><tr><td colspan="4" class="empty-msg">loading…</td></tr></tbody>
    </table>
  </div>

</div>

<script>
async function refresh() {
  let data;
  try {
    const res = await fetch('/api/stats');
    data = await res.json();
  } catch(e) { return; }

  // Timestamp
  document.getElementById('ts').textContent = data.timestamp || '—';

  // Uptime
  document.getElementById('uptime').textContent = data.uptime || '—';

  // IP addresses
  const ipList = document.getElementById('ip-list');
  const ips = data.ips || {};
  if (Object.keys(ips).length === 0) {
    ipList.innerHTML = '<li class="empty-msg">No addresses found</li>';
  } else {
    ipList.innerHTML = Object.entries(ips).map(([k,v]) =>
      `<li class="ip-item"><span class="ip-iface">${k}</span><span class="ip-addr">${v}</span></li>`
    ).join('');
  }

  // CPU temp
  const ct = data.cpu_temp_c;
  document.getElementById('cpu-temp').textContent = ct != null ? ct : '—';
  document.getElementById('gpu-temp').textContent = data.gpu_temp_c != null ? data.gpu_temp_c : '—';
  if (ct != null) {
    const pct = Math.min(100, (ct / 85) * 100);
    const bar = document.getElementById('temp-bar');
    bar.style.width = pct + '%';
    bar.className = 'gauge-fill' + (ct >= 80 ? ' hot' : ct >= 65 ? ' warn' : '');
  }

  // CPU usage
  const cp = data.cpu_percent;
  document.getElementById('cpu-pct').textContent = cp != null ? cp : '—';
  document.getElementById('cpu-freq').textContent = data.cpu_freq_ghz != null ? data.cpu_freq_ghz : '—';
  if (cp != null) {
    const bar = document.getElementById('cpu-bar');
    bar.style.width = cp + '%';
    bar.className = 'gauge-fill' + (cp >= 90 ? ' hot' : cp >= 70 ? ' warn' : '');
  }

  // RAM
  const mem = data.memory || {};
  document.getElementById('ram-pct').textContent = mem.percent != null ? mem.percent : '—';
  document.getElementById('ram-used').textContent = mem.used_mb ? mem.used_mb + ' MB' : '—';
  document.getElementById('ram-total').textContent = mem.total_mb ? mem.total_mb + ' MB' : '—';
  if (mem.percent != null) {
    const bar = document.getElementById('ram-bar');
    bar.style.width = mem.percent + '%';
    bar.className = 'gauge-fill' + (mem.percent >= 90 ? ' hot' : mem.percent >= 70 ? ' warn' : '');
  }

  // Disk
  const disk = data.disk || {};
  document.getElementById('disk-pct').textContent  = disk.percent || '—';
  document.getElementById('disk-used').textContent = disk.used    || '—';
  document.getElementById('disk-free').textContent = disk.free    || '—';

  // Throttle
  const thr = data.throttle || [];
  const thrEl = document.getElementById('throttle-content');
  if (thr.length === 0) {
    thrEl.innerHTML = '<div class="ok-msg">No issues detected</div>';
  } else {
    thrEl.innerHTML = '<div class="warn-list">' +
      thr.map(w => `<div class="warn-item">${w}</div>`).join('') +
      '</div>';
  }

  // USB
  const usbList = document.getElementById('usb-list');
  const usbs = data.usb_devices || [];
  if (usbs.length === 0) {
    usbList.innerHTML = '<li class="empty-msg">No USB devices found</li>';
  } else {
    usbList.innerHTML = usbs.map(d =>
      `<li class="device-item">
        <span class="device-icon">⬡</span>
        <div>
          <div class="device-name">${d.name}</div>
          <div class="device-id">ID ${d.id} · Bus ${d.bus} Dev ${d.device}</div>
        </div>
      </li>`
    ).join('');
  }

  // Cameras
  const camList = document.getElementById('cam-list');
  const cams = data.cameras || [];
  if (cams.length === 0) {
    camList.innerHTML = '<span class="empty-msg">No camera devices found</span>';
  } else {
    camList.innerHTML = cams.map(c =>
      `<div class="cam-item">
        <div class="cam-led"></div>
        <div class="cam-info">
          <div class="cam-name">${c.name}</div>
          <div class="cam-dev">${c.device}</div>
        </div>
      </div>`
    ).join('');
  }

  // Network
  const netList = document.getElementById('net-list');
  const net = data.network || {};
  const netEntries = Object.entries(net);
  if (netEntries.length === 0) {
    netList.innerHTML = '<li class="empty-msg">No interfaces found</li>';
  } else {
    netList.innerHTML = netEntries.map(([iface, s]) =>
      `<li class="net-item">
        <span class="net-iface">${iface}</span>
        <span class="net-val">↓ ${s.rx_mb} <span>MB rx</span></span>
        <span class="net-val">↑ ${s.tx_mb} <span>MB tx</span></span>
      </li>`
    ).join('');
  }

  // GPIO
  const gpioGrid = document.getElementById('gpio-grid');
  const pins = data.gpio_pins || [];
  if (pins.length === 0) {
    gpioGrid.innerHTML = '<span class="empty-msg">No pin data</span>';
  } else {
    gpioGrid.innerHTML = pins.map(p => {
      const cls = p.value === null ? 'unknown' : p.value === 1 ? 'on' : '';
      const val = p.value === null ? '?' : p.value === 1 ? 'HIGH' : 'LOW';
      return `<div class="gpio-pin">
        <div class="gpio-light ${cls}"></div>
        <div class="gpio-info">
          <div class="gpio-name">${p.name}</div>
          <div class="gpio-bcm">BCM ${p.bcm} · ${val}</div>
        </div>
      </div>`;
    }).join('');
  }

  // Processes
  const procs = data.top_processes || [];
  const tbody = document.getElementById('proc-body');
  if (procs.length === 0) {
    tbody.innerHTML = '<tr><td colspan="4" class="empty-msg">No data</td></tr>';
  } else {
    tbody.innerHTML = procs.map(p =>
      `<tr>
        <td>${p.pid}</td>
        <td class="num">${p.cpu}</td>
        <td class="num">${p.mem}</td>
        <td>${p.name}</td>
      </tr>`
    ).join('');
  }
}

// Refresh every 3 seconds
refresh();
setInterval(refresh, 3000);

// Tick the clock every second
setInterval(() => {
  const now = new Date();
  document.getElementById('ts').textContent =
    now.toISOString().replace('T',' ').slice(0,19);
}, 1000);
</script>
</body>
</html>"""

# ── Entry Point ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    print("=" * 40)
    print("  ROVER DIAGNOSTICS DASHBOARD")
    print("=" * 40)
    print(f"  Open in browser:")
    print(f"  http://{local_ip}:5001")
    print("=" * 40)
    print("  Ctrl+C to stop\n")
    try:
        app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\nDashboard stopped.")
