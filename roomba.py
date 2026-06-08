#!/usr/bin/env python3
"""
Roomba Mode - Raspberry Pi 4 + L298N H-Bridge + HC-SR04
Drives autonomously, backs up and turns right when an obstacle is detected.

Run:  python3 roomba.py
Stop: Ctrl+C
"""

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import LED, PWMOutputDevice, DigitalOutputDevice
from time import sleep, time
import threading
import lgpio

# ── Pin Definitions (BCM) ─────────────────────────────────────────────────────
STATUS_LED      = 25
MOTOR_LEFT_ENA  = 12
MOTOR_LEFT_IN1  = 27
MOTOR_LEFT_IN2  = 17
MOTOR_RIGHT_ENB = 19
MOTOR_RIGHT_IN3 = 6
MOTOR_RIGHT_IN4 = 5
TRIG_PIN        = 23
ECHO_PIN        = 24

# ── Tuning ────────────────────────────────────────────────────────────────────
DRIVE_SPEED      = 0.75  # 0.0 - 1.0
OBSTACLE_DIST_CM = 25.0  # stop and avoid if closer than this
BACKUP_TIME      = 0.6   # seconds to reverse
TURN_TIME        = 0.5   # seconds to spin right
SENSOR_INTERVAL  = 0.08  # how often to poll the sensor (seconds)

# ── Device Setup ──────────────────────────────────────────────────────────────
led       = LED(STATUS_LED)
left_ena  = PWMOutputDevice(MOTOR_LEFT_ENA,  initial_value=0)
left_in1  = DigitalOutputDevice(MOTOR_LEFT_IN1,  initial_value=False)
left_in2  = DigitalOutputDevice(MOTOR_LEFT_IN2,  initial_value=False)
right_enb = PWMOutputDevice(MOTOR_RIGHT_ENB, initial_value=0)
right_in3 = DigitalOutputDevice(MOTOR_RIGHT_IN3, initial_value=False)
right_in4 = DigitalOutputDevice(MOTOR_RIGHT_IN4, initial_value=False)
trig      = DigitalOutputDevice(TRIG_PIN, initial_value=False)

_gpio = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(_gpio, ECHO_PIN)

# ── Motor Helpers ─────────────────────────────────────────────────────────────

def set_motors(left: float, right: float) -> None:
    left_in1.value  = left  > 0
    left_in2.value  = left  < 0
    left_ena.value  = abs(left)
    right_in3.value = right > 0
    right_in4.value = right < 0
    right_enb.value = abs(right)

def forward():
    set_motors(DRIVE_SPEED, DRIVE_SPEED)
    led.on()

def stop_motors():
    set_motors(0, 0)
    led.off()

def cleanup():
    stop_motors()
    lgpio.gpiochip_close(_gpio)
    for dev in (left_ena, left_in1, left_in2,
                right_enb, right_in3, right_in4, led, trig):
        dev.close()

# ── Ultrasonic Sensor ─────────────────────────────────────────────────────────
_sonar_lock = threading.Lock()

def get_distance_cm() -> float:
    """Returns distance in cm, or 999 on timeout."""
    with _sonar_lock:
        trig.off()
        sleep(0.000002)
        trig.on()
        sleep(0.00001)
        trig.off()

        deadline = time() + 0.04
        while lgpio.gpio_read(_gpio, ECHO_PIN) == 0:
            if time() > deadline:
                return 999.0
        pulse_start = time()

        deadline = time() + 0.04
        while lgpio.gpio_read(_gpio, ECHO_PIN) == 1:
            if time() > deadline:
                return 999.0
        pulse_end = time()

    return round((pulse_end - pulse_start) * 34300 / 2, 1)

# ── Avoidance Sequence ────────────────────────────────────────────────────────

def avoid_obstacle():
    """Stop -> back up -> turn right -> brief pause (caller resumes forward)."""
    print("  [!] Obstacle detected -- avoiding")

    # 1. Full stop
    stop_motors()
    sleep(0.15)

    # 2. Back up
    print("  [<] Backing up...")
    set_motors(-DRIVE_SPEED, -DRIVE_SPEED)
    sleep(BACKUP_TIME)

    # 3. Turn right (left motor forward, right motor back)
    print("  [>] Turning right...")
    set_motors(DRIVE_SPEED, -DRIVE_SPEED)
    sleep(TURN_TIME)

    # 4. Brief stop before resuming
    stop_motors()
    sleep(0.1)
    print("  [^] Resuming forward")

# ── Main Loop ─────────────────────────────────────────────────────────────────

def run():
    print("=" * 40)
    print("  ROOMBA MODE")
    print("=" * 40)
    print(f"  Speed:              {int(DRIVE_SPEED * 100)}%")
    print(f"  Obstacle threshold: {OBSTACLE_DIST_CM} cm")
    print("  Ctrl+C to stop")
    print("=" * 40 + "\n")

    # Blink LED to signal startup
    for _ in range(3):
        led.on();  sleep(0.15)
        led.off(); sleep(0.15)

    forward()
    print("[^] Driving forward...\n")

    try:
        while True:
            dist = get_distance_cm()

            if dist < OBSTACLE_DIST_CM:
                avoid_obstacle()
                forward()  # resume after avoidance

            sleep(SENSOR_INTERVAL)

    except KeyboardInterrupt:
        print("\n[.] Stopped by user.")
    finally:
        cleanup()
        print("[.] GPIO released.")


if __name__ == "__main__":
    run()
