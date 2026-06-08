#!/usr/bin/env python3
"""
Rover Motor Control - Raspberry Pi 4 + L298N H-Bridge
Compatible with Debian Trixie (Debian 13) using gpiozero + lgpio
SSH-friendly held-key control using readchar + threads.
=====================================================
Wiring (BCM / BOARD):
  ENA  -> GPIO 12 / BOARD 32  — Left motor PWM speed
  IN1  -> GPIO 27 / BOARD 13  — Left motor direction A
  IN2  -> GPIO 17 / BOARD 11  — Left motor direction B
  ENB  -> GPIO 19 / BOARD 35  — Right motor PWM speed
  IN3  -> GPIO 6  / BOARD 31  — Right motor direction A
  IN4  -> GPIO 5  / BOARD 29  — Right motor direction B
  LED  -> GPIO 25 / BOARD 22  — Status LED
  GND  -> Any GND pin         — Common ground

Install deps:  pip3 install readchar
Run:           python3 rover_control.py
Test sequence: python3 rover_control.py test
"""

import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

from gpiozero import LED, PWMOutputDevice, DigitalOutputDevice
from time import sleep, time
import threading
import readchar
import sys

# ── Pin Definitions (BCM) ─────────────────────────────────────────────────────
STATUS_LED      = 25
MOTOR_LEFT_ENA  = 12
MOTOR_LEFT_IN1  = 27
MOTOR_LEFT_IN2  = 17
MOTOR_RIGHT_ENB = 19
MOTOR_RIGHT_IN3 = 6
MOTOR_RIGHT_IN4 = 5

DEFAULT_SPEED = 0.75

# ── Device Setup ──────────────────────────────────────────────────────────────
led       = LED(STATUS_LED)

left_ena  = PWMOutputDevice(MOTOR_LEFT_ENA,  initial_value=0)
left_in1  = DigitalOutputDevice(MOTOR_LEFT_IN1,  initial_value=False)
left_in2  = DigitalOutputDevice(MOTOR_LEFT_IN2,  initial_value=False)

right_enb = PWMOutputDevice(MOTOR_RIGHT_ENB, initial_value=0)
right_in3 = DigitalOutputDevice(MOTOR_RIGHT_IN3, initial_value=False)
right_in4 = DigitalOutputDevice(MOTOR_RIGHT_IN4, initial_value=False)


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


def cleanup() -> None:
    stop()
    for device in (left_ena, left_in1, left_in2,
                   right_enb, right_in3, right_in4, led):
        device.close()


# ── Held-Key Logic ────────────────────────────────────────────────────────────
#
# Over SSH there is no key-release event, so we simulate it:
# Every keypress updates a timestamp. A watchdog thread checks every
# KEY_TIMEOUT seconds — if no key has been pressed recently, it stops
# the motors. As long as you hold a key, the terminal repeats it fast
# enough to keep the watchdog fed.
#
KEY_TIMEOUT = 0.15   # seconds — stop if no keypress received within this time

BINDINGS = {
    'w': (( DEFAULT_SPEED,  DEFAULT_SPEED), 'FORWARD'),
    's': ((-DEFAULT_SPEED, -DEFAULT_SPEED), 'BACKWARD'),
    'a': ((-DEFAULT_SPEED,  DEFAULT_SPEED), 'SPIN LEFT'),
    'd': (( DEFAULT_SPEED, -DEFAULT_SPEED), 'SPIN RIGHT'),
    'q': ((0.0,             DEFAULT_SPEED), 'PIVOT LEFT'),
    'e': (( DEFAULT_SPEED,  0.0          ), 'PIVOT RIGHT'),
}

last_keypress = 0.0
current_key   = None
running       = True


def watchdog() -> None:
    """Stops motors if no key has been pressed within KEY_TIMEOUT seconds."""
    global current_key
    while running:
        if current_key is not None and (time() - last_keypress) > KEY_TIMEOUT:
            stop()
            led.off()
            current_key = None
            print("\r[STOP]                  ", end='', flush=True)
        sleep(0.02)


def interactive_mode() -> None:
    global last_keypress, current_key, running

    print("\n" + "=" * 40)
    print("  ROVER — HELD KEY CONTROL")
    print("=" * 40)
    print("  W = Forward      S = Backward")
    print("  A = Spin Left    D = Spin Right")
    print("  Q = Pivot Left   E = Pivot Right")
    print("  ESC / Ctrl+C = Quit")
    print("=" * 40)
    print("Hold a key to move, release to stop.\n")

    # Startup blink
    for _ in range(3):
        led.on();  sleep(0.2)
        led.off(); sleep(0.2)

    # Start watchdog thread
    wt = threading.Thread(target=watchdog, daemon=True)
    wt.start()

    try:
        while True:
            ch = readchar.readchar().lower()

            # Quit on ESC or Ctrl+C
            if ch in ('\x1b', '\x03'):
                break

            if ch in BINDINGS:
                (left, right), label = BINDINGS[ch]
                last_keypress = time()
                current_key = ch
                set_motors(left, right)
                led.on()
                print(f"\r[{label}] speed={int(DEFAULT_SPEED*100)}%   ", end='', flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        cleanup()
        print("\nRover stopped. GPIO released.")


# ── Test Sequence ─────────────────────────────────────────────────────────────

def run_test_sequence() -> None:
    print("=" * 40)
    print("  ROVER TEST SEQUENCE STARTING")
    print("=" * 40)

    try:
        for _ in range(3):
            led.on();  sleep(0.2)
            led.off(); sleep(0.2)

        print("\n1. Forward 2s")
        set_motors(0.75, 0.75);   sleep(2); stop(); sleep(0.5)

        print("\n2. Backward 2s")
        set_motors(-0.75, -0.75); sleep(2); stop(); sleep(0.5)

        print("\n3. Pivot Left 1s")
        set_motors(0.0, 0.7);     sleep(1); stop(); sleep(0.5)

        print("\n4. Pivot Right 1s")
        set_motors(0.7, 0.0);     sleep(1); stop(); sleep(0.5)

        print("\n5. Spin Left 1s")
        set_motors(-0.65, 0.65);  sleep(1); stop(); sleep(0.5)

        print("\n6. Spin Right 1s")
        set_motors(0.65, -0.65);  sleep(1); stop(); sleep(0.5)

        print("\n✅ TEST SEQUENCE COMPLETE")

    except KeyboardInterrupt:
        print("\n⚠ Interrupted by user")

    finally:
        cleanup()


# ── Entry Point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "interactive"
    if mode == "test":
        run_test_sequence()
    else:
        interactive_mode()
