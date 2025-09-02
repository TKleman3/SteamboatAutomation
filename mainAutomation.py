#!/usr/bin/env python3
"""
Infinite-loop art piece controller (Pi 5 ready)

- Drives 4 continuous-rotation servos via a PCA9685 board at a gentle, slow speed.
- Controls a 7-channel relay board to run 3 LED patterns in a loop.
- Clean shutdown on Ctrl+C / SIGTERM.

Dependencies (Pi 5 / Bookworm):
  sudo apt update
  sudo apt install -y python3-gpiozero python3-pip
  pip3 install --upgrade adafruit-blinka adafruit-circuitpython-pca9685 adafruit-circuitpython-motor
Enable I2C: sudo raspi-config  → Interface Options → I2C → Enable
"""

import time
import signal
import sys
import board
from gpiozero import OutputDevice
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# ----------------------------
# PCA9685 (I2C) Configuration
# ----------------------------
PCA9685_ADDRESS = 0x40    # Change if you moved the address jumpers
SERVO_FREQUENCY = 50      # 50 Hz = ~20 ms period (standard for servos)

# Servo channels (0-15 on PCA9685)
SERVO_CHANNELS = [0, 1, 2, 3]

# A small forward throttle for slow rotation on continuous servos
# (tweak per your servos; 0 = stop, 0.03..0.15 = slow, negative for reverse)
SLOW_THROTTLE = 0.08

# Optional per-servo trims if your “neutral” differs channel-to-channel
PER_SERVO_TRIM = [0.0, 0.0, 0.0, 0.0]  # add small +/- offsets if needed

# ----------------------------
# Relay (LED) Configuration
# ----------------------------
# BCM pin numbers connected to your relay board
RELAY_PINS = [5, 6, 13, 19, 26, 21, 20]

# Most 5V relay boards are ACTIVE-LOW (ON when input is pulled LOW).
# Set this to False for those boards. Set to True if your board is active-high.
RELAY_ACTIVE_HIGH = False

# ----------------------------
# Globals for cleanup
# ----------------------------
relays = []
pca = None
servos = []


def setup_relays():
    """Create gpiozero OutputDevice instances for each relay."""
    return [OutputDevice(pin, active_high=RELAY_ACTIVE_HIGH, initial_value=False) for pin in RELAY_PINS]


def setup_pca9685():
    """Initialize PCA9685 over I2C and return driver instance."""
    # Using board.I2C() is the recommended, portable way under Blinka
    i2c = board.I2C()  # uses board.SCL and board.SDA
    p = PCA9685(i2c, address=PCA9685_ADDRESS)
    p.frequency = SERVO_FREQUENCY
    return p


def setup_servos(pca_dev):
    """
    Wrap PCA9685 channels with ContinuousServo for human-friendly control.
    min_pulse/max_pulse can be tuned per servo model; 1000–2000 µs is common.
    """
    s = []
    for idx, ch in enumerate(SERVO_CHANNELS):
        s.append(servo.ContinuousServo(
            pca_dev.channels[ch],
            min_pulse=1000,  # µs
            max_pulse=2000   # µs
        ))
    return s


# ----------------------------
# LED (Relay) Pattern Functions
# ----------------------------
def led_pattern_all(duration=10, interval=1.0):
    """
    Pattern 1: Turn ALL relays ON, wait, then ALL OFF, repeatedly for 'duration' seconds.
    """
    end = time.time() + duration
    while time.time() < end:
        for r in relays: r.on()
        time.sleep(interval)
        for r in relays: r.off()
        time.sleep(interval)


def led_pattern_sequential(duration=10, interval=0.5):
    """
    Pattern 2: Step through relays ON one-by-one, then OFF one-by-one.
    """
    end = time.time() + duration
    while time.time() < end:
        for r in relays:
            r.on()
            time.sleep(interval)
        for r in relays:
            r.off()
            time.sleep(interval)


def led_pattern_alternate(duration=10, interval=0.5):
    """
    Pattern 3: Alternate between even-indexed and odd-indexed relays.
    """
    end = time.time() + duration
    while time.time() < end:
        # evens ON, odds OFF
        for i, r in enumerate(relays):
            (r.on() if i % 2 == 0 else r.off())
        time.sleep(interval)
        # odds ON, evens OFF
        for i, r in enumerate(relays):
            (r.on() if i % 2 == 1 else r.off())
        time.sleep(interval)


# ----------------------------
# Cleanup & Signal Handling
# ----------------------------
def safe_stop():
    """Stop servos, turn off relays, and release hardware cleanly."""
    try:
        for s, trim in zip(servos, PER_SERVO_TRIM):
            s.throttle = 0.0 + trim  # neutral (trim lets you bias if needed)
    except Exception:
        pass
    try:
        for r in relays:
            r.off()
            r.close()
    except Exception:
        pass
    try:
        if pca is not None:
            pca.deinit()
    except Exception:
        pass


def handle_exit(signum=None, frame=None):
    print("\nShutting down…")
    safe_stop()
    sys.exit(0)


# ----------------------------
# Main
# ----------------------------
def main():
    global relays, pca, servos

    # Arrange for clean shutdown on Ctrl+C and service stop
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    # Hardware setup
    pca = setup_pca9685()
    print(f"PCA9685 ready @ 0x{PCA9685_ADDRESS:02X}, {SERVO_FREQUENCY} Hz")

    relays = setup_relays()
    print(f"Initialized {len(relays)} relays (active_high={RELAY_ACTIVE_HIGH})")

    servos = setup_servos(pca)

    # Spin servos slowly (continuous rotation)
    for idx, (s, trim) in enumerate(zip(servos, PER_SERVO_TRIM)):
        s.throttle = SLOW_THROTTLE + trim
        print(f"Servo {idx} set to throttle {SLOW_THROTTLE + trim:.3f}")

    # Loop through patterns indefinitely
    while True:
        print("Running LED pattern: All ON/OFF")
        led_pattern_all(duration=10, interval=1.0)
        print("Running LED pattern: Sequential")
        led_pattern_sequential(duration=10, interval=0.5)
        print("Running LED pattern: Alternate")
        led_pattern_alternate(duration=10, interval=0.5)
        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Unexpected error:", e)
        handle_exit()
