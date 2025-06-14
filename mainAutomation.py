#!/usr/bin/env python3
"""
Infinite-loop art piece controller:
  - Drives 4 continuous rotation servos via a PCA9685 board at their slowest speed.
  - Controls a 7-channel relay board (using BCM numbering) to run 3 different LED patterns.
  
The servos are set to run continuously at a slight offset from neutral.
LED patterns cycle one after another indefinitely.
Press Ctrl+C to exit.
"""

import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685

# ----------------------------
# I2C Configuration for PCA9685
# ----------------------------
# Define the I2C pins for the PCA9685 board.
I2C_SCL = board.SCL   # Typically, the SCL pin on the Raspberry Pi (GPIO3)
I2C_SDA = board.SDA   # Typically, the SDA pin on the Raspberry Pi (GPIO2)
PCA9685_ADDRESS = 0x40  # Default I2C address for PCA9685; update if yours is different

# ----------------------------
# Servo (PCA9685) Configuration
# ----------------------------
SERVO_FREQUENCY = 50             # Hz (typical for servos; 20ms period)
SERVO_NEUTRAL_PULSE = 1.5        # ms pulse width for neutral (stop)
SERVO_SLOW_PULSE = 1.6           # ms pulse width for slow rotation (adjust as needed)
SERVO_CHANNELS = [0, 1, 2, 3]    # PCA9685 channels connected to the servos

def pulse_ms_to_duty_cycle(pulse_ms, frequency):
    """
    Convert a pulse width in milliseconds to a 16-bit duty cycle value
    for the PCA9685.
    """
    period_ms = 1000.0 / frequency
    duty_cycle = int((pulse_ms / period_ms) * 0xFFFF)
    return duty_cycle

# Compute duty cycle values (16-bit) for neutral and slow speed
SERVO_NEUTRAL_DUTY = pulse_ms_to_duty_cycle(SERVO_NEUTRAL_PULSE, SERVO_FREQUENCY)
SERVO_SLOW_DUTY    = pulse_ms_to_duty_cycle(SERVO_SLOW_PULSE, SERVO_FREQUENCY)

# ----------------------------
# Relay (LED) Configuration
# ----------------------------
# BCM GPIO pins connected to the relay board (adjust as needed)
RELAY_PINS = [5, 6, 13, 19, 26, 21, 20]

def setup_gpio():
    """Initialize GPIO for relay control."""
    GPIO.setmode(GPIO.BCM)
    for pin in RELAY_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

def cleanup_gpio():
    """Clean up GPIO resources."""
    GPIO.cleanup()

# ----------------------------
# LED (Relay) Pattern Functions
# ----------------------------
def led_pattern_all(duration=10, interval=1):
    """
    Pattern 1: Turn all relays ON, wait, then OFF.
    """
    end_time = time.time() + duration
    while time.time() < end_time:
        for pin in RELAY_PINS:
            GPIO.output(pin, GPIO.HIGH)
        time.sleep(interval)
        for pin in RELAY_PINS:
            GPIO.output(pin, GPIO.LOW)
        time.sleep(interval)

def led_pattern_sequential(duration=10, interval=0.5):
    """
    Pattern 2: Sequentially turn each relay ON then OFF.
    """
    end_time = time.time() + duration
    while time.time() < end_time:
        for pin in RELAY_PINS:
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(interval)
        for pin in RELAY_PINS:
            GPIO.output(pin, GPIO.LOW)
            time.sleep(interval)

def led_pattern_alternate(duration=10, interval=0.5):
    """
    Pattern 3: Alternate between even-indexed and odd-indexed relays.
    """
    end_time = time.time() + duration
    while time.time() < end_time:
        for idx, pin in enumerate(RELAY_PINS):
            GPIO.output(pin, GPIO.HIGH if idx % 2 == 0 else GPIO.LOW)
        time.sleep(interval)
        for idx, pin in enumerate(RELAY_PINS):
            GPIO.output(pin, GPIO.HIGH if idx % 2 == 1 else GPIO.LOW)
        time.sleep(interval)

# ----------------------------
# Main Loop
# ----------------------------
def main():
    # Initialize I2C bus using specified I2C pins for the PCA9685 board
    i2c = busio.I2C(I2C_SCL, I2C_SDA)
    pca = PCA9685(i2c, address=PCA9685_ADDRESS)
    pca.frequency = SERVO_FREQUENCY
    print(f"PCA9685 initialized with frequency {pca.frequency}Hz at address {PCA9685_ADDRESS:#04x}")

    # Initialize GPIO for relay control
    setup_gpio()

    try:
        # Set all servos to slow rotation
        for channel in SERVO_CHANNELS:
            pca.channels[channel].duty_cycle = SERVO_SLOW_DUTY
        print("Servos set to slow rotation.")

        # Infinite loop cycling through LED patterns
        while True:
            print("Running LED pattern: All ON/OFF")
            led_pattern_all(duration=10, interval=1)
            print("Running LED pattern: Sequential")
            led_pattern_sequential(duration=10, interval=0.5)
            print("Running LED pattern: Alternate")
            led_pattern_alternate(duration=10, interval=0.5)
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Exiting...")
    
    except Exception as e:
        print("Unexpected error:", e)
    
    finally:
        # Set servos to neutral (stop) before exiting
        for channel in SERVO_CHANNELS:
            pca.channels[channel].duty_cycle = SERVO_NEUTRAL_DUTY
        cleanup_gpio()
        pca.deinit()
        print("Cleanup complete.")

if __name__ == '__main__':
    main()
