#!/usr/bin/env python3
"""
Simple utility to command one servo output on a PCA9685 by pulse width.

This is intended for assembly and calibration work where students are told an
approximate pulse width that should correspond to a servo position.

Examples:
    python3 moveMotor.py 1 2200
    python3 moveMotor.py 14 1500 --address 0x70
"""

import argparse
import sys

import board
from adafruit_pca9685 import PCA9685

SERVO_CHANNELS = (0, 1, 14, 15)
DEFAULT_ADDRESS = 0x6F
DEFAULT_FREQUENCY = 50
MIN_PULSE_US = 500
MAX_PULSE_US = 2500


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


def pulse_to_duty_cycle(pulse_us, frequency_hz):
    """Convert a pulse width in microseconds to a 16-bit duty cycle."""
    period_us = 1_000_000.0 / frequency_hz
    duty_cycle = int((pulse_us / period_us) * 0xFFFF)
    return clamp(duty_cycle, 0, 0xFFFF)


def pulse_to_angle(pulse_us, min_pulse_us=MIN_PULSE_US, max_pulse_us=MAX_PULSE_US):
    """Approximate servo command angle corresponding to a pulse width."""
    bounded = clamp(pulse_us, min_pulse_us, max_pulse_us)
    span = max_pulse_us - min_pulse_us
    if span == 0:
        return 0.0
    return (bounded - min_pulse_us) * 180.0 / span


def set_servo_pulse(pca, channel, pulse_us):
    """Set one PCA9685 channel to a pulse width in microseconds."""
    duty_cycle = pulse_to_duty_cycle(pulse_us, pca.frequency)
    pca.channels[channel].duty_cycle = duty_cycle
    return duty_cycle


def parse_args():
    parser = argparse.ArgumentParser(
        description="Command one servo output on a PCA9685 by pulse width."
    )
    parser.add_argument(
        "channel",
        type=int,
        choices=SERVO_CHANNELS,
        help="Servo channel to drive. Valid channels: 0, 1, 14, 15.",
    )
    parser.add_argument(
        "pulse_us",
        type=float,
        help="Pulse width in microseconds. Typical servo range is about 500 to 2500 us.",
    )
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=DEFAULT_ADDRESS,
        help="I2C address of the PCA9685 board. Default: 0x6F.",
    )
    parser.add_argument(
        "--frequency",
        type=int,
        default=DEFAULT_FREQUENCY,
        help="PWM frequency in Hz. Servos typically use 50 Hz.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    i2c = board.I2C()
    try:
        pca = PCA9685(i2c, address=args.address)
    except ValueError as exc:
        print(f"Error: PCA9685 was not found at I2C address 0x{args.address:02X}.", file=sys.stderr)
        print("Check power/wiring, run `i2cdetect -y 1`, or pass the detected address with `--address`.", file=sys.stderr)
        raise SystemExit(1) from exc

    pca.frequency = args.frequency

    try:
        duty_cycle = set_servo_pulse(pca, args.channel, args.pulse_us)
        approx_angle = pulse_to_angle(args.pulse_us)
        print(f"Channel: {args.channel}")
        print(f"Address: 0x{args.address:02X}")
        print(f"Frequency: {args.frequency} Hz")
        print(f"Pulse width: {args.pulse_us:.0f} us")
        print(f"Duty cycle: {duty_cycle} / 65535")
        print(f"Approx angle: {approx_angle:.1f} deg (assuming {MIN_PULSE_US}-{MAX_PULSE_US} us -> 0-180 deg)")
    finally:
        pca.deinit()


if __name__ == "__main__":
    main()
