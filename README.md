# meArmPi

Python control tools for the Phenoptix meArm on Raspberry Pi using Adafruit's CircuitPython motor libraries.

This repository contains:

- `meArm.py`: inverse-kinematics and direct joint-angle control for the 4-servo meArm
- `Controller.py`: keyboard and gamepad controller in Cartesian `x/y/z` space
- `Controller-Direct.py`: keyboard and gamepad controller in direct joint/device angles
- `Controller-Stepper.py`: keyboard and joystick controller for 2 steppers on an Adafruit Motor HAT
- `Controller-Direct-wStepper.py`: combined direct-servo and M3/M4 stepper controller with exclusive hardware modes
- `Zero.py`: calibration tool for servo zero offsets and joint limits
- `moveMotor.py`: simple pulse-width utility for servo setup during assembly
- `mearm_config.json`: saved zero offsets and per-joint angle limits

## Hardware

The meArm uses four servos:

- gripper
- base
- shoulder
- elbow

The servo-based arm control in this repository uses:

- `board`
- `adafruit_pca9685`
- `adafruit_motor.servo`

The stepper controller uses:

- `board`
- `adafruit_motorkit`
- `adafruit_motor.stepper`

### Wiring

The servo arm is driven from a PCA9685-based servo board. In the current code the channels are:

- channel `0`: base
- channel `1`: shoulder
- channel `14`: elbow
- channel `15`: gripper

This mapping is used in both [`meArm.py`](./meArm.py) and [`Zero.py`](./Zero.py).

## Project Overview

`meArm.py` supports two control modes:

1. Cartesian motion using inverse kinematics
2. Direct joint/device angle control

Cartesian motion tracks and commands:

- `x`
- `y`
- `z`
- gripper opening percentage

Direct motion tracks and commands:

- `base`
- `shoulder`
- `elbow`
- `gripper`

The servo zero offsets and joint limits are loaded from `mearm_config.json`.

### Configuration Model

Each joint in `mearm_config.json` has:

- `zero`: servo command angle used as the calibrated zero offset
- `min_deg`: minimum joint/device angle
- `max_deg`: maximum joint/device angle

Important distinction:

- joint/device angles are the physical arm angles used by `meArm.py`
- servo command angles are the raw `0..180` values sent to the Adafruit servo API
- pulse width in microseconds is the low-level PWM representation used by `moveMotor.py`

## Installation

Clone the repository and install the Python dependencies you need for the tools you plan to use.

Python dependencies:

- `pygame`
- `adafruit-circuitpython-pca9685`
- `adafruit-circuitpython-motor`
- `adafruit-circuitpython-motorkit`

Example:

```bash
python3 -m pip install pygame \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    adafruit-circuitpython-motorkit
```

Raspberry Pi prerequisites:

- enable I2C on the Raspberry Pi
- make sure the user running the scripts has permission to access I2C devices
- install system support packages if your image does not already provide them

Typical system packages on Raspberry Pi OS:

- `python3-pip`
- `python3-dev`
- `python3-smbus`
- `i2c-tools`

Example:

```bash
sudo apt update
sudo apt install -y python3-pip python3-dev python3-smbus i2c-tools
```

To verify the board is visible on I2C:

```bash
i2cdetect -y 1
```

## Core Library Usage

Example using Cartesian inverse-kinematics control:

```python
import meArm

arm = meArm.meArm(address=0x6F)

arm.open_gripper()
arm.move_to(-80, 100, 140)
arm.close_gripper()
arm.move_linear(70, 200, 10)
arm.open_gripper()
arm.move_to(0, 150, 100)
```

Example using direct joint-angle control:

```python
import meArm

arm = meArm.meArm(address=0x6F)

arm.set_joint_angles(base=0, shoulder=90, elbow=0)
arm.set_gripper_angle(40)

print(arm.get_joint_angles())
print(arm.get_gripper_angle())
```

## Current `meArm` Methods

Main Cartesian methods:

- `open_gripper()`
- `close_gripper()`
- `partial_grip(percent)`
- `move_to(x, y, z)`
- `move_linear(x, y, z, step=10.0, delay=0.05)`
- `get_position()`
- `get_finger()`

Direct-angle methods:

- `set_joint_angles(base, shoulder, elbow)`
- `set_gripper_angle(gripper)`
- `get_joint_angles()`
- `get_gripper_angle()`

## Included Programs

### `Controller.py`

- keyboard and gamepad control in Cartesian space
- arrow keys and `w/s` move the arm in `x/y/z`
- gripper is controlled as open percentage

Keyboard:

- `Left Arrow` / `Right Arrow`: move `x` left/right.
- `Up Arrow` / `Down Arrow`: move `y` forward/backward.
- `W` / `S`: move `z` up/down.
- `O`: fully open gripper.
- `L`: fully close gripper.
- `P`: set gripper to 50%.
- `Q` / `A`: gradually open/close gripper.

Joystick:

- `axis 0`: `x` left/right.
- `axis 1`: `y` forward/backward.
- `axis 4`: `z` up/down.
- `axis 2` / `axis 5`: gripper open/close trigger input.
- `hat 0`: `x/y` left/right/forward/backward.
- `Cross`: defense position.
- `Circle`: attack right.
- `Triangle`: attack middle.
- `Square`: attack left.
- `L1` / `R1`: gripper open/close.
- `left center` / `right center`: short attack/defense sequence.
- `home button`: home position.

### `Controller-Direct.py`

- keyboard and gamepad control in direct joint/device angles
- useful when you want to bypass inverse kinematics and move each joint directly

Keyboard:

- `Left Arrow`: decrease base angle.
- `Right Arrow`: increase base angle.
- `Up Arrow`: increase shoulder angle.
- `Down Arrow`: decrease shoulder angle.
- `W`: increase elbow angle.
- `S`: decrease elbow angle.
- `A`: decrease gripper angle.
- `D`: increase gripper angle.

Joystick:

- `axis 0`: base angle.
- `axis 1`: shoulder angle.
- `axis 4`: elbow angle.
- `axis 2`: left trigger contribution to gripper opening.
- `axis 5`: right trigger contribution to gripper closing.
- `hat 0 left/right`: decrease/increase gripper angle.
- `Square`: move to or program the square waypoint.
- `X`: move to or program the X waypoint.
- `Circle`: move to or program the circle waypoint.
- `Triangle`: move to or program the triangle waypoint.
- `left center/share/select`: enter waypoint programming mode.
- `right center/options/start`: exit waypoint programming mode and return to run mode.

### `Controller-Stepper.py`

- controls two stepper motors on an Adafruit Motor HAT
- keyboard uses `Up/Down` for stepper 1 and `W/S` for stepper 2
- left and right joystick Y axes provide variable-speed stepper control

Keyboard:

- `Up Arrow` / `Down Arrow`: stepper 1 forward/backward.
- `W` / `S`: stepper 2 forward/backward.

Joystick:

- `axis 1` / left joystick up/down: stepper 1 variable speed.
- `axis 4` / right joystick up/down: stepper 2 variable speed.

### `Controller-Direct-wStepper.py`

- combines direct servo control and one M3/M4 stepper in one pygame program
- uses exclusive `SERVO`, `STEPPER`, and `IDLE` modes because the PCA9685 frequency cannot safely serve servos and steppers at the same time
- automatically enters stepper mode when `R1` or `L1` is pressed
- automatically returns to servo mode when servo joystick, gripper, waypoint, or programming controls are used
- keyboard `1`, `2`, and `0` remain available as fallback mode controls for servo, stepper, and idle/release
- servo mode uses the same direct joint, gripper, waypoint, and programming controls as `Controller-Direct.py`
- stepper mode uses `R1` for forward and `L1` for backward on the M3/M4 bridge through `MotorKit.stepper2`
- switching to stepper mode releases servo PWM outputs before initializing the stepper driver
- switching back to servo mode releases the stepper and restores the last saved servo angles

Keyboard:

- `1`: fallback switch to servo mode.
- `2`: fallback switch to stepper mode.
- `0`: idle mode and release active outputs.
- In servo mode, uses the same keyboard mappings as `Controller-Direct.py`.
- In stepper mode, `W` / `S` are a fallback for M3/M4 stepper forward/backward.

Joystick:

- `R1`: automatically enter stepper mode and run the M3/M4 stepper forward.
- `L1`: automatically enter stepper mode and run the M3/M4 stepper backward.
- Servo joystick axes, triggers, D-pad gripper controls, waypoint buttons, and programming buttons automatically return to servo mode.
- In servo mode, uses the same joystick mappings as `Controller-Direct.py`.

### `Zero.py`

- calibration utility for setting servo zero offsets and saving `mearm_config.json`
- also displays the configured min/max joint limits

### `moveMotor.py`

- direct pulse-width utility for servo setup and assembly checks
- useful when students are told an approximate pulse width for a target servo position

Example:

```bash
python3 moveMotor.py 1 2200
python3 moveMotor.py 14 1500 --address 0x6F
```

## Calibration

### moveMotor

You can adjust the meArm by seating the axle clip onto the motor:

- `python3 moveMotor.py 0 1500` should rotate the meArm to point forward
- `python3 moveMotor.py 1 1500` should rotate the shoulder so that it points upwards
- `python3 moveMotor.py 14 1500` should rotate the elbow to point horizontal
- `python3 moveMotor.py 15 2000` should close the gripper so that the fingers almost touch

### Zero

Run `Zero.py` to fine-calibrate the arm and save `mearm_config.json`.

When calibrating:

- base should point straight forward
- shoulder should point straight up
- elbow should be horizontal
- gripper should be in the intended reference position for your build

Do not force the servos beyond the mechanical limits of the meArm. If a horn is badly misaligned, remove it and reposition it on the spline instead of forcing the linkage.

## Gamepad Notes

The pygame-based controllers expect the gamepad to be connected before startup.

If the controller is unplugged and replugged while a program is running, the programs do not currently attempt to reconnect automatically.

## Notes

- The default I2C address depends on the script you run. Check the script before use and adjust if your board is strapped differently.
- Servo and stepper control are different hardware modes. Do not assume the same HAT/frequency setup is appropriate for both without checking your wiring and hardware configuration.
