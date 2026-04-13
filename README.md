meArmPi
=======

Python control tools for the Phenoptix meArm on Raspberry Pi using Adafruit's CircuitPython motor libraries.

This repository contains:

- `meArm.py`: inverse-kinematics and direct joint-angle control for the 4-servo meArm
- `Controller.py`: keyboard and gamepad controller in Cartesian `x/y/z` space
- `Controller-Direct.py`: keyboard and gamepad controller in direct joint/device angles
- `Controller-Stepper.py`: keyboard and joystick controller for 2 steppers on an Adafruit Motor HAT
- `Zero.py`: calibration tool for servo zero offsets and joint limits
- `moveMotor.py`: simple pulse-width utility for servo setup during assembly
- `mearm_config.json`: saved zero offsets and per-joint angle limits

Hardware
--------

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

Wiring
------

The servo arm is driven from a PCA9685-based servo board. In the current code the channels are:

- channel `0`: base
- channel `1`: shoulder
- channel `14`: elbow
- channel `15`: gripper

This mapping is used in both [`meArm.py`](./meArm.py) and [`Zero.py`](./Zero.py).

Project Overview
----------------

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

Configuration Model
-------------------

Each joint in `mearm_config.json` has:

- `zero`: servo command angle used as the calibrated zero offset
- `min_deg`: minimum joint/device angle
- `max_deg`: maximum joint/device angle

Important distinction:

- joint/device angles are the physical arm angles used by `meArm.py`
- servo command angles are the raw `0..180` values sent to the Adafruit servo API
- pulse width in microseconds is the low-level PWM representation used by `moveMotor.py`

Installation
------------

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

Core Library Usage
------------------

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

Current `meArm` Methods
-----------------------

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

Included Programs
-----------------

`Controller.py`

- keyboard and gamepad control in Cartesian space
- arrow keys and `w/s` move the arm in `x/y/z`
- gripper is controlled as open percentage

`Controller-Direct.py`

- keyboard and gamepad control in direct joint/device angles
- useful when you want to bypass inverse kinematics and move each joint directly

`Controller-Stepper.py`

- controls two stepper motors on an Adafruit Motor HAT
- keyboard uses `Up/Down` for stepper 1 and `W/S` for stepper 2
- left and right joystick Y axes provide variable-speed stepper control

`Zero.py`

- calibration utility for setting servo zero offsets and saving `mearm_config.json`
- also displays the configured min/max joint limits

`moveMotor.py`

- direct pulse-width utility for servo setup and assembly checks
- useful when students are told an approximate pulse width for a target servo position

Example:

```bash
python3 moveMotor.py 1 2200
python3 moveMotor.py 14 1500 --address 0x6F
```

Calibration
-----------

Run `Zero.py` to calibrate the arm and save `mearm_config.json`.

When calibrating:

- base should point straight forward
- shoulder should point straight up
- elbow should be horizontal
- gripper should be in the intended reference position for your build

Do not force the servos beyond the mechanical limits of the meArm. If a horn is badly misaligned, remove it and reposition it on the spline instead of forcing the linkage.

Gamepad Notes
-------------

The pygame-based controllers expect the gamepad to be connected before startup.

If the controller is unplugged and replugged while a program is running, the programs do not currently attempt to reconnect automatically.

Notes
-----

- The default I2C address depends on the script you run. Check the script before use and adjust if your board is strapped differently.
- Servo and stepper control are different hardware modes. Do not assume the same HAT/frequency setup is appropriate for both without checking your wiring and hardware configuration.
