`Controller-Direct.py` implementation status

Utilized controls

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

Completed

0. Preserve backward compatibility in `meArm.py`.
   Status: completed
   `Controller.py` still keeps the inverse-kinematics path (`move_to`, `move_linear`, `get_position`, `get_finger`, `partial_grip`).
   Direct-angle support was added as new API in `meArm.py`, not as a replacement.

1. Define the new control state as joint/device angles, not Cartesian position.
   Status: completed
   `Controller-Direct.py` now tracks `base`, `shoulder`, `elbow`, and `gripper` angles instead of `x`, `y`, `z`, and `finger`.

2. Add direct joint-motion support in `meArm.py`.
   Status: completed
   Added:
   `set_joint_angles(base, shoulder, elbow)`
   `set_gripper_angle(gripper)`
   `get_joint_angles()`
   `get_gripper_angle()`

3. Make `meArm.py` store current joint/device angles.
   Status: completed
   `meArm.py` now maintains joint/device-angle state alongside Cartesian state.

4. Replace Cartesian keyboard actions with joint actions in `Controller-Direct.py`.
   Status: completed
   Implemented mapping:
   `Left/Right` -> `BASE_DECREASE/INCREASE`
   `Up/Down` -> `SHOULDER_DECREASE/INCREASE`
   `W/S` -> `ELBOW_INCREASE/DECREASE`
   `A/D` -> `GRIPPER_DECREASE/INCREASE`

5. Replace joystick axis handling so each axis drives one joint directly.
   Status: completed
   Implemented mapping:
   `axis 0` -> base
   `axis 1` -> shoulder
   `axis 4` -> elbow
   `axes 2/5` triggers -> gripper
   `hat 0 left/right` -> gripper decrease/increase

6. Remove inverse-kinematics and Cartesian move calls from the controller path.
   Status: completed
   `Controller-Direct.py` no longer uses `move_to(x, y, z)` for normal motion.
   It now sends desired joint/device angles to `meArm`, and `meArm` performs zero-offset conversion and clamping.

7. Remove gamepad button and old hat motion logic.
   Status: completed
   `JOYBUTTON`, old `JOYHAT`, attack/defend logic, and timed reverse-move sequences were removed from `Controller-Direct.py`.
   A new direct `hat 0` polling path now uses D-pad left/right for gradual gripper control.

8. Update the UI/status text.
   Status: completed
   The UI now shows `Base`, `Shoulder`, `Elbow`, and `Gripper` angles plus the new keyboard/gamepad help text.

9. Verify limits and startup behavior.
   Status: partially completed
   Startup now initializes from `meArm` joint/device-angle state.
   Joint/device angles are clamped to `mearm_config.json`.
   Raw servo commands are still clamped to `0..180`.
   Remaining work: confirm on real hardware that the configured limits and axis directions behave as intended.

10. Run a non-hardware verification pass.
   Status: completed
   `python3 -m py_compile Controller-Direct.py meArm.py` passed.
   Remaining work: live motion test on the arm.

11. Add direct-controller waypoints.
   Status: completed
   `controller_direct_waypoints.json` stores four direct-angle waypoints: `square`, `x`, `circle`, and `triangle`.
   In run mode, pressing a face button loads its saved `base`, `shoulder`, `elbow`, and `gripper` angles.
   In programming mode, pressing a face button saves the current `base`, `shoulder`, `elbow`, and `gripper` angles to that waypoint.
   The left center/share/select button enters programming mode.
   The right center/options/start button exits programming mode.

Notes

- The direct controller uses joint/device angles, not raw servo command angles.
- `meArm.py` now has one shared path for clamping, zero-offset conversion, servo command generation, and state synchronization.
- Hardware validation is still required for final sign-off on axis direction, gripper trigger feel, and practical limit behavior.
