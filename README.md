# Motor Control Codes for HOMKAS Drone

This repository contains Python-based test scripts for controlling drone motors via RC override and MAV_CMD_DO_MOTOR_TEST using MAVLink protocol.

## Code List:
- `controlled_throttle_test.py` – Smooth throttle ramp-up & down
- `emergency_disarm.py` – Emergency disarm function
- `multi_motor_ramp_test.py` – Tests multiple motors with ramp
- `rc_override_emergency.py` – RC override with failsafe
- `safe_throttle_with_fail_safe.py` – Safe ramping with emergency logic
- `motor_debug_suite.py` – Diagnostic logs for system state
- `single_motor_ramp_test.py` – Individual motor test with controlled ramp
- `sequential_motor_test.py` – Motor 1-4 tested sequentially
- `rc_throttle_ramp.py` – Basic throttle ramp via RC override
- `full_motor_test_suite.py` – Combined test suite with failsafe

## Usage
Use with Raspberry Pi + pymavlink to send RC override or motor test commands.
