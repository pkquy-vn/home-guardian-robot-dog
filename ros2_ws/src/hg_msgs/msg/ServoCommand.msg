# Servo calibration and single-joint test command, handled by teensy_bridge (hg_teensy_bridge).
# The result of every command is published as text on /teensy/command_result.

uint8 CALIBRATE = 1        # Store the joint's current servo position as its reference pose
                           # (hip 0 deg, knee interior angle 90 deg). The joint must be limp.
uint8 FLIP_DIRECTION = 2   # Invert the joint's direction sign.
uint8 RESTORE = 3          # Re-send the calibration copy saved on the Pi to the Teensy.
uint8 SET_ANGLE = 4        # Drive the joint to `value` degrees until the next mode command.
uint8 SET_SPEED_LIMIT = 5  # Maximum joint speed: `value` in degrees per second. `joint` unused.

uint8 ALL_JOINTS = 255     # For RESTORE: every joint in the saved copy.

uint8 command
uint8 joint                # 0..7: FL hip, FL knee, FR hip, FR knee, RL hip, RL knee, RR hip, RR knee
float32 value
