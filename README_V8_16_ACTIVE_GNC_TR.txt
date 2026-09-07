TGY V8.16 - ESKF -> ACTIVE GNC BENCH INTEGRATION
==================================================

WHAT CHANGED
------------
The proven Full-State ESKF is now consumed by the control layer.

Control chain:

IMU / BARO / LIDAR
        |
        v
Full-State ESKF
  1000 Hz predict
  200 Hz correction/public state
        |
        +-----------------------------+
        |                             |
        v                             v
Roll/Pitch attitude control      Vertical landing control
        |                             |
        v                             v
RCS requested mask              Valve_Cmd
        |                             |
        v                             v
SolenoidOutput                  Needle controller 200 Hz
(DRY-RUN in V8.16)              REAL motor after manual arm

YAW
---
ESKF yaw is preserved/logged/diagnosed.

The existing AttitudeControl module is Roll + Pitch only.
No new yaw actuator law was invented in this version.

RCS OUTPUT POLICY - FIRST V8.16
-------------------------------
APP_GNC_RCS_DRY_RUN = 1

The COMPLETE attitude controller runs:
- phase-plane event logic
- landing prediction
- reversal deadtime
- chatter/event protection
- requested valve mask

But physical RCS GPIO is NOT energized yet.

This is intentional for the first estimator-to-controller validation.

Raw GNC diagnostic:
rcs_requested_mask may become nonzero.
rcs_applied_mask must remain 0 in dry-run.

After control-direction/log validation, set:
APP_GNC_RCS_DRY_RUN = 0

for a later physical-solenoid bench test with NO pressure.

NEEDLE OUTPUT
-------------
APP_GNC_NEEDLE_PHYSICAL_ENABLED = 1

The real needle motor is allowed only after the USER button sequence and the
existing zero/enable/fault protections pass.

Keep gas/high-pressure lines DISCONNECTED in this test.

USER BUTTON PA0
---------------
Press 1:
Capture needle CLOSED zero.

Press 2:
Enable needle controller.

Press 3:
ARM active GNC.

Press 4:
DISARM.
RCS goes safe immediately.
Needle is commanded to 0 and remains enabled until CLOSED/locked, then the
driver is disabled.

Any latched GNC fault:
next press acknowledges the GNC latch and returns to stage 0.

ARM REQUIREMENTS
----------------
The third press succeeds only when:
- ESKF enabled
- ESKF initialized
- ESKF healthy
- numerical_error_count = 0
- attitude state finite
- IMU gyro data valid
- origin_zeroed = 1
- vertical_position_valid = 1
- LIDAR fresh = 1
- LIDAR reference ready = 1
- needle zero_valid = 1
- needle enabled = 1
- needle fault = NONE

VERTICAL HEIGHT RECONSTRUCTION
------------------------------
The Full-State ESKF Z state is origin-relative.

V8.16 reconstructs height above the touchdown plane using:

height_agl =
    eskf.lidar_reference_m +
    eskf.position_z_m

Then the existing controller-family CG coordinate is:

z_cg =
    0.4013 +
    height_agl

This means:
h = z_cg - Z_TOUCH = height_agl

IMPORTANT:
This assumes LIDAR distance 0 corresponds to the touchdown plane.
If the LIDAR remains physically above ground at touchdown, calibrate that
mounting offset before flight.

VERTICAL CONTROLLER
-------------------
V8.16 uses the V10.6 actuator-aware early-brake controller already developed
for the measured needle dynamics.

Constants:
F_RATED            1120 N at 300 bar
model mass          27.5 kg
model pressure      300 bar
V_TOUCH             0.05 m/s
A_PROFILE           2.40 m/s^2
V_REFERENCE_MAX     5.40 m/s
K_V                 5.00
Valve_Cmd max       0.65

The local needle actuator still applies:
+1.10 /s opening slew
-1.00 /s closing slew
200 Hz closed-loop position control

VERY IMPORTANT:
Mass and pressure are currently CONFIG MODEL values because no live main tank
pressure/mass source is wired into GNCActiveControl yet.

Do not treat this V8.16 vertical thrust demand as flight-qualified until:
- live pressure is connected
- mass model is connected
- valve-position -> thrust map is measured
- pneumatic pressure dynamics are validated

ROLL/PITCH INPUT
----------------
Angles:
FullStateESKF roll_deg / pitch_deg

Body rates:
filtered IMU gyro X/Y
minus Full-State ESKF gyro bias X/Y

The source-folder axis/sign convention is preserved:
ROLL  = X-axis rotation
PITCH = Y-axis rotation
YAW   = Z-axis rotation

No sign swap was added.

TIMING
------
No new scheduler task was added.

1 kHz IMU task:
SensorManager_UpdateIMU
AttitudeEstimator_Update
FullStateESKF_Predict
GNCActiveControl_Service1kHz
SDLogger_PublishSources

200 Hz correction task:
FullStateESKF_CorrectMeasurements
GNCActiveControl_Update200Hz

Therefore the control layer consumes the same deterministic estimator clocks
already validated.

FIXED RAW MEMORY
----------------
Live Expressions is still not trusted.

Base estimator diagnostic:
0x1000F000

NEW V8.16 GNC diagnostic:
0x1000F100

GNC block = 128 bytes = 32 x 32-bit words.

First words:
0x1000F100  magic        expected 0x816C16D1
0x1000F104  version      expected 0x00081600
0x1000F108  block_size   expected 0x00000080
0x1000F10C  sequence

GNC MEMORY MAP
--------------
+0x00 magic
+0x04 version
+0x08 block_size
+0x0C sequence_begin

+0x10 flags
+0x14 button_stage
+0x18 fault_code
+0x1C attitude_controller_state

+0x20 rcs_requested_mask
+0x24 rcs_applied_mask
+0x28 needle_requested_x10000
+0x2C needle_limited_x10000

+0x30 vertical_cmd_x10000
+0x34 height_agl_mm
+0x38 z_cg_mm
+0x3C vertical_velocity_mmps

+0x40 roll_mdeg
+0x44 pitch_mdeg
+0x48 yaw_mdeg
+0x4C roll_rate_mdps
+0x50 pitch_rate_mdps

+0x54 vertical_reference_speed_mmps
+0x58 vertical_speed_error_mmps

+0x5C RCS roll predicted touchdown angle mdeg
+0x60 RCS pitch predicted touchdown angle mdeg

+0x64 needle raw ADC
+0x68 needle target ADC
+0x6C needle error ADC

+0x70 GNC update 200 Hz count
+0x74 GNC service 1 kHz count
+0x78 sequence_end
+0x7C checksum XOR

FLAGS +0x10
-----------
bit0  block alive
bit1  GNC armed
bit2  GNC fault latched
bit3  estimator OK
bit4  attitude state OK
bit5  vertical state OK
bit6  needle OK
bit7  disarm closing active
bit8  RCS dry-run
bit9  needle zero valid
bit10 needle enabled
bit11 needle fault none
bit12 attitude controller estimator healthy
bit13 attitude controller attitude fresh
bit14 landing prediction valid
bit15 attitude controller fault none

FIRST TEST
----------
NO PRESSURE / NO GAS.

1) Build / flash V8.16.
2) Do NOT press USER yet.
3) Let estimator settle 5-10 s.
4) Pause and read:
      0x1000F000
      0x1000F100

Expected before arm:
- GNC magic correct
- flags estimator/attitude/vertical OK
- armed bit = 0
- RCS requested = 0
- RCS applied = 0
- needle disabled until button sequence

5) Resume.
6) USER press 1 -> zero.
7) USER press 2 -> needle enable.
8) USER press 3 -> active GNC arm.

After arm:
- armed bit = 1
- button_stage = 3
- fault = 0
- rcs_applied_mask = 0 because dry-run
- needle controller may physically move only if vertical controller requests
  a nonzero command.

9) USER press 4 -> disarm.
Needle returns CLOSED before driver disable.

DO NOT CONNECT HIGH-PRESSURE GAS FOR THIS VERSION.
