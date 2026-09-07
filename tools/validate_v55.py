#!/usr/bin/env python3
"""Static and protocol checks for the V55 flight-interlock build."""

from __future__ import annotations

import ast
import binascii
import importlib.util
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8", errors="strict")


def require(text: str, pattern: str, label: str) -> None:
    if re.search(pattern, text, flags=re.MULTILINE | re.DOTALL) is None:
        raise AssertionError(label)


config = read("App/Common/app_config.h")
preflight = read("App/Services/PreflightTrigger/preflight_trigger.c")
solenoid = read("App/Services/SolenoidOutput/solenoid_output.c")
needle = read("App/Modules/Control/NeedleValve/needle_valve_controller.c")
uart = read("App/Services/UARTTelemetry/uart_telemetry.c")
tasks = read("App/Core/Tasks/app_tasks.c")
scheduler = read("App/Core/Scheduler/scheduler.c")
app = read("App/app.c")
sensor_manager = read("App/Modules/Sensors/SensorManager/sensor_manager.c")
lidar = read("App/Modules/Sensors/Lidar/Lidar.c")
full_eskf = read("App/Modules/Estimation/FullStateESKF/full_state_eskf.c")
system_monitor = read("App/Services/SystemMonitor/system_monitor.c")
system_monitor_h = read("App/Services/SystemMonitor/system_monitor.h")

require(config, r"APP_PREFLIGHT_PE9_CONNECTED_IS_LOW\s+1U", "PE9 polarity")
require(config, r"APP_ACTUATOR_FLIGHT_INTERLOCK_ENABLED\s+1U", "actuator gate")
require(config, r"APP_NEEDLE_AUTO_CONTROL_MODE\s+0U", "endurance disabled")
require(config, r"APP_GENERATED_FC_PHYSICAL_OUTPUT_ENABLED\s+1U", "main route")
require(config, r"APP_FULL_ESKF_COVARIANCE_DECIMATION\s+32U", "P39 covariance readiness")
require(config, r"APP_TASK_FULL_ESKF_COVARIANCE_PERIOD_US\s+40000UL", "25 Hz covariance scheduler")
require(config, r"APP_FULL_ESKF_EULER_DECIMATION\s+5U", "200 Hz ESKF output")

require(preflight, r"GPIO_PULLUP", "PE9 pull-up")
require(preflight, r"GPIO_PIN_SET\) \? 1U : 0U", "PE9 open semantics")
require(preflight, r"NeedleValveController_CaptureZero\(\)", "safe zero capture")

for port, pin in (
    ("SOLENOID_IN1_X_POS_PORT", "GPIOB"),
    ("SOLENOID_IN1_X_POS_PIN", "GPIO_PIN_15"),
    ("SOLENOID_IN2_X_NEG_PORT", "GPIOE"),
    ("SOLENOID_IN2_X_NEG_PIN", "GPIO_PIN_15"),
    ("SOLENOID_IN3_Y_POS_PORT", "GPIOE"),
    ("SOLENOID_IN3_Y_POS_PIN", "GPIO_PIN_11"),
    ("SOLENOID_IN4_Y_NEG_PORT", "GPIOE"),
    ("SOLENOID_IN4_Y_NEG_PIN", "GPIO_PIN_7"),
):
    require(solenoid, rf"#define\s+{port}\s+{pin}\b", f"mapping {port}")

require(solenoid, r"PreflightTrigger_IsFlightActive\(\)", "RCS flight gate")
require(solenoid, r"applied_candidate = SOLENOID_VALVE_NONE", "RCS inhibit")
require(needle, r"NV_ComputeDampedPwm", "damped needle controller")
require(needle, r"signed_effort\s*=.*error.*derivative", "signed D term")
require(needle, r"NV_RunPulse\(1U, pwm\)", "near-target open pulse")
require(needle, r"NV_RunPulse\(2U, pwm\)", "near-target close pulse")
require(needle, r"NV_TRAVEL_GUARD_ADC", "four-turn travel guard")
require(needle, r"PreflightTrigger_IsFlightActive\(\)", "needle flight gate")

require(tasks, r"V55_ApplyPhysicalNeedleOutput", "physical needle route")
require(tasks, r"SystemMonitor_Update\(\)", "system monitor service")
require(uart, r"\$TGY55", "V55 frame prefix")
require(uart, r"now_us = micros\(\)", "single timebase age")
require(scheduler, r"next_run_us\s*=\s*now\s*\+\s*tasks\[i\]\.period_us",
        "scheduler late-release realignment")
require(scheduler, r"scheduler_release_realign_count\+\+",
        "scheduler realignment diagnostics")
require(app, r"SensorManager_ServiceFreshness\(\).*PreflightTrigger_Update\(\)",
        "independent IMU freshness guard")
require(sensor_manager,
        r"sensor_imu_sample_age_us\s*>\s*APP_SENSOR_MANAGER_IMU_VALID_TIMEOUT_US",
        "stale IMU invalidation")
require(sensor_manager, r"sensor_data\.timestamp_us\s*=\s*sample_timestamp_us",
        "IMU sample timestamp propagation")
require(lidar, r"LIDAR_WAIT_BUSY_TIMEOUT_MS", "LiDAR WAIT/HAL_BUSY timeout")
require(lidar, r"wait_busy_timeout_count\+\+.*Lidar_RequestRecovery\(",
        "LiDAR WAIT/HAL_BUSY non-blocking recovery")
require(lidar, r"LIDAR_RECOVERY_STEP_DMA_WAIT", "LiDAR DMA-stop recovery step")
require(full_eskf,
        r"last_public_output_timestamp_us\s*=\s*micros\(\)",
        "ESKF public-output timestamp")
for fault in ("IMU_STALE", "LIDAR_STALE", "ESKF_STALE"):
    require(system_monitor_h, rf"SYS_FAULT_{fault}\b", f"system fault {fault}")
require(system_monitor, r"APP_SYSTEM_MONITOR_IMU_STALE_US", "IMU freeze monitor")
require(system_monitor, r"APP_SYSTEM_MONITOR_LIDAR_STALE_US", "LiDAR freeze monitor")
require(system_monitor, r"APP_SYSTEM_MONITOR_ESKF_PUBLIC_STALE_US",
        "ESKF freeze monitor")

start = uart.index("static uint8_t UARTTelemetry_BuildDataLine")
end = uart.index("void UARTTelemetry_Update10Hz", start)
builder = uart[start:end]
c_field_count = (
    1
    + builder.count("FIELD_U32(") - 1
    + builder.count("FIELD_I32(") - 1
)

monitor_path = ROOT / "monitor_uart_v55.py"
tree = ast.parse(monitor_path.read_text(encoding="utf-8"))
monitor_fields = None
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id == "FIELDS"
        for target in node.targets
    ):
        monitor_fields = ast.literal_eval(node.value)
        break

if monitor_fields is None:
    raise AssertionError("monitor FIELDS missing")
if c_field_count != len(monitor_fields):
    raise AssertionError(
        f"UART field mismatch: firmware={c_field_count} monitor={len(monitor_fields)}"
    )
if len(set(monitor_fields)) != len(monitor_fields):
    raise AssertionError("duplicate UART field name")

header_start = uart.index("static const char header[]")
header_end = uart.index("uint32_t now_ms", header_start)
header_text = "".join(re.findall(r'"([^"]*)"', uart[header_start:header_end]))
header_text = header_text.replace("\\r", "").replace("\\n", "")
if not header_text.startswith("# "):
    raise AssertionError("UART CSV header missing")
header_fields = header_text[2:].split(",")
if header_fields[:-1] != list(monitor_fields):
    raise AssertionError("firmware header and monitor field order differ")
if header_fields[-1] != "crc16_ccitt":
    raise AssertionError("UART CRC header missing")

spec = importlib.util.spec_from_file_location("monitor_uart_v55", monitor_path)
if spec is None or spec.loader is None:
    raise AssertionError("monitor import failed")
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)
values = ["$TGY55"] + ["0"] * (len(monitor_fields) - 1)
body = ",".join(values)
crc = binascii.crc_hqx(body.encode("ascii"), 0xFFFF)
decoded = monitor.decode_frame(f"{body}*{crc:04X}")
if len(decoded) != c_field_count:
    raise AssertionError("synthetic V55 frame decode failed")

print(f"V55 validation: PASS ({c_field_count} UART fields)")
