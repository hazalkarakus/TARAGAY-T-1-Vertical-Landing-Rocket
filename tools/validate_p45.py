from pathlib import Path
import ast
import binascii
import importlib.util
import re

ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def load_monitor(relative: str):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location("p45_monitor", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


version = read("App/Common/app_version.h")
config = read("App/Common/app_config.h")
eskf_h = read("App/Modules/Estimation/FullStateESKF/full_state_eskf.h")
eskf = read("App/Modules/Estimation/FullStateESKF/full_state_eskf.c")
imu_h = read("App/Modules/Sensors/IMU/imu.h")
imu = read("App/Modules/Sensors/IMU/imu.c")
uart = read("App/Services/UARTTelemetry/uart_telemetry.c")
scheduler = read("App/Core/Scheduler/scheduler.c")
scheduler_h = read("App/Core/Scheduler/scheduler.h")
sd = read("App/Services/SDLogger/sd_logger.c")

# ---------------------------------------------------------------------------
# P45 identity + long-soak SD
# ---------------------------------------------------------------------------
assert "8.19M-P45-ESKF-GUARD-COV-RECOVERY" in version
for token in (
    "APP_SDLOGGER_PREALLOCATE_BYTES          (128UL * 1024UL * 1024UL)",
    "APP_SDLOGGER_PREALLOCATE_MIN_BYTES      (96UL * 1024UL * 1024UL)",
    "APP_FULL_ESKF_COV_SYMMETRY_ABS_TOL",
    "APP_FULL_ESKF_COV_SYMMETRY_REL_TOL",
    "APP_FULL_ESKF_PUBLIC_GUARD_MAX_GAP_US",
    "APP_FULL_ESKF_PUBLIC_Z_JUMP_BASE_M",
    "APP_FULL_ESKF_PUBLIC_VZ_JUMP_BASE_MPS",
):
    assert token in config, token
assert "(128UL * 1024UL * 1024UL)" in sd
assert "(96UL * 1024UL * 1024UL)" in sd

# ---------------------------------------------------------------------------
# P44 scheduler/LiDAR timing invariants stay frozen.
# ---------------------------------------------------------------------------
assert re.search(
    r"scheduler_priority_order\[TASK_COUNT\].*?0U,\s*4U,\s*6U,\s*2U,\s*1U,\s*3U,\s*5U",
    scheduler,
    re.S,
)
assert "case 6UL: return APP_P44_COVARIANCE_SLOT_RESERVE_US;" in scheduler
assert "LIDAR_SERVICE_1KHZ_BEST_EFFORT" in scheduler
assert "lidar_scheduler_best_effort_skip_count" in scheduler
assert "extern volatile uint32_t lidar_scheduler_best_effort_skip_count" in scheduler_h

# ---------------------------------------------------------------------------
# ESKF: explicit reset reasons, covariance integrity and guard-before-publish.
# ---------------------------------------------------------------------------
for token in (
    "FULL_ESKF_RESET_REASON_STATE_NUMERICAL",
    "FULL_ESKF_RESET_REASON_COV_NONFINITE",
    "FULL_ESKF_RESET_REASON_COV_DIAGONAL",
    "FULL_ESKF_RESET_REASON_COV_ASYMMETRY",
    "FULL_ESKF_RESET_REASON_PUBLIC_Z_JUMP",
    "FULL_ESKF_RESET_REASON_PUBLIC_VZ_JUMP",
    "FULL_ESKF_RESET_REASON_PUBLIC_Z_VZ_JUMP",
    "covariance_integrity_check_count",
    "covariance_fault_count",
    "covariance_reinit_count",
    "public_output_reject_count",
):
    assert token in eskf_h, token

for token in (
    "static uint8_t FullESKF_CheckCovarianceIntegrity(void)",
    "static void FullESKF_SafeReinitialize(uint8_t reason)",
    "static uint8_t FullESKF_CheckPublicVerticalGuard(uint32_t now_us)",
    "APP_FULL_ESKF_COV_SYMMETRY_ABS_TOL",
    "APP_FULL_ESKF_COV_SYMMETRY_REL_TOL",
    "covariance_sanitization_fault_pending",
):
    assert token in eskf, token

correct_start = eskf.index("void FullStateESKF_CorrectMeasurements(void)")
correct_end = eskf.index("FullStateESKFData_t FullStateESKF_GetData", correct_start)
correct = eskf[correct_start:correct_end]
assert "FullESKF_CheckNumericalHealth()" in correct
assert "FullESKF_CheckCovarianceIntegrity()" in correct
assert "FullESKF_CheckPublicVerticalGuard(public_now_us)" in correct
assert "FullESKF_SafeReinitialize" in correct
assert correct.index("FullESKF_CheckPublicVerticalGuard(public_now_us)") < correct.index(
    "FullESKF_UpdatePublicState();"
)

cov_service_start = eskf.index("uint8_t FullStateESKF_ServiceCovariance(void)")
cov_service_end = eskf.index("void FullStateESKF_CorrectMeasurements", cov_service_start)
cov_service = eskf[cov_service_start:cov_service_end]
assert "FullESKF_CheckCovarianceIntegrity()" in cov_service
assert "FullESKF_SafeReinitialize(covariance_reason)" in cov_service

safe_start = eskf.index("static void FullESKF_SafeReinitialize(uint8_t reason)")
safe_end = eskf.index("static uint8_t FullESKF_CheckPublicVerticalGuard", safe_start)
safe = eskf[safe_start:safe_end]
for token in (
    "numerical_error_count",
    "public_output_count",
    "last_public_output_timestamp_us",
    "public_output_reject_count",
    "covariance_integrity_check_count",
    "covariance_fault_count",
    "covariance_reinit_count",
    "eskf_data.reset_reason = reason",
):
    assert token in safe, token

# ---------------------------------------------------------------------------
# IMU: exactly one soft resync; second bad sample arms full non-blocking recovery.
# ---------------------------------------------------------------------------
assert "IMU_GetPatternRecoveryEscalationCount" in imu_h
assert "imu_pattern_recovery_escalation_count" in imu
assert re.search(
    r"IMU_SPI_ResynchronizeBus\(\);\s*imu_pattern_retry_pending\s*=\s*1U;\s*imu_pattern_retry_count\+\+",
    imu,
)
assert len(re.findall(r"imu_pattern_recovery_escalation_count\+\+;\s*IMU_StartRecovery\(micros\(\)\);", imu)) >= 2

resync_service = imu.split("static void IMU_SPI_ResynchronizeBus(void)", 2)[2].split(
    "static inline uint8_t IMU_SW_SPI_Byte", 1
)[0]
assert "HAL_Delay" not in resync_service

recovery_service = imu.split("static void IMU_ServiceRecovery(uint32_t now_us)", 2)[2].split(
    "/* -------------------------------------------------------------------------- */\n/* Public API", 1
)[0]
assert "HAL_Delay" not in recovery_service

# ---------------------------------------------------------------------------
# Safety architecture remains in place.
# ---------------------------------------------------------------------------
for relative in (
    "App/Services/SolenoidOutput/solenoid_output.c",
    "App/Modules/Control/AttitudeControl/attitude_control.c",
    "App/Core/Tasks/app_tasks.c",
):
    assert "SystemMonitor_IsActuatorFaultActive()" in read(relative), relative

# ---------------------------------------------------------------------------
# UART TGY56: C emission count/header and Python decoder must agree.
# ---------------------------------------------------------------------------
assert 'UARTTelemetry_AppendText(&position, "$TGY56")' in uart
assert "FLIGHT DIAGNOSTICS V56" in uart
for token in (
    "imu_pattern_recovery_escalations",
    "eskf_reset_reason",
    "eskf_num_errors",
    "eskf_public_rejects",
    "eskf_cov_ok",
    "eskf_cov_faults",
    "eskf_cov_reinits",
    "eskf_cov_sym_max_u1e6",
):
    assert token in uart, token

build_start = uart.index("static uint8_t UARTTelemetry_BuildDataLine")
build_end = uart.index("void UARTTelemetry_Update10Hz", build_start)
build = uart[build_start:build_end]
emit_start = build.index('UARTTelemetry_AppendText(&position, "$TGY56")')
emissions = re.findall(r"\bFIELD_(?:U32|I32)\s*\(", build[emit_start:])
assert len(emissions) + 1 == 228

header_start = uart.index("static const char header[] =", build_end)
header_end = uart.index(";", header_start)
header_literals = re.findall(r'"(?:\\.|[^"\\])*"', uart[header_start:header_end])
header = "".join(ast.literal_eval(value) for value in header_literals)
header_fields = header.strip().lstrip("# ").split(",")
assert len(header_fields) == 229
assert header_fields[-1] == "crc16_ccitt"
assert len(set(header_fields)) == len(header_fields)

monitor = load_monitor("monitor_uart_p45.py")
assert len(monitor.FIELDS) == 228
assert len(set(monitor.FIELDS)) == 228
assert tuple(header_fields[:-1]) == monitor.FIELDS

# Synthetic valid 228-field frame + CRC roundtrip.
values = ["$TGY56"] + [str(i) for i in range(1, len(monitor.FIELDS))]
body = ",".join(values)
crc = binascii.crc_hqx(body.encode("ascii"), 0xFFFF)
decoded = monitor.decode_frame(f"{body}*{crc:04X}")
assert len(decoded) == 228

# Historical P44 protocol + V14 decoder are left untouched.
assert read("decode_flight_v14_p44.py") == read("tools/decode_flight_v14.py")

print(
    "P45 validation: PASS "
    "(ESKF guard-before-publish; covariance safe reacquire; "
    "IMU second-bad escalation; TGY56/228; SD 128/96 MiB)"
)
