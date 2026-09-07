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
    spec = importlib.util.spec_from_file_location("p46_monitor", path)
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
# P46 identity + frozen P45/P44 safety/timing baseline.
# ---------------------------------------------------------------------------
assert "8.19M-P46-COV-ROOTCAUSE-STATE-PRESERVE" in version
for token in (
    "APP_SDLOGGER_PREALLOCATE_BYTES          (128UL * 1024UL * 1024UL)",
    "APP_SDLOGGER_PREALLOCATE_MIN_BYTES      (96UL * 1024UL * 1024UL)",
    "APP_FULL_ESKF_COV_SYMMETRY_ABS_TOL",
    "APP_FULL_ESKF_COV_SYMMETRY_REL_TOL",
    "APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL       1.0e-7f",
    "APP_FULL_ESKF_PUBLIC_GUARD_MAX_GAP_US",
    "APP_FULL_ESKF_PUBLIC_Z_JUMP_BASE_M",
    "APP_FULL_ESKF_PUBLIC_VZ_JUMP_BASE_MPS",
):
    assert token in config, token
assert "(128UL * 1024UL * 1024UL)" in sd
assert "(96UL * 1024UL * 1024UL)" in sd

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
# Root-cause diagnostics are explicit and persistent.
# ---------------------------------------------------------------------------
for token in (
    "FULL_ESKF_COV_STAGE_NONE",
    "FULL_ESKF_COV_STAGE_PROPAGATE",
    "FULL_ESKF_COV_STAGE_GRAVITY",
    "FULL_ESKF_COV_STAGE_ZUPT",
    "FULL_ESKF_COV_STAGE_GYRO_BIAS",
    "FULL_ESKF_COV_STAGE_ACCEL_BIAS",
    "FULL_ESKF_COV_STAGE_BARO",
    "FULL_ESKF_COV_STAGE_LIDAR",
    "FULL_ESKF_COV_STAGE_INTEGRITY",
    "FULL_ESKF_COV_STAGE_GAP_INFLATE",
    "FULL_ESKF_COV_STATE_INVALID",
    "covariance_fault_stage",
    "covariance_fault_state_index",
    "covariance_fault_other_index",
    "covariance_fault_used_last_good",
    "covariance_fault_raw_value",
    "covariance_fault_aux_value",
    "covariance_fault_timestamp_us",
    "covariance_roundoff_last_stage",
    "covariance_roundoff_last_state_index",
    "covariance_roundoff_last_raw_value",
    "covariance_roundoff_last_timestamp_us",
    "covariance_state_preserving_recovery_count",
    "covariance_roundoff_clamp_count",
):
    assert token in eskf_h, token

for token in (
    "covariance_last_good[FULL_ESKF_STATE_COUNT][FULL_ESKF_STATE_COUNT]",
    "covariance_last_good_valid",
    "FullESKF_SaveGoodCovariance",
    "FullESKF_LatchCovarianceFault",
    "FullESKF_RecoverCovarianceOnly",
    "APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL",
):
    assert token in eskf, token

# Tiny negative roundoff is distinguished from structural covariance failure.
assert "static void FullESKF_RecordCovarianceRoundoff(" in eskf
roundoff_helper = eskf[
    eskf.index("static void FullESKF_RecordCovarianceRoundoff("):
    eskf.index("static void FullESKF_SymmetrizeAndClampCovariance",
               eskf.index("static void FullESKF_RecordCovarianceRoundoff("))
]
for token in (
    "covariance_roundoff_clamp_count++",
    "covariance_roundoff_last_stage = stage",
    "covariance_roundoff_last_state_index = state_index",
    "covariance_roundoff_last_raw_value = raw_value",
    "covariance_roundoff_last_timestamp_us = micros()",
):
    assert token in roundoff_helper, token
assert re.search(
    r"raw_diagonal\s*>=\s*-APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL.*?"
    r"FullESKF_RecordCovarianceRoundoff",
    eskf,
    re.S,
)
assert re.search(
    r"else\s*\{\s*FullESKF_LatchCovarianceFault\(\s*"
    r"FULL_ESKF_RESET_REASON_COV_DIAGONAL",
    eskf,
    re.S,
)

# ---------------------------------------------------------------------------
# Covariance-only recovery: restore P, preserve nominal/public state.
# ---------------------------------------------------------------------------
recover_start = eskf.index("static void FullESKF_RecoverCovarianceOnly(uint8_t reason)")
recover_end = eskf.index("static void FullESKF_SafeReinitialize", recover_start)
recover = eskf[recover_start:recover_end]
for token in (
    "memcpy(covariance, covariance_last_good, sizeof(covariance))",
    "FullESKF_SetInitialCovariance()",
    "FullESKF_ClearCovarianceAccumulator()",
    "covariance_state_preserving_recovery_count++",
    "covariance_reinit_count++",
    "eskf_data.healthy = 1U",
):
    assert token in recover, token
assert "FullStateESKF_Init()" not in recover
assert "nominal_position" not in recover
assert "nominal_velocity" not in recover
assert "nominal_quaternion" not in recover
assert "public_output_count" not in recover

# A complete integrity PASS becomes the rollback checkpoint.
check_start = eskf.index("static uint8_t FullESKF_CheckCovarianceIntegrity(void)")
check_end = eskf.index("static void FullESKF_RecoverCovarianceOnly", check_start)
check = eskf[check_start:check_end]
assert "FullESKF_SaveGoodCovariance();" in check
assert check.index("if (reason == FULL_ESKF_RESET_REASON_NONE)") < check.index(
    "FullESKF_SaveGoodCovariance();"
)

# Covariance fault paths use rollback; state/public numerical guards keep full reacquire.
cov_service_start = eskf.index("uint8_t FullStateESKF_ServiceCovariance(void)")
cov_service_end = eskf.index("void FullStateESKF_CorrectMeasurements", cov_service_start)
cov_service = eskf[cov_service_start:cov_service_end]
assert "FullESKF_RecoverCovarianceOnly(covariance_reason)" in cov_service
assert "FullESKF_SafeReinitialize(covariance_reason)" not in cov_service

correct_start = eskf.index("void FullStateESKF_CorrectMeasurements(void)")
correct_end = eskf.index("FullStateESKFData_t FullStateESKF_GetData", correct_start)
correct = eskf[correct_start:correct_end]
assert correct.count("FullESKF_RecoverCovarianceOnly(covariance_reason)") >= 2
assert "FullESKF_SafeReinitialize(FULL_ESKF_RESET_REASON_STATE_NUMERICAL)" in correct
assert "FullESKF_CheckPublicVerticalGuard(public_now_us)" in correct
assert correct.index("FullESKF_CheckPublicVerticalGuard(public_now_us)") < correct.index(
    "FullESKF_UpdatePublicState();"
)

# An update that creates a structural covariance fault cannot inject its error state.
for function_name in (
    "FullESKF_ScalarUpdateUnitState",
    "FullESKF_ScalarUpdateTwoStates",
    "FullESKF_ConstrainedSingleStateUpdate",
    "FullESKF_ConstrainedAltitudeUpdate",
):
    if function_name not in eskf:
        continue
    start = eskf.index("static uint8_t " + function_name)
    # enough to span each function without depending on the next function name
    fragment = eskf[start:start + 9000]
    clamp = fragment.index("FullESKF_ClampCovarianceDiagonal(covariance_stage)")
    pending = fragment.index("covariance_sanitization_fault_pending", clamp)
    inject = fragment.index("FullESKF_InjectError", pending)
    assert clamp < pending < inject

# Stage labels are actually supplied at the measurement/propagation call sites.
for token in (
    "FULL_ESKF_COV_STAGE_PROPAGATE",
    "FULL_ESKF_COV_STAGE_GRAVITY",
    "FULL_ESKF_COV_STAGE_ZUPT",
    "FULL_ESKF_COV_STAGE_GYRO_BIAS",
    "FULL_ESKF_COV_STAGE_ACCEL_BIAS",
    "FULL_ESKF_COV_STAGE_BARO",
    "FULL_ESKF_COV_STAGE_LIDAR",
    "FULL_ESKF_COV_STAGE_GAP_INFLATE",
):
    assert eskf.count(token) >= 1, token

# ---------------------------------------------------------------------------
# P45 IMU full non-blocking recovery remains frozen.
# ---------------------------------------------------------------------------
assert "IMU_GetPatternRecoveryEscalationCount" in imu_h
assert "imu_pattern_recovery_escalation_count" in imu
assert re.search(
    r"IMU_SPI_ResynchronizeBus\(\);\s*imu_pattern_retry_pending\s*=\s*1U;\s*imu_pattern_retry_count\+\+",
    imu,
)
assert len(re.findall(
    r"imu_pattern_recovery_escalation_count\+\+;\s*IMU_StartRecovery\(micros\(\)\);",
    imu,
)) >= 2

resync_service = imu.split("static void IMU_SPI_ResynchronizeBus(void)", 2)[2].split(
    "static inline uint8_t IMU_SW_SPI_Byte", 1
)[0]
assert "HAL_Delay" not in resync_service
recovery_service = imu.split("static void IMU_ServiceRecovery(uint32_t now_us)", 2)[2].split(
    "/* -------------------------------------------------------------------------- */\n/* Public API", 1
)[0]
assert "HAL_Delay" not in recovery_service

# Safety architecture remains in place.
for relative in (
    "App/Services/SolenoidOutput/solenoid_output.c",
    "App/Modules/Control/AttitudeControl/attitude_control.c",
    "App/Core/Tasks/app_tasks.c",
):
    assert "SystemMonitor_IsActuatorFaultActive()" in read(relative), relative

# ---------------------------------------------------------------------------
# UART TGY57: C emission/header/Python monitor must agree at 237 data fields.
# ---------------------------------------------------------------------------
assert 'UARTTelemetry_AppendText(&position, "$TGY57")' in uart
assert "FLIGHT DIAGNOSTICS V57" in uart
for token in (
    "eskf_cov_fault_stage",
    "eskf_cov_fault_state",
    "eskf_cov_fault_other",
    "eskf_cov_fault_raw_n1e9",
    "eskf_cov_fault_aux_n1e9",
    "eskf_cov_fault_time_ms",
    "eskf_cov_rollback_last_good",
    "eskf_cov_rollbacks",
    "eskf_cov_roundoff_clamps",
    "eskf_cov_roundoff_stage",
    "eskf_cov_roundoff_state",
    "eskf_cov_roundoff_raw_n1e9",
    "eskf_cov_roundoff_time_ms",
):
    assert token in uart, token

build_start = uart.index("static uint8_t UARTTelemetry_BuildDataLine")
build_end = uart.index("void UARTTelemetry_Update10Hz", build_start)
build = uart[build_start:build_end]
emit_start = build.index('UARTTelemetry_AppendText(&position, "$TGY57")')
emissions = re.findall(r"\bFIELD_(?:U32|I32)\s*\(", build[emit_start:])
assert len(emissions) + 1 == 241

header_start = uart.index("static const char header[] =", build_end)
header_end = uart.index(";", header_start)
header_literals = re.findall(r'"(?:\\.|[^"\\])*"', uart[header_start:header_end])
header = "".join(ast.literal_eval(value) for value in header_literals)
header_fields = header.strip().lstrip("# ").split(",")
assert len(header_fields) == 242
assert header_fields[-1] == "crc16_ccitt"
assert len(set(header_fields)) == len(header_fields)

monitor = load_monitor("monitor_uart_p46.py")
assert len(monitor.FIELDS) == 241
assert len(set(monitor.FIELDS)) == 241
assert tuple(header_fields[:-1]) == monitor.FIELDS

# Synthetic valid TGY57 frame + CRC roundtrip.
values = ["$TGY57"] + [str(i) for i in range(1, len(monitor.FIELDS))]
body = ",".join(values)
crc = binascii.crc_hqx(body.encode("ascii"), 0xFFFF)
decoded = monitor.decode_frame(f"{body}*{crc:04X}")
assert len(decoded) == 241

# SD V14 binary contract deliberately unchanged.
assert read("decode_flight_v14_p46.py") == read("decode_flight_v14_p44.py")
assert read("decode_flight_v14_p44.py") == read("tools/decode_flight_v14.py")

print(
    "P46 validation: PASS "
    "(cov root-cause stage/state/raw; roundoff split; state-preserving P rollback; "
    "guard-before-publish; P45 IMU recovery frozen; TGY57/241; SD V14 128/96 MiB)"
)
