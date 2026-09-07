from pathlib import Path
import ast
import re

ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


version = read("App/Common/app_version.h")
scheduler = read("App/Core/Scheduler/scheduler.c")
scheduler_h = read("App/Core/Scheduler/scheduler.h")
imu = read("App/Modules/Sensors/IMU/imu.c")
config = read("App/Common/app_config.h")

assert "8.19M-P44-SD-COV-IMU-RESYNC" in version
assert re.search(r"scheduler_priority_order\[TASK_COUNT\].*?0U,\s*4U,\s*6U,\s*2U,\s*1U,\s*3U,\s*5U", scheduler, re.S)
assert "case 6UL: return APP_P44_COVARIANCE_SLOT_RESERVE_US;" in scheduler
assert "LIDAR_SERVICE_1KHZ_BEST_EFFORT" in scheduler
assert "lidar_intentional_defer_imu_generation" in scheduler
assert "lidar_scheduler_carry_count" in scheduler
assert "lidar_scheduler_carry_served_count" in scheduler
assert "lidar_scheduler_best_effort_skip_count" in scheduler
assert "extern volatile uint32_t lidar_scheduler_best_effort_skip_count" in scheduler_h

# LiDAR best-effort branch must precede the generic hard-deadline branch.
best_effort_pos = scheduler.index("if (i == 2UL)", scheduler.index("uint32_t lateness_us"))
hard_deadline_pos = scheduler.index("tasks[i].deadline_miss_count++", best_effort_pos)
assert best_effort_pos < hard_deadline_pos

# P44 retry is deferred and preceded by a transaction-boundary resync.
assert "imu_pattern_retry_pending" in imu
assert "retry_samples" not in imu
assert "imu_pattern_retry_success_count++" in imu
assert re.search(r"IMU_SPI_ResynchronizeBus\(\);\s*imu_pattern_retry_pending\s*=\s*1U;\s*imu_pattern_retry_count\+\+", imu)
assert re.search(r"imu_pattern_retry_pending\s*=\s*0U;\s*imu_recovery_state\s*=\s*IMU_RECOVERY_STATE_REQUESTED", imu)

resync_service = imu.split(
    "static void IMU_SPI_ResynchronizeBus(void)", 2
)[2].split("static inline uint8_t IMU_SW_SPI_Byte", 1)[0]
assert "HAL_Delay" not in resync_service
assert "pulse < 16U" in resync_service
assert "IMU_SW_CS_HIGH()" in resync_service

# Frozen architecture/tuning targets.
for token in (
    "APP_TASK_IMU_PERIOD_US                  1000UL",
    "APP_TASK_LIDAR_PERIOD_US                1000UL",
    "APP_TASK_FULL_ESKF_CORRECTION_PERIOD_US 5000UL",
    "APP_TASK_FULL_ESKF_COVARIANCE_PERIOD_US 40000UL",
    "APP_FULL_ESKF_COVARIANCE_DECIMATION",
    "APP_P44_COVARIANCE_SLOT_RESERVE_US        500UL",
    "APP_SDLOGGER_PREALLOCATE_BYTES          (64UL * 1024UL * 1024UL)",
    "APP_SDLOGGER_PREALLOCATE_MIN_BYTES      (48UL * 1024UL * 1024UL)",
):
    assert token in config

# P42 safety, signal-integrity and protocol invariants remain frozen.
sw_spi_delay = imu.split("#define IMU_SW_SPI_EDGE_DELAY()", 1)[1].split(
    "} while (0)", 1
)[0]
assert sw_spi_delay.count("__NOP()") == 12
assert "GPIO_SPEED_FREQ_MEDIUM" in imu

recovery_service = imu.split(
    "static void IMU_ServiceRecovery(uint32_t now_us)", 2
)[2].split(
    "/* -------------------------------------------------------------------------- */\n/* Public API", 1
)[0]
assert "HAL_Delay" not in recovery_service

for relative in (
    "App/Services/SolenoidOutput/solenoid_output.c",
    "App/Modules/Control/AttitudeControl/attitude_control.c",
    "App/Core/Tasks/app_tasks.c",
):
    assert "SystemMonitor_IsActuatorFaultActive()" in read(relative), relative

decoder = read("tools/decode_flight_v14.py")
for token in (
    "u32_went_backwards",
    "session_boundary_detected",
    "old preallocated tail ignored",
):
    assert token in decoder

monitor = read("monitor_uart_p44.py")
tree = ast.parse(monitor)
fields = None
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id == "FIELDS"
        for target in node.targets
    ):
        fields = ast.literal_eval(node.value)
        break

assert fields is not None
assert len(fields) == 214 and len(set(fields)) == 214
assert "imu_pattern_retries" in fields
assert "imu_pattern_retry_success" in fields
assert read("monitor_uart_p42.py") == read("monitor_uart_v55.py")
assert read("decode_flight_v14_p42.py") == decoder
assert read("monitor_uart_p43.py") == read("monitor_uart_v55.py")
assert read("decode_flight_v14_p43.py") == decoder
assert read("decode_flight_v14_p44.py") == decoder

print("P44 validation: PASS (64/48 MiB SD; covariance-first slot; IMU soft bus resync)")
