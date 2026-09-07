#!/usr/bin/env python3
from pathlib import Path
import ast,re
root=Path(__file__).resolve().parents[1]
def txt(p): return (root/p).read_text(errors='strict')
def req(p,n):
    s=txt(p); assert n in s, f'missing {n!r} in {p}'

req('App/Common/app_version.h','P36-NONBLOCKING-IMU-LIDAR1K')
config=txt('App/Common/app_config.h')
assert re.search(r'APP_TASK_IMU_PERIOD_US\s+1000UL',config)
assert re.search(r'APP_TASK_LIDAR_PERIOD_US\s+1000UL',config)
assert re.search(r'APP_TASK_FULL_ESKF_CORRECTION_PERIOD_US\s+5000UL',config)
assert re.search(r'APP_TASK_FULL_ESKF_COVARIANCE_PERIOD_US\s+40000UL',config)
assert re.search(r'APP_FULL_ESKF_COVARIANCE_DECIMATION\s+40U',config)
assert re.search(r'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES\s+4U',config)
assert re.search(r'APP_SDLOGGER_RING_DRAIN_BUDGET_US\s+320UL',config)

imu=txt('App/Modules/Sensors/IMU/imu.c')
assert 'IMU_ServiceRecovery(uint32_t now_us)' in imu
assert 'IMU_RECOVERY_STEP_SETTLE_WAIT' in imu
assert 'IMU_RECOVERY_SETTLE_US                 40000UL' in imu
assert 'IMU_PerformRecovery' not in imu
service=imu.split('static void IMU_ServiceRecovery(uint32_t now_us)',2)[2].split('/* -------------------------------------------------------------------------- */\n/* Public API',1)[0]
assert 'HAL_Delay' not in service
assert 'IMU_RECOVERY_STEP_RESET_WAIT' in service
assert 'IMU_RECOVERY_STEP_VERIFY' in service
assert 'imu_recovery_attempt_count++' in service
assert 'imu_recovery_failure_count++' in imu

sched=txt('App/Core/Scheduler/scheduler.c')
assert 'FULL_ESKF_COV_25HZ' in sched
assert 'scheduler_priority_order' in sched
for tok in ('0U, 4U, 1U, 3U, 5U, 6U, 2U', 'case 6UL: return APP_TASK_FULL_ESKF_COVARIANCE_BUDGET_US'):
    assert tok in sched

tasks=txt('App/Core/Tasks/app_tasks.c')
assert 'void Task_FullESKFCovariance_25Hz(void)' in tasks
assert 'FullStateESKF_ServiceCovariance();' in tasks

app=txt('App/app.c')
body=app.split('void App_Run(void)',1)[1]
assert 'FullStateESKF_ServiceCovariance();' not in body
assert 'APP_P36_FRESH_MIN_SLACK_US' in body
assert 'APP_P36_REMOTE_MIN_SLACK_US' in body

sd=txt('App/Services/SDLogger/sd_logger.c')
assert 'static const uint16_t crc16_ccitt_table[256]' in sd
uart=txt('App/Services/UARTTelemetry/uart_telemetry.c')
assert 'static const uint16_t uart_crc16_ccitt_table[256]' in uart
for n in ('imu_recovery_state','imu_recovery_step','imu_recovery_attempts','imu_recovery_failures','imu_recovery_max_us','imu_redundant_rejects'):
    assert n in uart

mon=txt('monitor_uart_v55.py')
tree=ast.parse(mon); fields=None
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in node.targets):
        fields=ast.literal_eval(node.value)
assert fields is not None and len(fields)==178
assert len(set(fields))==178
for n in ('imu_recovery_state','imu_recovery_step','imu_recovery_attempts','imu_recovery_failures','imu_recovery_max_us','imu_redundant_rejects'):
    assert n in fields
print('P36 validation: PASS (non-blocking IMU recovery + LiDAR 1 kHz service + scheduler-owned 25 Hz covariance + 178 UART fields)')
