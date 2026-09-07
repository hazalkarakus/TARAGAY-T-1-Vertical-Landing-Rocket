#!/usr/bin/env python3
from pathlib import Path
import ast,re
root=Path(__file__).resolve().parents[1]
def txt(p): return (root/p).read_text(encoding='utf-8',errors='strict')
ver=txt('App/Common/app_version.h')
assert 'APP_VERSION_PATCH                       41' in ver
assert '8.19M-P41-SCHEDULER-FINAL' in ver
# P40 hardening must remain intact.
imu=txt('App/Modules/Sensors/IMU/imu.c')
macro=imu.split('#define IMU_SW_SPI_EDGE_DELAY()',1)[1].split('} while (0)',1)[0]
assert macro.count('__NOP()') == 12
assert 'GPIO_SPEED_FREQ_MEDIUM' in imu
assert 'imu_pattern_retry_count++' in imu
assert 'imu_pattern_retry_success_count++' in imu
service=imu.split('static void IMU_ServiceRecovery(uint32_t now_us)',2)[2].split('/* -------------------------------------------------------------------------- */\n/* Public API',1)[0]
assert 'HAL_Delay' not in service
for p in ('App/Services/SolenoidOutput/solenoid_output.c','App/Modules/Control/AttitudeControl/attitude_control.c','App/Core/Tasks/app_tasks.c'):
    assert 'SystemMonitor_IsActuatorFaultActive()' in txt(p),p
# P41 changes only execution priority, never task-array indices/phases.
sched=txt('App/Core/Scheduler/scheduler.c')
assert re.search(r'0U,\s*2U,\s*4U,\s*1U,\s*6U,\s*3U,\s*5U',sched)
assert 'APP_TASK_LIDAR_PERIOD_US' in sched
assert '3000UL' in sched  # LiDAR/cov phases are intentionally preserved.
# Session-safe V14 decoder remains.
dec=txt('tools/decode_flight_v14.py')
for t in ('u32_went_backwards','session_boundary_detected','old preallocated tail ignored'):
    assert t in dec,t
# UART remains protocol-compatible with P40: 214 unique fields.
mon=txt('monitor_uart_v55.py')
tree=ast.parse(mon); fields=None
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in node.targets):
        fields=ast.literal_eval(node.value); break
assert fields is not None and len(fields)==214 and len(set(fields))==214
assert 'imu_pattern_retries' in fields and 'imu_pattern_retry_success' in fields
assert txt('monitor_uart_p41.py') == mon
print('P41 validation: PASS (LiDAR-before-ESKF scheduler + covariance priority; P40 hardening preserved; 214 UART fields)')
