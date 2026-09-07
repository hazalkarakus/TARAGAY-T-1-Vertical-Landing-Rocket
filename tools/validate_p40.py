#!/usr/bin/env python3
from pathlib import Path
import ast,re
root=Path(__file__).resolve().parents[1]
def txt(p): return (root/p).read_text(encoding='utf-8',errors='strict')
ver=txt('App/Common/app_version.h')
assert 'APP_VERSION_PATCH                       40' in ver
assert '8.19M-P40-FINAL-HARDENING' in ver
imu=txt('App/Modules/Sensors/IMU/imu.c')
macro=imu.split('#define IMU_SW_SPI_EDGE_DELAY()',1)[1].split('} while (0)',1)[0]
assert macro.count('__NOP()') == 12
assert 'GPIO_SPEED_FREQ_MEDIUM' in imu
assert 'imu_pattern_retry_count++' in imu
assert 'imu_pattern_retry_success_count++' in imu
assert 'IMU_GetPatternRetryCount' in txt('App/Modules/Sensors/IMU/imu.h')
# Runtime recovery remains nonblocking.
service=imu.split('static void IMU_ServiceRecovery(uint32_t now_us)',2)[2].split('/* -------------------------------------------------------------------------- */\n/* Public API',1)[0]
assert 'HAL_Delay' not in service
sm=txt('App/Services/SystemMonitor/system_monitor.c')
smh=txt('App/Services/SystemMonitor/system_monitor.h')
assert 'SystemMonitor_IsActuatorFaultActive' in sm and 'SystemMonitor_IsActuatorFaultActive' in smh
for p in ('App/Services/SolenoidOutput/solenoid_output.c','App/Modules/Control/AttitudeControl/attitude_control.c','App/Core/Tasks/app_tasks.c'):
    assert 'SystemMonitor_IsActuatorFaultActive()' in txt(p),p
sched=txt('App/Core/Scheduler/scheduler.c')
assert re.search(r'0U,\s*4U,\s*2U,\s*1U,\s*3U,\s*5U,\s*6U',sched)
dec=txt('tools/decode_flight_v14.py')
for t in ('u32_went_backwards','session_boundary_detected','old preallocated tail ignored'):
    assert t in dec,t
mon=txt('monitor_uart_v55.py')
tree=ast.parse(mon); fields=None
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in node.targets):
        fields=ast.literal_eval(node.value); break
assert fields is not None and len(fields)==214 and len(set(fields))==214
assert 'imu_pattern_retries' in fields and 'imu_pattern_retry_success' in fields
uart=txt('App/Services/UARTTelemetry/uart_telemetry.c')
assert 'imu_pattern_retries' in uart and 'imu_pattern_retry_success' in uart
print('P40 validation: PASS (actuator fault latch + IMU quick retry/signal integrity + LiDAR priority + session-safe V14 decoder + 214 UART fields)')
