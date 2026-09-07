#!/usr/bin/env python3
from pathlib import Path
import ast,re
root=Path(__file__).resolve().parents[1]
def txt(p): return (root/p).read_text(encoding='utf-8',errors='strict')
def req(p,n):
    s=txt(p); assert n in s, f'missing {n!r} in {p}'
def pat(p,r):
    s=txt(p); assert re.search(r,s,re.M|re.S), f'missing /{r}/ in {p}'

req('App/Common/app_version.h','APP_VERSION_PATCH                       39')
req('App/Common/app_version.h','8.19M-P39-FUSION-IMU-RCS-HARDENING')
config=txt('App/Common/app_config.h')
for r in (
 r'APP_TASK_IMU_PERIOD_US\s+1000UL',
 r'APP_TASK_LIDAR_PERIOD_US\s+1000UL',
 r'APP_TASK_FULL_ESKF_CORRECTION_PERIOD_US\s+5000UL',
 r'APP_TASK_FULL_ESKF_COVARIANCE_PERIOD_US\s+40000UL',
 r'APP_FULL_ESKF_COVARIANCE_DECIMATION\s+32U',
 r'APP_FULL_ESKF_LIDAR_ABS_GATE_M\s+0\.25f',
 r'APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_CONFIRM_SAMPLES\s+6U',
 r'APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_MAX_M\s+1\.00f',
 r'APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_EXIT_M\s+0\.12f',
 r'APP_BARO_RAW_MAX_SINGLE_JUMP_PA\s+50\.0f',
 r'APP_BARO_RAW_STEP_CONFIRM_TOLERANCE_PA\s+12\.0f',
): assert re.search(r,config),r

imu=txt('App/Modules/Sensors/IMU/imu.c')
assert re.search(r'IMU_STALE_TIMEOUT_US\s+20000UL',imu)
assert 'IMU_RawDataEqual(raw, &imu_stale_reference_raw)' in imu
assert 'imu_stale_count++' in imu
# P36 non-blocking recovery must remain.
service=imu.split('static void IMU_ServiceRecovery(uint32_t now_us)',2)[2].split('/* -------------------------------------------------------------------------- */\n/* Public API',1)[0]
assert 'HAL_Delay' not in service

baro=txt('App/Modules/Sensors/Barometer/barometer.c')
baroh=txt('App/Modules/Sensors/Barometer/barometer.h')
for token in ('Barometer_RawPressurePlausible','baro_raw_step_candidate_valid',
              'raw_spike_reject_count','raw_step_confirm_count'):
    assert token in baro or token in baroh,token
assert 'barometer_data.pressure_pa = ms.pressure_pa;' in baro
# Raw pressure must only publish after plausibility check.
assert baro.index('Barometer_RawPressurePlausible(ms.pressure_pa)') < baro.index('barometer_data.pressure_pa = ms.pressure_pa;')

eskf=txt('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
for token in ('lidar_innov_reacquire_active','full_eskf_lidar_innov_reacquire_count',
              'APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_MAX_M',
              'APP_FULL_ESKF_LIDAR_MAX_POSITION_STEP_M',
              'APP_FULL_ESKF_LIDAR_MAX_VELOCITY_STEP_MPS'):
    assert token in eskf,token
# Normal gate is preserved; widened gate is only inside explicit reacquisition branch.
assert eskf.count('APP_FULL_ESKF_LIDAR_ABS_GATE_M') >= 2
assert 'APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_CONFIRM_SAMPLES' in eskf
assert 'APP_FULL_ESKF_LIDAR_INNOV_REACQUIRE_EXIT_SAMPLES' in eskf

smh=txt('App/Services/SystemMonitor/system_monitor.h')
sm=txt('App/Services/SystemMonitor/system_monitor.c')
assert 'SystemMonitor_GetFastFaultCode' in smh and 'SystemMonitor_GetFastFaultCode' in sm
for token in ('APP_SYSTEM_MONITOR_IMU_STALE_US','APP_SYSTEM_MONITOR_LIDAR_STALE_US',
              'APP_SYSTEM_MONITOR_ESKF_PUBLIC_STALE_US','SDLogger_IsReady()','SDLogger_IsLogging()'):
    assert token in sm,token

sol=txt('App/Services/SolenoidOutput/solenoid_output.c')
att=txt('App/Modules/Control/AttitudeControl/attitude_control.c')
tasks=txt('App/Core/Tasks/app_tasks.c')
assert 'SystemMonitor_IsFastFaultActive()' in sol
assert 'SystemMonitor_IsFastFaultActive()' in att
assert tasks.count('SystemMonitor_IsFastFaultActive()') >= 3
# Timer ISR must abort pulse state, not only hide the physical mask.
timer=att.split('void AttitudeControl_TimerTickISR(void)',1)[1].split('/* Legacy wrappers',1)[0]
assert 'AttitudeControl_AbortAxis(&roll_axis, 1U);' in timer
assert 'AttitudeControl_AbortAxis(&pitch_axis, 1U);' in timer
assert 'SolenoidOutput_ForceSafe();' in timer

sched=txt('App/Core/Scheduler/scheduler.c')
assert 'lidar_intentional_defer_pending' in sched
assert 'Expected one-slot carry' in sched
# P38/P36 release-dropping assignment must not exist in defer branch anymore.
defer=sched.split('if (Scheduler_HasIMUSlack(',1)[1].split('any_task_ran = 1U;',1)[0]
assert 'tasks[i].next_run_us = now + tasks[i].period_us;' not in defer

# Preserve P38 LiDAR watchdog fix and non-blocking recovery.
lidar=txt('App/Modules/Sensors/Lidar/Lidar.c')
for token in ('lidar_watchdog_armed','lidar_recovery_confirmation_pending',
              'Lidar_BeginFirstSampleGrace','LIDAR_FIRST_SAMPLE_GRACE_MS'):
    assert token in lidar,token
active='\n'.join(line.split('//',1)[0] for line in lidar.splitlines())
active=re.sub(r'/\*.*?\*/','',active,flags=re.S)
assert 'HAL_I2C_IsDeviceReady' not in active

# Preserve P35/P37 SD path and 200 Hz frame production.
sd=txt('App/Services/SDLogger/sd_logger.c')
for token in ('SDLogger_CRC16','SDLogger_AttemptRuntimeHostRecovery','sd_logger_dropped_frame_count'):
    assert token in sd,token
assert re.search(r'APP_SDLOGGER_SAMPLE_PERIOD_US\s+5000UL',config)

# P39 UART adds six targeted diagnostics; protocol/CRC format remains unchanged.
mon=txt('monitor_uart_v55.py')
tree=ast.parse(mon); fields=None
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in node.targets):
        fields=ast.literal_eval(node.value); break
assert fields is not None and len(fields)==212 and len(set(fields))==212
for name in ('fast_fault','baro_raw_spike_rejects','baro_raw_step_confirms',
             'eskf_lidar_soft_reacq_active','eskf_lidar_soft_reacq_count',
             'eskf_lidar_soft_reacq_success'):
    assert name in fields, name
uart=txt('App/Services/UARTTelemetry/uart_telemetry.c')
assert '$TGY55' in uart

print('P39 validation: PASS (RCS 1kHz fast-fault hard gate + bounded LiDAR innovation reacquisition + IMU 20ms exact-repeat stale + baro raw-step guard + carried LiDAR service + 25Hz covariance readiness + 212 UART fields)')
