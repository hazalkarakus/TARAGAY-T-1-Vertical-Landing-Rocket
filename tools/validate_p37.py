#!/usr/bin/env python3
from pathlib import Path
import ast,re
root=Path(__file__).resolve().parents[1]
def txt(p): return (root/p).read_text(encoding='utf-8',errors='strict')
def req(p,n):
    s=txt(p); assert n in s, f'missing {n!r} in {p}'

req('App/Common/app_version.h','8.19M-P37-LIDAR-SD-ESKF-RECOVERY')
config=txt('App/Common/app_config.h')
for pat in (
    r'APP_TASK_IMU_PERIOD_US\s+1000UL',
    r'APP_TASK_LIDAR_PERIOD_US\s+1000UL',
    r'APP_TASK_FULL_ESKF_CORRECTION_PERIOD_US\s+5000UL',
    r'APP_TASK_FULL_ESKF_COVARIANCE_PERIOD_US\s+40000UL',
    r'APP_FULL_ESKF_VERTICAL_DIVERGENCE_ENABLED\s+1U',
    r'APP_FULL_ESKF_VERTICAL_DIVERGENCE_CONFIRM_SAMPLES\s+10U',
    r'APP_FULL_ESKF_VERTICAL_REACQUIRE_STABLE_SAMPLES\s+40U',
): assert re.search(pat,config), pat

imu=txt('App/Modules/Sensors/IMU/imu.c')
assert '10 x __NOP' not in imu  # source should be actual repeated NOPs, not prose
# Count NOP tokens in the SW SPI edge macro body.
m=re.search(r'#define IMU_SW_SPI_EDGE_DELAY\(\).*?while\s*\(0\)',imu,re.S)
assert m and m.group(0).count('__NOP()')==10
service=imu.split('static void IMU_ServiceRecovery(uint32_t now_us)',2)[2].split('/* -------------------------------------------------------------------------- */\n/* Public API',1)[0]
assert 'HAL_Delay' not in service

lidar=txt('App/Modules/Sensors/Lidar/Lidar.c')
# No active blocking address-ready probe is permitted. Ignore comments.
active='\n'.join(line.split('//',1)[0] for line in lidar.splitlines())
active=re.sub(r'/\*.*?\*/','',active,flags=re.S)
assert 'HAL_I2C_IsDeviceReady' not in active
assert 'LIDAR_RECOVERY_STEP_DMA_WAIT' in lidar
assert '__HAL_DMA_DISABLE' in lidar
assert 'Lidar_RequestRecovery(LIDAR_RUNTIME_RETRY_MS)' in lidar
lsvc=lidar.split('static void Lidar_ServiceRecovery(void)',1)[1].split('static uint8_t Lidar_TryConnect',1)[0]
assert 'HAL_Delay' not in lsvc
assert 'HAL_I2C_IsDeviceReady' not in lsvc

sd=txt('App/Services/SDLogger/sd_logger.c')
bsp=txt('FATFS/Target/bsp_driver_sd.c')
for tok in ('SDLogger_AttemptRuntimeHostRecovery','BSP_SD_RuntimeSoftRecover',
            'BSP_SD_RuntimeQuiesceNonBlocking','sd_logger_runtime_recovery_flight_abort_count'):
    assert tok in sd or tok in bsp, tok
assert 'PreflightTrigger_IsFlightActive()' in sd
# Runtime diagnostics must only reset in init, not on every successful buffer start.
assert sd.count('sd_logger_runtime_recovery_count = 0UL;') == 2  # declaration + Init reset
assert 'sd_logger_runtime_recovery_flight_abort_count++' in sd
assert 'BSP_SD_RuntimeQuiesceNonBlocking' in bsp

eskf_h=txt('App/Modules/Estimation/FullStateESKF/full_state_eskf.h')
eskf=txt('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
for tok in ('output_inhibited','vertical_reacquire_active','vertical_divergence_reason',
            'vertical_divergence_count','vertical_reacquire_count'):
    assert tok in eskf_h
for tok in ('FullESKF_ServiceVerticalDivergence','FullESKF_ResetVerticalSubstate',
            'APP_FULL_ESKF_VERTICAL_DUAL_CONSISTENCY_M',
            'APP_FULL_ESKF_VERTICAL_REACQUIRE_INNOV_M'):
    assert tok in eskf

tasks=txt('App/Core/Tasks/app_tasks.c')
assert tasks.count('output_inhibited == 0U') >= 3
assert 'GeneratedFlightControl_HoldSafe();' in tasks
assert 'AttitudeControl_ForceSafe();' in tasks

gnc=txt('App/Modules/Control/GNCActiveControl/gnc_active_control.c')
assert 'eskf->output_inhibited == 0U' in gnc

smh=txt('App/Services/SystemMonitor/system_monitor.h')
sm=txt('App/Services/SystemMonitor/system_monitor.c')
assert 'SYS_FAULT_ESKF_DIVERGENCE = 16' in smh
assert 'SYS_FAULT_SD_LOGGING = 17' in smh
assert 'output_inhibited' in sm
assert 'SDLogger_IsReady()' in sm and 'SDLogger_IsLogging()' in sm

uart=txt('App/Services/UARTTelemetry/uart_telemetry.c')
m=re.search(r'UART_TELEMETRY_BUFFER_SIZE\s+(\d+)U',uart)
assert m and int(m.group(1)) >= 3072
for tok in ('eskf_inhibit','eskf_reacquire','lidar_rec_active','lidar_rec_step_max_us',
            'sd_runtime_rec_success','sd_runtime_rec_flight_aborts'):
    assert tok in uart

mon=txt('monitor_uart_v55.py')
tree=ast.parse(mon); fields=None
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in node.targets):
        fields=ast.literal_eval(node.value); break
assert fields is not None and len(fields)==206
assert len(set(fields))==206
for tok in ('eskf_inhibit','eskf_reacquire','lidar_rec_active','lidar_rec_step_max_us',
            'sd_runtime_rec_success','sd_runtime_rec_flight_aborts'):
    assert tok in fields
print('P37 validation: PASS (non-blocking LiDAR recovery + SD runtime containment/recovery + ESKF reacquisition/inhibit + 206 UART fields)')
