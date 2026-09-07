#!/usr/bin/env python3
from pathlib import Path
import ast
import re
root=Path(__file__).resolve().parents[1]

def txt(p): return (root/p).read_text(errors='strict')
def req(p, needle):
    s=txt(p)
    assert needle in s, f'missing {needle!r} in {p}'

req('App/Common/app_version.h','P34-DETERMINISTIC-IMU')
config=txt('App/Common/app_config.h')
assert re.search(r'APP_TASK_IMU_PERIOD_US\s+1000UL',config)
assert re.search(r'APP_TASK_FULL_ESKF_CORRECTION_PERIOD_US\s+5000UL',config)
assert re.search(r'APP_FULL_ESKF_COVARIANCE_DECIMATION\s+40U',config)
assert re.search(r'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES\s+2U',config)
assert re.search(r'APP_SDLOGGER_RING_DRAIN_BUDGET_US\s+180UL',config)

sched=txt('App/Core/Scheduler/scheduler.c')
for phase in ('1000UL','2000UL','3000UL','4000UL'):
    assert phase in sched
for n in ('Scheduler_GetTimeUntilIMUReleaseUs','Scheduler_HasIMUSlack','Scheduler_RecordExternalBusyTime','scheduler_slow_task_defer_count'):
    assert n in sched
assert 'cpu_external_busy_accum_us' in sched

app=txt('App/app.c')
first=app.index('void App_Run(void)')
body=app[first:]
assert body.index('Scheduler_Run();') < body.index('SensorManager_ServiceFreshness();')
assert 'FullStateESKF_ServiceCovariance();' in body
assert 'Scheduler_HasIMUSlack(APP_P34_SD_MIN_SLACK_US)' in body
assert 'PreflightTrigger_IsFlightActive() == 0U' in body
assert 'AttitudeControl_ForceSafe();' in body
assert 'NeedleValveController_Stop();' in body

imu=txt('App/Modules/Sensors/IMU/imu.c')
assert 'IMU_REDUNDANT_BURST_COUNT     3U' in imu
macro=imu.split('#define IMU_SW_SPI_EDGE_DELAY()',1)[1].split('} while (0)',1)[0]
assert macro.count('__NOP()') == 8, macro.count('__NOP()')
# Successful software burst path no longer refreshes debug each burst.
sw=imu.split('static HAL_StatusTypeDef IMU_DMA_TransferBlocking',1)[1].split('HAL_StatusTypeDef status = IMU_DMA_StartTransfer',1)[0]
ok=sw.split('if (sw_status == HAL_OK)',1)[1].split('else',1)[0]
assert 'IMU_UpdateLiveDebug();' not in ok

eskf=txt('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
assert 'uint8_t FullStateESKF_ServiceCovariance(void)' in eskf
corr=eskf.split('void FullStateESKF_CorrectMeasurements(void)',1)[1]
assert 'FullESKF_PropagateCovariance(' not in corr

sd=txt('App/Services/SDLogger/sd_logger.c')
assert 'APP_SDLOGGER_RING_DRAIN_BUDGET_US' in sd
assert 'sd_logger_drain_budget_yield_count++' in sd

mon=txt('monitor_uart_v55.py')
tree=ast.parse(mon); fields=None
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in node.targets):
        fields=ast.literal_eval(node.value)
assert fields is not None and len(fields)==167
for name in ('cpu_bg_x100','cov_count','sd_drain_us','sched_slow_defers','bg_uart_us'):
    assert name in fields
print('P34 validation: PASS (deterministic IMU slots + deferred covariance + bounded SD + 167 UART fields)')
