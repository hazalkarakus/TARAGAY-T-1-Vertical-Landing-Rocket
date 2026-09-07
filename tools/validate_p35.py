#!/usr/bin/env python3
from pathlib import Path
import ast,re
root=Path(__file__).resolve().parents[1]
def txt(p): return (root/p).read_text(errors='strict')
def req(p,n):
    s=txt(p); assert n in s, f'missing {n!r} in {p}'
req('App/Common/app_version.h','P35-FAST-CRC-SD-BALANCED')
config=txt('App/Common/app_config.h')
assert re.search(r'APP_TASK_IMU_PERIOD_US\s+1000UL',config)
assert re.search(r'APP_TASK_FULL_ESKF_CORRECTION_PERIOD_US\s+5000UL',config)
assert re.search(r'APP_FULL_ESKF_COVARIANCE_DECIMATION\s+40U',config)
assert re.search(r'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES\s+4U',config)
assert re.search(r'APP_SDLOGGER_RING_DRAIN_BUDGET_US\s+320UL',config)
assert re.search(r'APP_P34_SD_MIN_SLACK_US\s+220UL',config)
assert re.search(r'APP_P34_UART_MIN_SLACK_US\s+300UL',config)

sd=txt('App/Services/SDLogger/sd_logger.c')
assert 'static const uint16_t crc16_ccitt_table[256]' in sd
assert 'crc16_ccitt_table[index]' in sd
# old bit loop must be gone from the SD CRC function
crcpart=sd.split('static uint16_t SDLogger_CRC16_CCITT',1)[1].split('static uint32_t SDLogger_RingCount',1)[0]
assert 'for (uint8_t bit' not in crcpart

uart=txt('App/Services/UARTTelemetry/uart_telemetry.c')
assert 'static const uint16_t uart_crc16_ccitt_table[256]' in uart
crcpart=uart.split('static uint16_t UARTTelemetry_Crc16Ccitt',1)[1].split('static uint8_t UARTTelemetry_BuildDataLine',1)[0]
assert 'for (bit =' not in crcpart
assert 'uart_crc16_ccitt_table[table_index]' in crcpart

sched=txt('App/Core/Scheduler/scheduler.c')
assert 'slow_task_blocked_imu_generation[TASK_COUNT]' in sched
assert 'slow_task_blocked_imu_generation[i] == imu_generation' in sched
assert 'scheduler_slow_task_defer_count++' in sched

mon=txt('monitor_uart_p34.py')
tree=ast.parse(mon); fields=None
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in node.targets):
        fields=ast.literal_eval(node.value)
assert fields is not None and len(fields)==167
print('P35 validation: PASS (fast CRC + balanced SD drain + one-defer-per-IMU-generation + 167 UART fields)')
