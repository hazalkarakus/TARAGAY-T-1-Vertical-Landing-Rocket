#!/usr/bin/env python3
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
checks = []

def must(path, text):
    data=(root/path).read_text(errors='ignore')
    assert text in data, f'missing {text!r} in {path}'

must('App/Services/SDLogger/sd_logger.h', 'void SDLogger_PushFastIMU(void);')
must('App/Services/SDLogger/sd_logger.c', 'void SDLogger_PushFastIMU(void)')
must('App/Services/SDLogger/sd_logger.c', 'sd_logger_logging_active == 0U')
must('App/Core/Tasks/app_tasks.c', 'SDLogger_PushFastIMU();')
must('App/Core/Tasks/app_tasks.c', 'SDLogger_PublishSources();')
must('App/Core/Tasks/app_tasks.c', '(v87_lidar_task_delta_100ms >= 15UL)')
must('App/app.c', 'sd_boot_attempt < 3U')
must('App/Services/UARTTelemetry/uart_telemetry.c', 'cpu_imu_x100')
must('App/Services/UARTTelemetry/uart_telemetry.c', 'sd_last_hal_error')
must('App/Services/UARTTelemetry/uart_telemetry.c', 'nrf_fifo')
must('App/Common/app_version.h', 'P32-UART-CPU-SD-RECOVERY')

# Heavy publish must not remain in the 1 kHz task body.
tasks=(root/'App/Core/Tasks/app_tasks.c').read_text()
imu=tasks.split('void Task_IMU_1kHz(void)',1)[1].split('void Task_BarometerValveHealth_200Hz',1)[0]
assert 'SDLogger_PublishSources();' not in imu, 'heavy SD publish still in 1 kHz task'
assert 'SDLogger_PushFastIMU();' in imu

# Monitor and firmware header field counts must stay aligned.
import ast
mon=(root/'monitor_uart_v55.py').read_text()
node=ast.parse(mon)
fields=None
for n in node.body:
    if isinstance(n, ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in n.targets):
        fields=ast.literal_eval(n.value)
assert fields is not None
assert len(fields)==149, f'unexpected monitor field count {len(fields)}'

print('P32 validation: PASS (149 UART fields)')
