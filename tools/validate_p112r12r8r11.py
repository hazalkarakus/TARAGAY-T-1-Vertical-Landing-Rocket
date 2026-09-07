#!/usr/bin/env python3
from pathlib import Path
import hashlib,re,subprocess,shutil,sys,importlib.util,binascii,os
R=Path(__file__).resolve().parents[1]
def txt(r): return (R/r).read_text(errors='ignore')
def sha(r): return hashlib.sha256((R/r).read_bytes()).hexdigest()
checks=[]
def ck(n,v): checks.append((n,bool(v)))
ck('version','8.19M-P112R12R8R11-BOUNDED-BACKGROUND-INERT' in txt('App/Common/app_version.h'))
ck('project','<name>P112R12R8R11_BOUNDED_BACKGROUND_INERT</name>' in txt('.project'))
for f in ['00_START_HERE_P112R12R8R11_TR.md','P112R12R8R11_DO_NOT_FLY.txt','P112R12R8R11_VALIDATION.txt','monitor_uart_p112r12r8r11_bounded_background.py']:
    ck('artifact '+f,(R/f).is_file())
expected={
'App/Modules/Estimation/FullStateESKF/full_state_eskf.c':'0586a0ffa7544b44303357a2f61e3570cc0282a0a0569ae6fb0cf2ad5f47e43e',
'App/Core/Scheduler/scheduler.c':'ab4f73a091ada9363c3fbe2690fd17d2ac5bb0866e7fec862e172093c641fe53',
'App/Core/Tasks/app_tasks.c':'ce3bf2f3197b3f25a7cc732c29209924ff45649525f9f6b6663e0c16ac42c59a',
'App/Modules/Sensors/IMU/imu.c':'ea896a917b7296495437db24e86e23c7270f7d2f93d144ab123a43197398767f',
'App/Modules/Sensors/Barometer/ms5611_spi.c':'fde0a8cb07273b47fe868b228fa2f1cd343f6cda15a8502141bff4dace6fcffc',
'App/Modules/Sensors/Lidar/Lidar.c':'f0bc9547ff78400d56abb4c07068539cde9761443e3fd4380111ca9718813f92',
'App/Modules/Control/GeneratedFlightControl/generated_flight_control.c':'df2b24d4942a5917504b97749add4a926eba362db7893130364ab5d349b41694',
'App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c':'69f6e01e93dbff5b7149f502862b8dea19081f94b5fcfba1573aaae2f19116b6',
'Core/Src/main.c':'cc3806f2a6b33a37756a718a84c580b06eda48329d6a70d64d32fdcc39245f16'}
for r,h in expected.items(): ck('R8R10 frozen '+r,sha(r)==h)
cfg=txt('App/Common/app_config.h'); sc=txt('App/Core/Scheduler/scheduler.c'); sd=txt('App/Services/SDLogger/sd_logger.c'); uart=txt('App/Services/UARTTelemetry/uart_telemetry.c')
for n,v in [('APP_P34_ESKF_SLOT_RESERVE_US','620UL'),('APP_P34_BARO_SLOT_RESERVE_US','360UL'),('APP_P44_COVARIANCE_SLOT_RESERVE_US','650UL'),('APP_P34_IMU_GUARD_US','40UL'),('APP_P34_SD_MIN_SLACK_US','520UL'),('APP_P34_UART_MIN_SLACK_US','600UL'),('APP_SDLOGGER_RING_DRAIN_MAX_FRAMES','2U'),('APP_SDLOGGER_RING_DRAIN_BUDGET_US','220UL'),('APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_HIGH','3U'),('APP_SDLOGGER_RING_DRAIN_BUDGET_US_HIGH','330UL'),('APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_CRITICAL','3U'),('APP_SDLOGGER_RING_DRAIN_BUDGET_US_CRITICAL','420UL')]:
    ck(n,re.search(rf'#define\s+{n}\s+{v}',cfg) is not None)
ck('priority unchanged','0U, 4U, 6U, 2U, 1U, 3U, 5U' in sc)
ck('admission formula unchanged','slot_reserve_us + APP_P34_IMU_GUARD_US' in sc)
ck('SD O2 bounded','optimize("O2")' in sd and 'SDLogger_CRC16_CCITT' in sd and 'SDLogger_DrainRingToBuffers' in sd)
ck('UART O2 timing','optimize("O2")' in uart and '$TGY71' in uart)
ck('inert mode','#define APP_P112R12_INERT_OUTPUT_ISOLATION_MODE                     1U' in cfg)
ck('UART RX disabled','#define APP_V55_UART_RX_COMMANDS_ENABLED                0U' in cfg)
for n,v in checks: print(('PASS' if v else 'FAIL')+': '+n)
fail=[n for n,v in checks if not v]
if fail: print('FAILED:',fail); sys.exit(1)
if shutil.which('gcc'):
    inc=[f'-I{R/"App"}',f'-I{R/"Core/Inc"}',f'-I{R/"Drivers/STM32F4xx_HAL_Driver/Inc"}',f'-I{R/"Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"}',f'-I{R/"Drivers/CMSIS/Device/ST/STM32F4xx/Include"}',f'-I{R/"Drivers/CMSIS/Include"}',f'-I{R/"FATFS/App"}',f'-I{R/"FATFS/Target"}',f'-I{R/"Middlewares/Third_Party/FatFs/src"}']
    src=['App/Modules/Estimation/FullStateESKF/full_state_eskf.c','App/Core/Scheduler/scheduler.c','App/Core/Tasks/app_tasks.c','App/Services/UARTTelemetry/uart_telemetry.c','App/Services/SDLogger/sd_logger.c','App/Modules/Sensors/Barometer/ms5611_spi.c','App/Modules/Sensors/Lidar/Lidar.c','App/app.c']
    subprocess.run(['gcc','-std=gnu11','-fsyntax-only','-DSTM32F407xx','-DUSE_HAL_DRIVER',*inc,*[str(R/x) for x in src]],check=True)
    print('PASS: host gcc syntax-only including changed SD/UART')
    tests=[
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r5_autonomous_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r11_auto'],'/tmp/r8r11_auto'),
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r12r1_same_target_guard_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r11_guard'],'/tmp/r8r11_guard'),
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r12r6r2_fault_cause_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r11_fault'],'/tmp/r8r11_fault')]
    for c,o in tests: subprocess.run(c,cwd=R,check=True); subprocess.run([o],cwd=R,check=True)
    print('PASS: actuator host regressions')
mon=R/'monitor_uart_p112r12r8r11_bounded_background.py'
env=dict(os.environ); env['PYTHONPYCACHEPREFIX']='/tmp/r8r11_pycache'
subprocess.run([sys.executable,'-m','py_compile',str(mon)],check=True,env=env)
spec=importlib.util.spec_from_file_location('m',mon); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
vals=['$TGY71']+['0']*108; b=','.join(vals); crc=binascii.crc_hqx(b.encode('ascii'),0xffff); f,_=m.dec(b+f'*{crc:04X}')
assert f['frame']=='$TGY71' and len(f)==109
print('PASS: R8R11 monitor TGY71 CRC parser')
print('NOTE: ARM/CubeIDE target build unavailable; not claimed.')
print('P112R12R8R11 structural/host validation: PASS')
