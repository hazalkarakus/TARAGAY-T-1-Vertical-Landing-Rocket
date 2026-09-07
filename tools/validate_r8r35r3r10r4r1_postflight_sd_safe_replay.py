#!/usr/bin/env python3
from pathlib import Path
import hashlib, importlib.util, os, re, shutil, subprocess, sys

R = Path(__file__).resolve().parents[1]
checks=[]

def ck(name, cond, detail=''):
    checks.append((name, bool(cond), detail))

def txt(rel):
    return (R/rel).read_text(errors='replace')

def sha(rel):
    return hashlib.sha256((R/rel).read_bytes()).hexdigest()

ver=txt('App/Common/app_version.h')
proj=txt('.project')
cfg=txt('App/Common/app_config.h')
sd=txt('App/Services/SDLogger/sd_logger.c')

ck('R4R1 version string', 'R3R10R4R1-NRF-STABLE-TDD-POSTFLIGHT-SD-SAFE-REPLAY' in ver)
ck('CubeIDE project name', '<name>P112R12R8R35R3R10R4R1_NRF_STABLE_TDD_POSTFLIGHT_SD_SAFE_REPLAY</name>' in proj)
ck('postflight quiet = 3000 ms', re.search(r'#define\s+APP_SDLOGGER_R10_POSTFLIGHT_QUIET_MS\s+3000UL', cfg) is not None)
ck('E-stop CLOSE complete gate', 'NeedleValveAutonomousControl_IsEStopSafeCloseComplete() == 0U' in sd)
ck('E-stop close failure blocks replay', 'NeedleValveAutonomousControl_HasEStopSafeCloseFailed() != 0U' in sd)
ck('actual motor/RCS OFF gate', all(x in sd for x in ['needle_valve_rpwm == 0U','needle_valve_lpwm == 0U','solenoid.requested_mask == SOLENOID_VALVE_NONE','solenoid.applied_mask == SOLENOID_VALVE_NONE']))
ck('quiet timestamp starts after gates', 'sd_r10_postflight_quiet_since_ms = now_ms;' in sd)
ck('postflight command-path soft recovery', 'BSP_SD_RuntimeSoftRecover() != MSD_OK' in sd)
ck('no full card reinit in R10 postflight', 'BSP_SD_RuntimeReinit()' not in sd[sd.find('static void SDLogger_R10ServicePostflight(void)'):sd.find('static void SDLogger_ExtendWriteHoldUntil')])
ck('historical PE9/tail SD fault no longer hard-blocks replay', 'if (sd_logger_r10_physical_sd_fault_count != 0UL)' not in sd[sd.find('static void SDLogger_R10ServicePostflight(void)'):sd.find('static void SDLogger_ExtendWriteHoldUntil')])
ck('fresh post-recovery SD fault blocks further replay', 'sd_logger_r10_physical_sd_fault_count > sd_r10_replay_fault_baseline' in sd)
ck('R10 RAM decimation retained', '#define APP_SDLOGGER_R10_RAM_DECIMATION          4U' in cfg)
ck('runtime HAL_Delay guard retained', sha('App/Common/runtime_delay_guard.c') == 'd5b685c9cc92a14209562aeeb6109b7623d446e1c3aeeb342fd57e4ee56d2a61')

# Byte-identical R4 nRF path and frozen control authority.
expected={
'App/Modules/NRF24/nrf24.c':'06fd9206da4ad270419da8adaca699d4dbae77dfb33510cec0ecd762afbadc68',
'App/Services/NRFTelemetry/nrf_telemetry.c':'8b591357b3aef2583abf46ea6e6804cc4572cb90d70bf632e5e3362ae3cca53b',
'App/Modules/RemoteControl/remote_control.c':'d36d7729163d2ae8d66d3c76d0e22b369cf4d47e426ff22c1ff84640afb7b1be',
'App/app.c':'3c1792f3a98b6eb3e10b98251a8a09ceb42a35a99785f93c3c439920ebf24f66',
'App/Core/Tasks/app_tasks.c':'6f3adda7a0bcfd9f67ff0fa820c4ed5bd791693c9826318357036cdfdc6d8d1a',
'App/Core/Scheduler/scheduler.c':'ab4f73a091ada9363c3fbe2690fd17d2ac5bb0866e7fec862e172093c641fe53',
'App/Modules/Control/TaragayFlightLogic/taragay_flight_logic.c':'2bded646fbd07c16bebed9769c87b0a7011fcfd917b3d6a4363a45428d6103fc',
'App/Modules/Estimation/FullStateESKF/full_state_eskf.c':'0586a0ffa7544b44303357a2f61e3570cc0282a0a0569ae6fb0cf2ad5f47e43e',
'App/Services/SolenoidOutput/solenoid_output.c':'cb580faa642181fc4dee3253684b8b7210108f665bf786e7d650f38779f03aa7',
'App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c':'d0df374d9848db4d9cf8c01237d93f044244cd5e58b885bf8457d6fdc58d71ba',
'App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c':'e21c58cbcf5f751124d7e5dce82f21cf72b0c0a20686d4d041f612983918dbf6',
'App/Modules/Control/NeedleValve/needle_valve_controller.c':'11d17fa548233757016b92b36ef78de2d502335e50ef5dc2915ab130b4f8aa31',
'FATFS/Target/bsp_driver_sd.c':'f73a1d058328f2a816b57625026066fda4bbe0444f5cae68fdb6e06caf4670d9',
}
for rel,h in expected.items():
    ck('R4/frozen unchanged: '+rel, sha(rel)==h)

monitor=R/'monitor_uart_p112r12r8r35r3r10r4r1_postflight_sd_safe_replay_LIVE.py'
ck('R4R1 monitor present', monitor.is_file())
if monitor.is_file():
    env=dict(os.environ); env['PYTHONPYCACHEPREFIX']='/tmp/r4r1_pycache'
    try:
        subprocess.run([sys.executable,'-m','py_compile',str(monitor)],check=True,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        ck('monitor py_compile', True)
    except subprocess.CalledProcessError as e:
        ck('monitor py_compile', False, e.stderr.decode(errors='replace'))
    try:
        spec=importlib.util.spec_from_file_location('r4r1mon',monitor)
        m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
        ck('TGY73 schema remains 341 fields', len(m.FIELDS)==341 and m.N==341)
    except Exception as e:
        ck('TGY73 schema remains 341 fields', False, str(e))

if shutil.which('gcc'):
    inc=[f'-I{R/"App"}',f'-I{R/"Core/Inc"}',f'-I{R/"FATFS/App"}',f'-I{R/"FATFS/Target"}',f'-I{R/"Middlewares/Third_Party/FatFs/src"}',f'-I{R/"Drivers/STM32F4xx_HAL_Driver/Inc"}',f'-I{R/"Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"}',f'-I{R/"Drivers/CMSIS/Device/ST/STM32F4xx/Include"}',f'-I{R/"Drivers/CMSIS/Include"}']
    cmd=['gcc','-std=gnu11','-fsyntax-only','-DSTM32F407xx','-DUSE_HAL_DRIVER',*inc,str(R/'App/Services/SDLogger/sd_logger.c')]
    try:
        subprocess.run(cmd,check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        ck('modified SDLogger host GCC syntax-only', True)
    except subprocess.CalledProcessError as e:
        ck('modified SDLogger host GCC syntax-only', False, e.stderr.decode(errors='replace')[-3000:])
else:
    ck('modified SDLogger host GCC syntax-only', True, 'gcc unavailable; skipped')

fails=[]
print('R8R35R3R10R4R1 POSTFLIGHT SD SAFE REPLAY - STATIC VALIDATION')
for name,ok,detail in checks:
    print(('[PASS] ' if ok else '[FAIL] ')+name+((' :: '+detail) if detail else ''))
    if not ok: fails.append(name)
print(f'PASS={sum(1 for _,ok,_ in checks if ok)} FAIL={len(fails)}')
print('NOTE: ARM/CubeIDE target build and hardware inert test are NOT run here.')
sys.exit(1 if fails else 0)
