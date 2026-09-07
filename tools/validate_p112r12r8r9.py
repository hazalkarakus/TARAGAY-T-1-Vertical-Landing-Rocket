#!/usr/bin/env python3
from pathlib import Path
import hashlib, re, subprocess, shutil, sys, importlib.util, binascii, os
R=Path(__file__).resolve().parents[1]
def txt(r): return (R/r).read_text(errors='ignore')
def sha(r): return hashlib.sha256((R/r).read_bytes()).hexdigest()
checks=[]
def ck(n,v): checks.append((n,bool(v)))
ck('version','8.19M-P112R12R8R9-ESKF-COV-DETERMINISTIC-INERT' in txt('App/Common/app_version.h'))
ck('project','<name>P112R12R8R9_ESKF_COV_DETERMINISTIC_INERT</name>' in txt('.project'))
for f in ['00_START_HERE_P112R12R8R9_TR.md','P112R12R8R9_DO_NOT_FLY.txt','P112R12R8R9_VALIDATION.txt','monitor_uart_p112r12r8r9_eskf_cov_determinism.py']:
    ck('artifact '+f,(R/f).is_file())
expected={
'App/Core/Scheduler/scheduler.c':'4009a3359fad06c0035b599b3d08c6f2684e090f4567303fd5440625c2c5786e',
'App/Core/Tasks/app_tasks.c':'ce3bf2f3197b3f25a7cc732c29209924ff45649525f9f6b6663e0c16ac42c59a',
'App/app.c':'a177c23fb147b2df2ac834df9c433755c9c9055d122bba7616a5d845365982c0',
'App/Modules/Sensors/IMU/imu.c':'ea896a917b7296495437db24e86e23c7270f7d2f93d144ab123a43197398767f',
'App/Modules/Sensors/Barometer/ms5611_spi.c':'fde0a8cb07273b47fe868b228fa2f1cd343f6cda15a8502141bff4dace6fcffc',
'App/Modules/Sensors/Barometer/barometer.c':'e475dd99c9045bf643318006fbe9e05f4d5d68adf27c787ecdaa90dd04787143',
'App/Modules/Sensors/Lidar/Lidar.c':'f0bc9547ff78400d56abb4c07068539cde9761443e3fd4380111ca9718813f92',
'App/Services/UARTTelemetry/uart_telemetry.c':'08f8763d865c8688b232c8039f3e44f6e02ad73f754c00622208d3c0bf1dc0b7',
'App/Services/SDLogger/sd_logger.c':'9fcbf3d9685cdece8d9c2b386aef5d061e98d9cd2b1b16010edd159c6680dc4f',
'App/Modules/Control/GeneratedFlightControl/generated_flight_control.c':'df2b24d4942a5917504b97749add4a926eba362db7893130364ab5d349b41694',
'App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c':'69f6e01e93dbff5b7149f502862b8dea19081f94b5fcfba1573aaae2f19116b6',
'App/Services/NeedleValveHardware/needle_valve_hw.c':'b439781f1fc3923cc4f93528ee42b431b8dc24b04cb82939475d8f6e7d041f4f',
'App/Services/SolenoidOutput/solenoid_output.c':'8915eb3f0246ddeb555f53fce73d1212f4a8a750611b6f8f7259c5c3d66ea901',
'Core/Src/main.c':'cc3806f2a6b33a37756a718a84c580b06eda48329d6a70d64d32fdcc39245f16',
'Core/Src/stm32f4xx_it.c':'6d45c65dd78a85ebc595a876f2ac0f62bf619a817779d1f4fd1d20696874969c',
'Core/Src/tim.c':'6d44a06f525b525334ed61733ce95b557d99fb823270aa1a082f38051ca822e6'}
for r,h in expected.items(): ck('frozen R8R8 '+r,sha(r)==h)
eskf=txt('App/Modules/Estimation/FullStateESKF/full_state_eskf.c'); cfg=txt('App/Common/app_config.h')
ck('cov O2 macro','FULL_ESKF_COV_TIMING_OPT __attribute__((optimize("O2")))' in eskf)
ck('no fast math','fast-math' not in eskf.lower() or 'No -ffast-math' in eskf)
ck('propagation optimized','static FULL_ESKF_COV_TIMING_OPT void FullESKF_PropagateCovariance' in eskf)
ck('sanitization optimized','static FULL_ESKF_COV_TIMING_OPT void FullESKF_SymmetrizeAndClampCovariance' in eskf)
for fn in ['FullESKF_Abs','FullESKF_IsFiniteFloat','FullESKF_CovarianceTouchesUnobservedHorizontalPosition','FullESKF_IsFiniteReasonableCovariance']:
    ck('forced inline '+fn,('FULL_ESKF_FORCE_INLINE' in eskf[eskf.find(fn)-80:eskf.find(fn)+len(fn)+20]))
for n,v in [('APP_FULL_ESKF_GRAVITY_UPDATE_DECIMATION','4U'),('APP_FULL_ESKF_GRAVITY_AXES_PER_CORRECTION','1U'),('APP_FULL_ESKF_COVARIANCE_DECIMATION','32U'),('APP_FULL_ESKF_INTEGRITY_ROWS_PER_CORRECTION','3U')]:
    ck(n,re.search(rf'#define\s+{n}\s+{v}',cfg) is not None)
ck('R8R9 timing flag','#define APP_P112R12R8R9_COV_TIMING_OPT            1U' in cfg)
ck('inert mode','#define APP_P112R12_INERT_OUTPUT_ISOLATION_MODE                     1U' in cfg)
ck('UART RX disabled','#define APP_V55_UART_RX_COMMANDS_ENABLED                0U' in cfg)
for n,v in checks: print(('PASS' if v else 'FAIL')+': '+n)
fail=[n for n,v in checks if not v]
if fail: print('FAILED:',fail); sys.exit(1)
if shutil.which('gcc'):
    inc=[f'-I{R/"App"}',f'-I{R/"Core/Inc"}',f'-I{R/"Drivers/STM32F4xx_HAL_Driver/Inc"}',f'-I{R/"Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"}',f'-I{R/"Drivers/CMSIS/Device/ST/STM32F4xx/Include"}',f'-I{R/"Drivers/CMSIS/Include"}']
    src=['App/Modules/Estimation/FullStateESKF/full_state_eskf.c','App/Core/Scheduler/scheduler.c','App/Core/Tasks/app_tasks.c','App/Services/UARTTelemetry/uart_telemetry.c','App/Modules/Sensors/Barometer/ms5611_spi.c','App/Modules/Sensors/Lidar/Lidar.c']
    subprocess.run(['gcc','-std=gnu11','-fsyntax-only','-DSTM32F407xx','-DUSE_HAL_DRIVER',*inc,*[str(R/x) for x in src]],check=True)
    print('PASS: host gcc syntax-only')
    tests=[
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r5_autonomous_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r9_auto'],'/tmp/r8r9_auto'),
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r12r1_same_target_guard_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r9_guard'],'/tmp/r8r9_guard'),
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r12r6r2_fault_cause_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r9_fault'],'/tmp/r8r9_fault')]
    for c,o in tests: subprocess.run(c,cwd=R,check=True); subprocess.run([o],cwd=R,check=True)
    print('PASS: actuator host regressions')
mon=R/'monitor_uart_p112r12r8r9_eskf_cov_determinism.py'
env=dict(os.environ); env['PYTHONPYCACHEPREFIX']='/tmp/r8r9_pycache'
subprocess.run([sys.executable,'-m','py_compile',str(mon)],check=True,env=env)
spec=importlib.util.spec_from_file_location('m',mon); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
vals=['$TGY70']+['0']*86; b=','.join(vals); crc=binascii.crc_hqx(b.encode('ascii'),0xffff); f,_=m.dec(b+f'*{crc:04X}')
assert f['frame']=='$TGY70' and len(f)==87
print('PASS: R8R9 monitor TGY70 CRC parser')
print('NOTE: ARM/CubeIDE target build unavailable; not claimed.')
print('P112R12R8R9 structural/host validation: PASS')
