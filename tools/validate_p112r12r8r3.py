#!/usr/bin/env python3
from pathlib import Path
import hashlib, subprocess, sys, shutil, os
R=Path(__file__).resolve().parents[1]
def text(rel): return (R/rel).read_text(errors='ignore')
def has(rel,s): return s in text(rel)
def lacks(rel,s): return s not in text(rel)
def sha(rel): return hashlib.sha256((R/rel).read_bytes()).hexdigest()
checks=[
('version',has('App/Common/app_version.h','8.19M-P112R12R8R3-BARO-ATOMIC-BURST-PROD-CANDIDATE-INERT')),
('project name',has('.project','<name>P112R12R8R3_BARO_ATOMIC_BURST_PROD_CANDIDATE_INERT</name>')),
('atomic burst attempts',has('App/Modules/Sensors/Barometer/ms5611_spi.c','#define BMP585_MEASUREMENT_ATOMIC_ATTEMPTS  3U')),
('measurement uses six-byte burst',has('App/Modules/Sensors/Barometer/ms5611_spi.c','BMP585_ReadRegs(BMP585_REG_TEMP_DATA_XLSB, raw, 6U)')),
('no single-register measurement helper',lacks('App/Modules/Sensors/Barometer/ms5611_spi.c','BMP585_ReadMeasurementBytesSingle')),
('fallback never enabled',lacks('App/Modules/Sensors/Barometer/ms5611_spi.c','bmp585_single_register_fallback_active = 1U')),
('temperature absolute plausibility',has('App/Modules/Sensors/Barometer/ms5611_spi.c','BMP585_TEMPERATURE_MIN_C')),
('temperature continuity plausibility',has('App/Modules/Sensors/Barometer/ms5611_spi.c','BMP585_TEMPERATURE_MAX_STEP_C')),
('pressure continuity plausibility',has('App/Modules/Sensors/Barometer/ms5611_spi.c','BMP585_PRESSURE_MAX_STEP_PA')),
('bounded atomic retry counter',has('App/Modules/Sensors/Barometer/ms5611_spi.c','bmp585_atomic_retry_count++')),
('upper temp publish after pressure guard', text('App/Modules/Sensors/Barometer/barometer.c').find('barometer_data.temperature_c = ms.temperature_c;') > text('App/Modules/Sensors/Barometer/barometer.c').find('Barometer_RawPressurePlausible(ms.pressure_pa, now_us)')),
('R12 clean production retained',has('App/Common/app_config.h','#define APP_P112R12R3_CLEAN_PRODUCTION_CHAIN_REV                 1U')),
('freshness guard retained',has('App/Common/app_config.h','#define APP_P112R12R8_GNC_FRESHNESS_GUARD_ENABLED                   1U')),
('inert output isolation',has('App/Common/app_config.h','#define APP_P112R12_INERT_OUTPUT_ISOLATION_MODE                     1U')),
('UART RX disabled',has('App/Common/app_config.h','#define APP_V55_UART_RX_COMMANDS_ENABLED                0U')),
('LiDAR bus clear retained',has('App/Modules/Sensors/Lidar/Lidar.c','static uint8_t Lidar_BusClearGPIO(void)')),
('do not fly',(R/'P112R12R8R3_DO_NOT_FLY.txt').is_file()),
('start doc',(R/'00_START_HERE_P112R12R8R3_TR.md').is_file()),
('monitor',(R/'monitor_uart_p112r12r8r3_baro_atomic_cpu_stability.py').is_file()),
]
expected={
'App/Modules/Sensors/Lidar/Lidar.c':'86e5fd5f7664243f88255895d19817d40f9bf771d16ceb83425ede524304849a',
'App/Services/UARTTelemetry/uart_telemetry.c':'db3168b54aed8c12e16c0a0f80baf15fde56b2d87facd1c5c80da8937a18cab4',
'App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c':'e21c58cbcf5f751124d7e5dce82f21cf72b0c0a20686d4d041f612983918dbf6',
'App/Services/NeedleValveHardware/needle_valve_hw.c':'b439781f1fc3923cc4f93528ee42b431b8dc24b04cb82939475d8f6e7d041f4f',
'App/Modules/Control/GeneratedFlightControl/generated_flight_control.c':'df2b24d4942a5917504b97749add4a926eba362db7893130364ab5d349b41694',
'App/app.c':'a177c23fb147b2df2ac834df9c433755c9c9055d122bba7616a5d845365982c0',
'Core/Src/stm32f4xx_it.c':'6d45c65dd78a85ebc595a876f2ac0f62bf619a817779d1f4fd1d20696874969c',
'App/Services/SDLogger/sd_logger.c':'9fcbf3d9685cdece8d9c2b386aef5d061e98d9cd2b1b16010edd159c6680dc4f',
'App/Services/SolenoidOutput/solenoid_output.c':'8915eb3f0246ddeb555f53fce73d1212f4a8a750611b6f8f7259c5c3d66ea901',
'App/Services/ServoOutput/servo_output.c':'c67feac970817ea4a5086a8fca669ca4c1674de82ba08264a60f76099e0bb95e',
'App/Core/Tasks/app_tasks.c':'ce3bf2f3197b3f25a7cc732c29209924ff45649525f9f6b6663e0c16ac42c59a',
'App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c':'69f6e01e93dbff5b7149f502862b8dea19081f94b5fcfba1573aaae2f19116b6',
}
for rel,want in expected.items(): checks.append((f'unchanged R12R8R2 core: {rel}',sha(rel)==want))
for name,ok in checks: print(('PASS' if ok else 'FAIL')+': '+name)
failed=[name for name,ok in checks if not ok]
if failed:
    print('FAILED:', ', '.join(failed)); sys.exit(1)

# Simple behavioral model of R12R8 freshness (unchanged).
def gate(valid,src,cmd,step,last_step,last_ms,now,seen):
    if (not seen) or step!=last_step: last_step,last_ms,seen=step,now,True
    fresh=seen and (now-last_ms)<=50
    out=cmd if (valid and src and cmd==cmd and fresh) else 0.0
    return max(0.0,min(1.0,out)),last_step,last_ms,seen
ls=0;lm=0;seen=False
out,ls,lm,seen=gate(1,3,.75,100,ls,lm,1000,seen); assert abs(out-.75)<1e-6
out,ls,lm,seen=gate(1,3,.75,100,ls,lm,1060,seen); assert out==0
out,ls,lm,seen=gate(1,3,.80,101,ls,lm,1070,seen); assert abs(out-.80)<1e-6
print('PASS: R12R8 production freshness model')

if shutil.which('gcc'):
    inc=[f'-I{R/"App"}',f'-I{R/"Core/Inc"}',f'-I{R/"Drivers/STM32F4xx_HAL_Driver/Inc"}',f'-I{R/"Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"}',f'-I{R/"Drivers/CMSIS/Device/ST/STM32F4xx/Include"}',f'-I{R/"Drivers/CMSIS/Include"}']
    subprocess.run(['gcc','-std=gnu11','-fsyntax-only','-DSTM32F407xx','-DUSE_HAL_DRIVER',*inc,str(R/'App/Modules/Sensors/Barometer/ms5611_spi.c'),str(R/'App/Modules/Sensors/Barometer/barometer.c')],check=True)
    print('PASS: host gcc syntax-only barometer sources')
    tests=[
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r5_autonomous_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r3_auto'],'/tmp/r8r3_auto'),
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r12r1_same_target_guard_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r3_guard'],'/tmp/r8r3_guard'),
      (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r12r6r2_fault_cause_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/r8r3_fault'],'/tmp/r8r3_fault'),
    ]
    for cmd,out in tests:
        subprocess.run(cmd,cwd=R,check=True); subprocess.run([out],cwd=R,check=True)
    print('PASS: P111 regressions + feedback propagation')
else:
    print('SKIP: host gcc unavailable')

env=dict(os.environ); env['PYTHONPYCACHEPREFIX']='/tmp/r8r3_pycache'
subprocess.run([sys.executable,'-m','py_compile',str(R/'monitor_uart_p112r12r8r3_baro_atomic_cpu_stability.py')],check=True,env=env)
print('PASS: monitor py_compile')
print('NOTE: ARM/Cortex-M4 target compiler/CubeIDE target build unavailable here; target build is NOT claimed.')
print('P112R12R8R3 barometer atomic-burst production-candidate inert validation: PASS')
