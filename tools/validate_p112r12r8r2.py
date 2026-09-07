#!/usr/bin/env python3
from pathlib import Path
import hashlib,subprocess,sys,shutil
R=Path(__file__).resolve().parents[1]
def t(rel): return (R/rel).read_text(errors='ignore')
def has(rel,s): return s in t(rel)
def lacks(rel,s): return s not in t(rel)
def sha(rel): return hashlib.sha256((R/rel).read_bytes()).hexdigest()
checks=[
('version',has('App/Common/app_version.h','8.19M-P112R12R8R2-LIDAR-BUS-CLEAR-PROD-CANDIDATE-INERT')),
('project name',has('.project','<name>P112R12R8R2_LIDAR_BUS_CLEAR_PROD_CANDIDATE_INERT</name>')),
('R12 clean production retained',has('App/Common/app_config.h','#define APP_P112R12R3_CLEAN_PRODUCTION_CHAIN_REV                 1U')),
('R6R2 propagation retained',has('App/Common/app_config.h','#define APP_P112R12R6R2_FAULT_CAUSE_PROPAGATION_REV                1U')),
('R12R8 candidate rev',has('App/Common/app_config.h','#define APP_P112R12R8_PRODUCTION_CANDIDATE_REV                      1U')),
('freshness guard enabled',has('App/Common/app_config.h','#define APP_P112R12R8_GNC_FRESHNESS_GUARD_ENABLED                   1U')),
('freshness max 50ms',has('App/Common/app_config.h','#define APP_P112R12R8_GNC_MAX_AGE_MS                               50UL')),
('inert output isolation',has('App/Common/app_config.h','#define APP_P112R12_INERT_OUTPUT_ISOLATION_MODE                     1U')),
('UART RX disabled',has('App/Common/app_config.h','#define APP_V55_UART_RX_COMMANDS_ENABLED                0U')),
('R10 stimulus disabled',has('App/Common/app_config.h','#define APP_P112R10R3_GNC_MOTOR_BENCH_MODE                 0U')),
('R11 commissioning disabled',has('App/Common/app_config.h','#define APP_NEEDLE_P112R11_MECH_COMMISSION_MODE              0U')),
('no R12R5 stimulus macro',lacks('App/Common/app_config.h','APP_P112R12R5_GNC_DYNAMIC_TRACKING_MODE')),
('no R12R6 injection macro',lacks('App/Common/app_config.h','APP_P112R12R6_FAULT_REVOKE_FEEDBACK_MODE')),
('no R12R7 test macro',lacks('App/Common/app_config.h','APP_P112R12R7_GNC_FAILSAFE_NO_MOTION_MODE')),
('no virtual GNC stimulus source',lacks('App/Modules/Control/GeneratedFlightControl/generated_flight_control.c','P112R12R5')),
('no R12R7 GFC freeze state',lacks('App/Core/Tasks/app_tasks.c','P112R12R7_STALE_FREEZE')),
('no R12R6 fault injection state',lacks('App/Core/Tasks/app_tasks.c','P112R12R6_REVOKE_HOLD')),
('production step-count freshness',has('App/Core/Tasks/app_tasks.c','main_control.step_count != v55_gnc_last_step_count')),
('production stale age gate',has('App/Core/Tasks/app_tasks.c','APP_P112R12R8_GNC_MAX_AGE_MS')),
('freshness reset on revoke',has('App/Core/Tasks/app_tasks.c','v55_gnc_step_seen = 0U;')),
('production submit route',has('App/Core/Tasks/app_tasks.c','NeedleValveAutonomousControl_SubmitCommand(command)')),
('public feedback abort code',has('App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.h','#define NEEDLE_ADAPTIVE_ABORT_FEEDBACK_INVALID  1U')),
('P111 feedback abort mapping',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','p110_abort_reason == NEEDLE_ADAPTIVE_ABORT_FEEDBACK_INVALID')),
('P111 FEEDBACK latch',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R5_LatchFault(NEEDLE_AUTONOMOUS_FAULT_FEEDBACK);')),
('other P110 failures low-level',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R5_LatchFault(NEEDLE_AUTONOMOUS_FAULT_LOW_LEVEL);')),
('950 floor',has('App/Modules/Control/NeedleValve/needle_valve_controller.h','#define NEEDLE_VALVE_SAFE_ZERO_MIN_ADC 950U')),
('no 255 kick',lacks('App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c','P110_BREAKAWAY_KICK_PWM')),
('RCS hard isolation',has('App/Services/SolenoidOutput/solenoid_output.c','APP_P112R12_INERT_OUTPUT_ISOLATION_MODE')),
('vent hard isolation',has('App/Services/ServoOutput/servo_output.c','#if (APP_P112R12_INERT_OUTPUT_ISOLATION_MODE != 0U)')),
('do not fly',(R/'P112R12R8R2_DO_NOT_FLY.txt').is_file()),
('start doc',(R/'00_START_HERE_P112R12R8R2_TR.md').is_file()),
('monitor',(R/'monitor_uart_p112r12r8r2_lidar_bus_clear.py').is_file()),
('bus clear helper',has('App/Modules/Sensors/Lidar/Lidar.c','static uint8_t Lidar_BusClearGPIO(void)')),
('nine clock bound',has('App/Modules/Sensors/Lidar/Lidar.c','#define LIDAR_BUS_CLEAR_CLOCKS         9U')),
('GPIO SCL/SDA pins',has('App/Modules/Sensors/Lidar/Lidar.c','GPIO_PIN_10 | GPIO_PIN_11')),
('bus clear before MX init',t('App/Modules/Sensors/Lidar/Lidar.c').find('Lidar_BusClearGPIO()') < t('App/Modules/Sensors/Lidar/Lidar.c').find('MX_I2C2_Init();', t('App/Modules/Sensors/Lidar/Lidar.c').find('case LIDAR_RECOVERY_STEP_HOST_RESET'))),
('I2C2 peripheral reset',has('App/Modules/Sensors/Lidar/Lidar.c','__HAL_RCC_I2C2_FORCE_RESET();')),
('do not init while bus stuck',has('App/Modules/Sensors/Lidar/Lidar.c','if (Lidar_BusClearGPIO() == 0U)')), 
]
# Critical low-level runtime files remain byte-identical to the physically exercised R12R3 clean core.
expected={
'App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c':'e21c58cbcf5f751124d7e5dce82f21cf72b0c0a20686d4d041f612983918dbf6',
'App/Services/NeedleValveHardware/needle_valve_hw.c':'b439781f1fc3923cc4f93528ee42b431b8dc24b04cb82939475d8f6e7d041f4f',
'App/Modules/Control/GeneratedFlightControl/generated_flight_control.c':'df2b24d4942a5917504b97749add4a926eba362db7893130364ab5d349b41694',
'App/app.c':'a177c23fb147b2df2ac834df9c433755c9c9055d122bba7616a5d845365982c0',
'Core/Src/stm32f4xx_it.c':'6d45c65dd78a85ebc595a876f2ac0f62bf619a817779d1f4fd1d20696874969c',
'App/Services/SDLogger/sd_logger.c':'9fcbf3d9685cdece8d9c2b386aef5d061e98d9cd2b1b16010edd159c6680dc4f',
'App/Services/UARTTelemetry/uart_telemetry.c':'db3168b54aed8c12e16c0a0f80baf15fde56b2d87facd1c5c80da8937a18cab4',
'App/Services/SolenoidOutput/solenoid_output.c':'8915eb3f0246ddeb555f53fce73d1212f4a8a750611b6f8f7259c5c3d66ea901',
'App/Services/ServoOutput/servo_output.c':'c67feac970817ea4a5086a8fca669ca4c1674de82ba08264a60f76099e0bb95e',
}
for rel,want in expected.items(): checks.append((f'unchanged core: {rel}',sha(rel)==want))
for n,o in checks: print(('PASS' if o else 'FAIL')+': '+n)
failed=[n for n,o in checks if not o]
if failed:
 print('FAILED:',', '.join(failed));sys.exit(1)
# Production freshness behavior model: fresh forwards, >50 ms stale forces CLOSED/0,
# a new step recovers, invalid/source-less/NaN forces CLOSED/0.
def gate(valid,src,cmd,step,last_step,last_ms,now,seen):
 if (not seen) or step!=last_step:
  last_step,last_ms,seen=step,now,True
 fresh=seen and (now-last_ms)<=50
 out=cmd if (valid and src and cmd==cmd and fresh) else 0.0
 return max(0.0,min(1.0,out)),last_step,last_ms,seen
ls=0;lm=0;seen=False
out,ls,lm,seen=gate(1,3,0.75,100,ls,lm,1000,seen); assert abs(out-.75)<1e-6
out,ls,lm,seen=gate(1,3,0.75,100,ls,lm,1040,seen); assert out>.7
out,ls,lm,seen=gate(1,3,0.75,100,ls,lm,1060,seen); assert out==0
out,ls,lm,seen=gate(1,3,0.80,101,ls,lm,1070,seen); assert abs(out-.80)<1e-6
out,ls,lm,seen=gate(0,0,0.0,101,ls,lm,1080,seen); assert out==0
print('PASS: R12R8 production freshness model (fresh -> stale safe0 -> recover -> invalid safe0)')
if shutil.which('gcc'):
 cmds=[
  (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r5_autonomous_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/p112r12r8_auto'],'/tmp/p112r12r8_auto'),
  (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r12r1_same_target_guard_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/p112r12r8_guard'],'/tmp/p112r12r8_guard'),
  (['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r12r6r2_fault_cause_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/p112r12r8_fault'],'/tmp/p112r12r8_fault'),
 ]
 for cmd,out in cmds:
  subprocess.run(cmd,cwd=R,check=True);subprocess.run([out],cwd=R,check=True)
 print('PASS: P111 regressions + feedback root-cause propagation')
else: print('SKIP: host gcc unavailable')
env=dict(__import__('os').environ); env['PYTHONPYCACHEPREFIX']='/tmp/p112r12r8_pycache'
subprocess.run([sys.executable,'-m','py_compile',str(R/'monitor_uart_p112r12r8r2_lidar_bus_clear.py')],check=True,env=env)
print('PASS: monitor py_compile')
print('NOTE: ARM/Cortex-M4 target compiler/CubeIDE target build unavailable here; target build is NOT claimed.')
print('P112R12R8R2 LiDAR bus-clear production-candidate inert static/host validation: PASS')
