#!/usr/bin/env python3
from pathlib import Path
import hashlib,re,subprocess,sys
R=Path(__file__).resolve().parents[1]
def t(rel): return (R/rel).read_text(errors='ignore')
def has(rel,s): return s in t(rel)
def sha(rel): return hashlib.sha256((R/rel).read_bytes()).hexdigest()
checks=[
('version',has('App/Common/app_version.h','8.19M-P112R12-PRODUCTION-AUTONOMOUS-950PLUS-INERT')),
('project name',has('.project','<name>P112R12_PROD_AUTO_950PLUS_INERT</name>')),
('R12 production rev',has('App/Common/app_config.h','#define APP_P112R12_PRODUCTION_AUTONOMOUS_950PLUS_REV          1U')),
('R12 inert output isolation',has('App/Common/app_config.h','#define APP_P112R12_INERT_OUTPUT_ISOLATION_MODE                1U')),
('R10 virtual stimulus disabled',has('App/Common/app_config.h','#define APP_P112R10R3_GNC_MOTOR_BENCH_MODE                 0U')),
('R11 PA0 commissioning disabled',has('App/Common/app_config.h','#define APP_NEEDLE_P112R11_MECH_COMMISSION_MODE              0U')),
('autonomous actuator owner enabled',has('App/Common/app_config.h','#define APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE              1U')),
('UART RX actuator commands off guard',has('App/Common/app_config.h','(APP_V55_UART_RX_COMMANDS_ENABLED != 0U)')),
('950 closed-reference floor',has('App/Modules/Control/NeedleValve/needle_valve_controller.h','#define NEEDLE_VALVE_SAFE_ZERO_MIN_ADC 950U')),
('950 host floor',has('tools/p112r12_host_stub/Modules/Control/NeedleValve/needle_valve_controller.h','#define NEEDLE_VALVE_SAFE_ZERO_MIN_ADC 950U')),
('production GNC submit route',has('App/Core/Tasks/app_tasks.c','NeedleValveAutonomousControl_SubmitCommand(command)')),
('commissioning branch compile-time off',has('App/Core/Tasks/app_tasks.c','#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)')),
('real ESKF GNC branch retained',has('App/Modules/Control/GeneratedFlightControl/generated_flight_control.c','height_m = eskf->lidar_reference_m + eskf->position_z_m;')),
('RCS hard lock includes R12',has('App/Services/SolenoidOutput/solenoid_output.c','APP_P112R12_INERT_OUTPUT_ISOLATION_MODE')),
('ground vent hard lock includes R12',t('App/Services/SolenoidOutput/solenoid_output.c').count('APP_P112R12_INERT_OUTPUT_ISOLATION_MODE')>=2),
('vent servo hard lock includes R12',has('App/Services/ServoOutput/servo_output.c','#if (APP_P112R12_INERT_OUTPUT_ISOLATION_MODE != 0U)')),
('R10R3 settled same-target hold retained',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','#define P112R5_SETTLED_HOLD_DRIFT_ADC             8U')),
('R10R2 correction guard retained',has('App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c','#define P110_CORRECTION_QUANTUM_MAX_ADC           20U')),
('generated private header packaged',(R/'App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_private.h').is_file()),
('do-not-fly marker',(R/'P112R12_DO_NOT_FLY.txt').is_file()),
]
# Main production low-level control, timing and SD path must be byte-identical to the mechanically validated R11R2 base.
expected={
'App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c':'e21c58cbcf5f751124d7e5dce82f21cf72b0c0a20686d4d041f612983918dbf6',
'App/Services/NeedleValveHardware/needle_valve_hw.c':'b439781f1fc3923cc4f93528ee42b431b8dc24b04cb82939475d8f6e7d041f4f',
'App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c':'adfbeed68b7e75116463a365e702d61d1bf42514f6f7b9574b4850c0e93d97f2',
'App/Core/Tasks/app_tasks.c':'61ed98b1764a54aa51c46be07957083d8456d98c9aa96fd873f9e65ca761adb0',
'App/Modules/Control/GeneratedFlightControl/generated_flight_control.c':'df2b24d4942a5917504b97749add4a926eba362db7893130364ab5d349b41694',
'App/app.c':'a177c23fb147b2df2ac834df9c433755c9c9055d122bba7616a5d845365982c0',
'Core/Src/stm32f4xx_it.c':'6d45c65dd78a85ebc595a876f2ac0f62bf619a817779d1f4fd1d20696874969c',
'App/Services/SDLogger/sd_logger.c':'9fcbf3d9685cdece8d9c2b386aef5d061e98d9cd2b1b16010edd159c6680dc4f',
'App/Services/UARTTelemetry/uart_telemetry.c':'db3168b54aed8c12e16c0a0f80baf15fde56b2d87facd1c5c80da8937a18cab4',
}
for rel,h in expected.items(): checks.append((f'validated core unchanged: {rel}',sha(rel)==h))
# R12 must not accidentally compile physical RCS on in this profile.
cfg=t('App/Common/app_config.h')
checks.append(('RCS macro is conditioned by inert isolation', 'APP_P112R12_INERT_OUTPUT_ISOLATION_MODE != 0U' in cfg and 'APP_V49_ESKF_RCS_PHYSICAL_ENABLED' in cfg))
for name,ok in checks: print(('PASS' if ok else 'FAIL')+': '+name)
failed=[name for name,ok in checks if not ok]
if failed:
 print('FAILED:',', '.join(failed)); sys.exit(1)
# Host regression of autonomous supervisor and settled-target hold with commissioning compiled out.
cmd1=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r5_autonomous_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/p112r12_auto']
subprocess.run(cmd1,cwd=R,check=True); subprocess.run(['/tmp/p112r12_auto'],cwd=R,check=True)
cmd2=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp','tools/p112r10r3_same_target_hold_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o','/tmp/p112r12_hold']
subprocess.run(cmd2,cwd=R,check=True); subprocess.run(['/tmp/p112r12_hold'],cwd=R,check=True)
print('P112R12 production autonomous 950+ inert static/host validation: PASS')
