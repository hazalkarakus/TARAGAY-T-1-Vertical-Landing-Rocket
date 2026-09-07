#!/usr/bin/env python3
from pathlib import Path
import hashlib, subprocess, sys
R=Path(__file__).resolve().parents[1]
def text(rel): return (R/rel).read_text(errors='ignore')
def has(rel,s): return s in text(rel)
def sha(rel): return hashlib.sha256((R/rel).read_bytes()).hexdigest()
checks=[
('version',has('App/Common/app_version.h','8.19M-P112R12R1-PRODUCTION-AUTONOMOUS-950PLUS-INERT-SAME-TARGET-GUARD')),
('project name',has('.project','<name>P112R12R1_PROD_AUTO_950PLUS_INERT_GUARD</name>')),
('R12 production profile retained',has('App/Common/app_config.h','#define APP_P112R12_PRODUCTION_AUTONOMOUS_950PLUS_REV          1U')),
('R12R1 guard rev',has('App/Common/app_config.h','#define APP_P112R12R1_SAME_TARGET_GUARD_REV                     1U')),
('R12 inert isolation retained',has('App/Common/app_config.h','#define APP_P112R12_INERT_OUTPUT_ISOLATION_MODE                1U')),
('R10 stimulus disabled',has('App/Common/app_config.h','#define APP_P112R10R3_GNC_MOTOR_BENCH_MODE                 0U')),
('R11 commissioning disabled',has('App/Common/app_config.h','#define APP_NEEDLE_P112R11_MECH_COMMISSION_MODE              0U')),
('950 closed floor',has('App/Modules/Control/NeedleValve/needle_valve_controller.h','#define NEEDLE_VALVE_SAFE_ZERO_MIN_ADC 950U')),
('same target firm hold 12',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','#define P112R12R1_SAME_TARGET_HOLD_ADC            12U')),
('same target deadband 20',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','#define P112R12R1_SAME_TARGET_DEADBAND_ADC        20U')),
('same target confirm 200ms',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','#define P112R12R1_SAME_TARGET_CONFIRM_MS         200UL')),
('new target clears guard immediately',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R5_AbsDiffU16(target, previous_target) >')),
('persistent excursion timing',has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','now - autonomous_same_target_excursion_since_ms')),
('production GNC submit route',has('App/Core/Tasks/app_tasks.c','NeedleValveAutonomousControl_SubmitCommand(command)')),
('RCS hard isolation retained',has('App/Services/SolenoidOutput/solenoid_output.c','APP_P112R12_INERT_OUTPUT_ISOLATION_MODE')),
('vent servo hard isolation retained',has('App/Services/ServoOutput/servo_output.c','#if (APP_P112R12_INERT_OUTPUT_ISOLATION_MODE != 0U)')),
('generated private header packaged',(R/'App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_private.h').is_file()),
('do not fly marker',(R/'P112R12R1_DO_NOT_FLY.txt').is_file()),
]
# Everything below P111 and all production GNC/timing/logging/output-isolation files
# must remain byte-identical to P112R12.
expected={
'App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c':'e21c58cbcf5f751124d7e5dce82f21cf72b0c0a20686d4d041f612983918dbf6',
'App/Services/NeedleValveHardware/needle_valve_hw.c':'b439781f1fc3923cc4f93528ee42b431b8dc24b04cb82939475d8f6e7d041f4f',
'App/Core/Tasks/app_tasks.c':'61ed98b1764a54aa51c46be07957083d8456d98c9aa96fd873f9e65ca761adb0',
'App/Modules/Control/GeneratedFlightControl/generated_flight_control.c':'df2b24d4942a5917504b97749add4a926eba362db7893130364ab5d349b41694',
'App/app.c':'a177c23fb147b2df2ac834df9c433755c9c9055d122bba7616a5d845365982c0',
'Core/Src/stm32f4xx_it.c':'6d45c65dd78a85ebc595a876f2ac0f62bf619a817779d1f4fd1d20696874969c',
'App/Services/SDLogger/sd_logger.c':'9fcbf3d9685cdece8d9c2b386aef5d061e98d9cd2b1b16010edd159c6680dc4f',
'App/Services/UARTTelemetry/uart_telemetry.c':'db3168b54aed8c12e16c0a0f80baf15fde56b2d87facd1c5c80da8937a18cab4',
'App/Services/SolenoidOutput/solenoid_output.c':'8915eb3f0246ddeb555f53fce73d1212f4a8a750611b6f8f7259c5c3d66ea901',
'App/Services/ServoOutput/servo_output.c':'c67feac970817ea4a5086a8fca669ca4c1674de82ba08264a60f76099e0bb95e',
}
for rel,want in expected.items(): checks.append((f'R12 core unchanged: {rel}',sha(rel)==want))
for name,ok in checks: print(('PASS' if ok else 'FAIL')+': '+name)
failed=[n for n,o in checks if not o]
if failed:
    print('FAILED:',', '.join(failed)); sys.exit(1)
# Base P111 regression: real retarget/movement/fault behavior still valid.
cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp',
     'tools/p112r5_autonomous_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c',
     '-o','/tmp/p112r12r1_auto']
subprocess.run(cmd,cwd=R,check=True); subprocess.run(['/tmp/p112r12r1_auto'],cwd=R,check=True)
# New R12R1 behavior regression.
cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r12_host_stub','-IApp',
     'tools/p112r12r1_same_target_guard_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c',
     '-o','/tmp/p112r12r1_guard']
subprocess.run(cmd,cwd=R,check=True); subprocess.run(['/tmp/p112r12r1_guard'],cwd=R,check=True)
print('P112R12R1 production autonomous same-target mechanical guard validation: PASS')
