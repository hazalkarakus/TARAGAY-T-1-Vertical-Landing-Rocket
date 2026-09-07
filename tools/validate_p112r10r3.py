#!/usr/bin/env python3
from pathlib import Path
import hashlib, subprocess

ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text(encoding='utf-8')
def req(t,x,label):
    if x not in t: raise AssertionError(label)
def sha(p): return hashlib.sha256((ROOT/p).read_bytes()).hexdigest()

cfg=read('App/Common/app_config.h')
ver=read('App/Common/app_version.h')
proj=read('.project'); cp=read('.cproject')
tasks=read('App/Core/Tasks/app_tasks.c')
gfc=read('App/Modules/Control/GeneratedFlightControl/generated_flight_control.c')
sol=read('App/Services/SolenoidOutput/solenoid_output.c')
auto=read('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c')
hw=read('App/Services/NeedleValveHardware/needle_valve_hw.c')
ad=read('App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c')
irq=read('Core/Src/stm32f4xx_it.c'); app=read('App/app.c'); sd=read('App/Services/SDLogger/sd_logger.c')

checks=[
(cfg,'#define APP_P112R10R3_GNC_MOTOR_BENCH_MODE                 1U','profile'),
(cfg,'#define APP_P112R10R3_STIMULUS_COMMAND_CAP                 0.30f','stim cap'),
(cfg,'#define APP_V55_UART_RX_COMMANDS_ENABLED                0U','TX only'),
(cfg,'#define APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE              1U','autonomous ownership'),
(ver,'8.19M-P112R10R3-GNC-MOTOR-BENCH-SETTLED-TARGET-HOLD','version'),
(proj,'TGY_V8_19M_P112R10R3_GNC_MOTOR_BENCH_SETTLED_TARGET_HOLD','project'),
(cp,'TGY_V8_19M_P112R10R3_GNC_MOTOR_BENCH_SETTLED_TARGET_HOLD','cproject'),
(gfc,'height_m = APP_P112R10R3_VIRTUAL_HEIGHT_M;','virtual GNC'),
(tasks,'APP_P112R10R3_STIMULUS_COMMAND_CAP','stimulus'),
(tasks,'NeedleValveAutonomousControl_SubmitCommand(command)','P111 route'),
(sol,'#if (APP_P112R10R3_GNC_MOTOR_BENCH_MODE != 0U)','solenoid lock'),
(sol,'valve_mask = SOLENOID_VALVE_NONE;','RCS lock'),
(sol,'sanitized_mask = SOLENOID_VALVE_NONE;','vent lock'),
(auto,'#define P112R5_SETTLED_HOLD_DRIFT_ADC             8U','hold drift'),
(auto,'autonomous_settled_hold_valid','hold latch'),
(auto,'autonomous_settled_hold_target_adc = p110_target_adc;','completed target latch'),
(auto,'autonomous_settled_hold_position_adc = p83_filtered_adc;','settled position latch'),
(auto,'P112R5_AbsDiffU16(target,','same target comparison'),
(auto,'autonomous_settled_hold_position_adc) <=','drift comparison'),
(ad,'#define P110_TARGET_TOL_ADC                        8U','P110 nominal tol'),
(ad,'#define P110_SETTLED_ACCEPT_ADC                   12U','P110 settled guard'),
(ad,'#define P110_CORRECTION_QUANTUM_MAX_ADC           20U','P110 adaptive quantum'),
(ad,'#define P110_HARD_OVERSHOOT_ADC                   12U','P110 hard overshoot'),
(irq,'NeedleValveAutonomousControl_TimerTickISR();','TIM7'),
(app,'NVIC_DisableIRQ(TIM7_IRQn);','SD boot isolation'),
(sd,'SDLogger_Update','runtime SD'),
]
for t,x,l in checks: req(t,x,l)

# Generated private header must be packaged; this was the prior CubeIDE import/build failure point.
priv=ROOT/'App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_private.h'
if not priv.is_file(): raise AssertionError('generated private header missing')

# Bench profile must not bypass motor HW ownership inside autonomous/hardware layers.
if 'APP_P112R10R3_GNC_MOTOR_BENCH_MODE' in auto:
    raise AssertionError('bench-profile bypass inside P111 supervisor')
if 'APP_P112R10R3_GNC_MOTOR_BENCH_MODE' in hw:
    raise AssertionError('bench-profile bypass inside needle HW')

# R10R3 changes P111 only among low-level actuator control layers. R10R2 adaptive
# controller, HW, TIM7 ISR, app SD-boot isolation and logger remain byte-identical.
unchanged={
 'App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c':'e21c58cbcf5f751124d7e5dce82f21cf72b0c0a20686d4d041f612983918dbf6',
 'App/Services/NeedleValveHardware/needle_valve_hw.c':'b439781f1fc3923cc4f93528ee42b431b8dc24b04cb82939475d8f6e7d041f4f',
 'App/app.c':'a177c23fb147b2df2ac834df9c433755c9c9055d122bba7616a5d845365982c0',
 'Core/Src/stm32f4xx_it.c':'6d45c65dd78a85ebc595a876f2ac0f62bf619a817779d1f4fd1d20696874969c',
 'App/Services/SDLogger/sd_logger.c':'9fcbf3d9685cdece8d9c2b386aef5d061e98d9cd2b1b16010edd159c6680dc4f',
}
for rel,want in unchanged.items():
    got=sha(rel)
    if got!=want: raise AssertionError(f'unexpected low-level change {rel}: {got}')

# Base autonomous supervisor regression.
bin1=Path('/tmp/p112r10r3_autonomous_host_test')
subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r5_host_stub','-IApp',
 'tools/p112r5_autonomous_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o',str(bin1)],cwd=ROOT,check=True)
subprocess.run([str(bin1)],cwd=ROOT,check=True)

# R10R3 regression: one OPEN PASS + repeated same target must not start another
# move; a genuine CLOSE target must start exactly one new move; drift releases hold.
bin2=Path('/tmp/p112r10r3_same_target_hold_host_test')
subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-Itools/p112r5_host_stub','-IApp',
 'tools/p112r10r3_same_target_hold_host_test.c','App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','-o',str(bin2)],cwd=ROOT,check=True)
subprocess.run([str(bin2)],cwd=ROOT,check=True)

print('P112R10R3 static settled-target hold architecture: PASS')
print('R10R2 P110 adaptive correction layer byte-identical: PASS')
print('Repeated same GNC target after P110 PASS -> no retrigger: PASS')
print('Genuine target change -> normal CLOSE request: PASS')
print('Settled-position drift >8 ADC -> closed-loop hold releases: PASS')
print('RCS + vent hard-lock and SD/TIM7 regression: PASS')
print('Generated Ucus_Bilgisayari_private.h packaged: PASS')
