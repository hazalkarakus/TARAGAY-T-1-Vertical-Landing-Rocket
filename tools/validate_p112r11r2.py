#!/usr/bin/env python3
from pathlib import Path
import hashlib,re,sys,shutil,subprocess,tempfile
R=Path(__file__).resolve().parents[1]

def txt(rel): return (R/rel).read_text(errors='ignore')
def has(rel,s): return s in txt(rel)
def sha(rel): return hashlib.sha256((R/rel).read_bytes()).hexdigest()

checks=[]
checks += [
 ('version', has('App/Common/app_version.h','8.19M-P112R11R2-NEEDLE-VALVE-MECHANICAL-COMMISSIONING-950PLUS')),
 ('profile', has('App/Common/app_config.h','#define APP_NEEDLE_P112R11_MECH_COMMISSION_MODE              1U')),
 ('R11R1 motion safety retained', has('App/Common/app_config.h','#define APP_P112R11R1_MOTION_SAFETY_REV                       1U')),
 ('R11R2 950+ revision', has('App/Common/app_config.h','#define APP_P112R11R2_CLOSED_REFERENCE_950PLUS_REV            1U')),
 ('closed reference floor 950', has('App/Modules/Control/NeedleValve/needle_valve_controller.h','#define NEEDLE_VALVE_SAFE_ZERO_MIN_ADC 950U')),
 ('host stub floor 950', has('tools/p112r5_host_stub/Modules/Control/NeedleValve/needle_valve_controller.h','#define NEEDLE_VALVE_SAFE_ZERO_MIN_ADC 950U')),
 ('short travel 78', has('App/Common/app_config.h','#define APP_P112R11_OPEN_TRAVEL_ADC                           78U')),
 ('UART RX remains off guard', has('App/Common/app_config.h','(APP_V55_UART_RX_COMMANDS_ENABLED != 0U)')),
 ('RCS hard-lock retained', has('App/Services/SolenoidOutput/solenoid_output.c','#if (APP_P112R10R3_GNC_MOTOR_BENCH_MODE != 0U)')),
 ('full preflight still required before arming', has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','if (pf.preflight_ready == 0U) return 0U;')),
 ('PE9/connector abort retained', has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R11_ABORT_PE9_OR_CONNECTOR')),
 ('flight-active abort retained', has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R11_ABORT_FLIGHT_ACTIVE')),
 ('P83 feedback abort retained', has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R11_ABORT_FEEDBACK')),
 ('needle-fault abort retained', has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R11_ABORT_NEEDLE_FAULT')),
 ('operator abort retained', has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R11_ABORT_OPERATOR_PA0')),
 ('timeout abort retained', has('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c','P112R11_ABORT_TIMEOUT')),
 ('exact commissioning telemetry appended', has('App/Services/UARTTelemetry/uart_telemetry.c','p112r11_state,p112r11_abort_reason,p112r11_abort_count,p112r11_button_debounced,p112r11_open_target_adc')),
 ('generated private header', (R/'App/Modules/Control/GeneratedFlightControl/Generated/Ucus_Bilgisayari_private.h').exists()),
]

c=txt('App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c')
m=re.search(r'static uint8_t P112R11_MotionAbortReason\(void\)(.*?)static void P112R11_Abort',c,re.S)
seg=m.group(1) if m else ''
forbidden=['pf.imu_ready','pf.eskf_ready','pf.lidar_reference_ready','pf.barometer_reference_ready','pf.sd_ready','pf.preflight_ready']
checks.append(('motion safety ignores unrelated sensor/reference/SD readiness', bool(seg) and not any(x in seg for x in forbidden)))

# Existing field indices must remain stable; exactly five R11R1 fields append after old index 492.
u=txt('App/Services/UARTTelemetry/uart_telemetry.c')
hm=re.search(r'static const char header\[\]\s*=\s*(.*?);\s*uint32_t now_ms',u,re.S)
field_count_ok=False
if hm:
    ss=re.findall(r'"((?:\\.|[^"\\])*)"',hm.group(1))
    header=''.join(bytes(x,'utf-8').decode('unicode_escape') for x in ss)
    names=header.strip().split(',')
    field_count_ok=(len(names)==499 and names[492]=='p112_initial_coast_adc' and names[493:498]==[
        'p112r11_state','p112r11_abort_reason','p112r11_abort_count','p112r11_button_debounced','p112r11_open_target_adc'] and names[498]=='crc16_ccitt')
checks.append(('UART old indices stable + five fields appended',field_count_ok))

expected={
 'App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c':'e21c58cbcf5f751124d7e5dce82f21cf72b0c0a20686d4d041f612983918dbf6',
 'App/Services/NeedleValveHardware/needle_valve_hw.c':'b439781f1fc3923cc4f93528ee42b431b8dc24b04cb82939475d8f6e7d041f4f',
 'App/app.c':'a177c23fb147b2df2ac834df9c433755c9c9055d122bba7616a5d845365982c0',
 'Core/Src/stm32f4xx_it.c':'6d45c65dd78a85ebc595a876f2ac0f62bf619a817779d1f4fd1d20696874969c',
 'App/Services/SDLogger/sd_logger.c':'9fcbf3d9685cdece8d9c2b386aef5d061e98d9cd2b1b16010edd159c6680dc4f',
}
for rel,h in expected.items(): checks.append((f'low-level hash unchanged: {rel}',sha(rel)==h))

failed=[n for n,ok in checks if not ok]
for n,ok in checks: print(('PASS' if ok else 'FAIL')+': '+n)
if failed:
    print('FAILED:', ', '.join(failed)); sys.exit(1)
print('P112R11R2 950+ static commissioning safety/regression checks: PASS')
