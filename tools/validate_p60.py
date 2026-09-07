#!/usr/bin/env python3
from pathlib import Path
import hashlib, os, sys

ROOT = Path(__file__).resolve().parents[1]
TX = Path(os.environ.get('P60_TX_ROOT', str(ROOT.parent/'nrf_verici_2SWITCH_LED_V4_TAHLIYE_ESTOP')))

fails=[]; passes=[]; skips=[]
def ck(name, cond, detail=''):
    (passes if cond else fails).append((name, detail))
def sk(name, detail=''): skips.append((name,detail))
def text(rel): return (ROOT/rel).read_text(errors='replace')
def sha(p):
    h=hashlib.sha256(); h.update(p.read_bytes()); return h.hexdigest()

cfg=text('App/Common/app_config.h'); app=text('App/app.c'); apph=text('App/app.h')
tasks=text('App/Core/Tasks/app_tasks.c'); remote=text('App/Modules/RemoteControl/remote_control.c')
nrf=text('App/Modules/NRF24/nrf24.c'); sol=text('App/Services/SolenoidOutput/solenoid_output.c')
ver=text('App/Common/app_version.h'); gpio=text('Core/Src/gpio.c')
pins=text('App/Platform/board_pins.h'); ioc=text('tgy.ioc'); uart=text('App/Services/UARTTelemetry/uart_telemetry.c')

ck('version P60', 'P60-ESTOP-AUTH-HARDENING' in ver and '53' in ver)
ck('central authorization API declared', 'uint8_t App_IsActuatorAuthorized(void);' in apph)
ck('central authorization implemented', 'uint8_t App_IsActuatorAuthorized(void)' in app)
ck('authorization revokes on STOP', 'if (v30_stop_latched != 0U)' in app and app.find('if (v30_stop_latched != 0U)') < app.find('return 1U;', app.find('uint8_t App_IsActuatorAuthorized')))
ck('authorization requires flight/no preflight fault', 'PreflightTrigger_IsFlightActive() == 0U' in app and 'PreflightTrigger_HasFault() != 0U' in app)
ck('authorization includes actuator fault', 'SystemMonitor_IsActuatorFaultActive() != 0U' in app)
ck('authorization includes ESKF inhibit', '(eskf == 0) || (eskf->output_inhibited != 0U)' in app)
ck('1k RCS uses central authorization', 'if ((App_IsActuatorAuthorized() != 0U) &&' in tasks)
ck('200Hz RCS uses central authorization', '(App_IsActuatorAuthorized() != 0U))' in tasks)
ck('needle physical output uses central authorization', 'if (App_IsActuatorAuthorized() == 0U)' in tasks)
ck('generated physical path uses central authorization', tasks.count('App_IsActuatorAuthorized()') >= 4)
ck('UART actuator_authorized uses central authorization', 'FIELD_U32(App_IsActuatorAuthorized());' in uart)

# P59 radio/vent/STOP behavior must remain intact.
ck('receiver address TGY01', "{'T', 'G', 'Y', '0', '1'}" in nrf)
ck('receiver channel 76', '#define APP_NRF24_CHANNEL                       76U' in cfg)
ck('receiver payload 4', '#define APP_NRF24_PAYLOAD_SIZE                  4U' in cfg)
ck('receiver 1Mbps 0dBm', 'NRF24_RF_SETUP_1MBPS_0DBM  0x06U' in nrf)
ck('receiver protocol magic/checksum', 'REMOTE_PACKET_MAGIC      0xA5U' in remote and 'REMOTE_PACKET_CHECK_XOR  0x5AU' in remote)
ck('vent physical enabled', 'APP_REMOTE_VENT_PHYSICAL_ENABLED          1U' in cfg)
ck('vent ground only', 'APP_REMOTE_VENT_GROUND_ONLY                1U' in cfg)
ck('vent hold confirm', 'APP_REMOTE_VENT_HOLD_CONFIRM_MS          250UL' in cfg)
ck('vent RF freshness', 'APP_REMOTE_VENT_MAX_PACKET_AGE_MS       250UL' in cfg)
ck('STOP latch remains one-way', 'v30_stop_latched = 1U;' in app and 'v30_stop_latch_count++' in app)
ck('STOP final output guards remain', 'AttitudeControl_ForceSafe();' in app and 'SolenoidOutput_ForceSafe();' in app and 'NeedleValveController_Stop();' in app)
ck('LED mapping preserved', all(x in pins for x in ['BOARD_STATUS_LED_PIN','BOARD_LINK_LED_PIN','BOARD_ESTOP_LED_PIN']))

# Verify every P59 file outside the intentionally changed P60 set is byte-identical.
baseline = Path(os.environ.get('P60_P59_BASELINE', '/mnt/data/p60baseline/TGY_V8_19M_P59_NRF_TAHLIYE_ESTOP_INTEGRATED'))
changed = {'.project','App/Common/app_version.h','App/app.h','App/app.c','App/Core/Tasks/app_tasks.c','App/Services/UARTTelemetry/uart_telemetry.c'}
if baseline.exists():
    bad=[]; checked=0
    for p in baseline.rglob('*'):
        if not p.is_file(): continue
        rel=str(p.relative_to(baseline))
        if rel in changed: continue
        q=ROOT/rel
        # New P60 docs/tools are allowed to exist only on output side.
        if not q.exists() or sha(p)!=sha(q): bad.append(rel)
        checked += 1
    ck(f'P59 unchanged files byte-identical ({checked} files)', not bad, ', '.join(bad[:12]))
else:
    sk('P59 byte-integrity comparison', 'baseline path unavailable')

# TX is intentionally unchanged from P59.
if TX.exists():
    txm=(TX/'Core/Src/main.c').read_text(errors='replace'); txh=(TX/'Core/Inc/main.h').read_text(errors='replace')
    ck('TX vent PA1 unchanged', '#define SWITCH_VENT_Pin GPIO_PIN_1' in txh and 'SWITCH_VENT_GPIO_Port GPIOA' in txh)
    ck('TX estop PA0 unchanged', '#define SWITCH_ESTOP_Pin GPIO_PIN_0' in txh and 'SWITCH_ESTOP_GPIO_Port GPIOA' in txh)
    ck('TX semantic mapping unchanged', 'COMMAND_FLAG_SWITCH1        0x01U  /* VENT */' in txm and 'COMMAND_FLAG_ESTOP          0x02U  /* EMERGENCY STOP */' in txm)
else:
    sk('TX validation', 'standalone receiver package; P60 does not modify TX')

print('P60 VALIDATION')
for n,d in passes: print('[PASS]', n + ((' :: '+d) if d else ''))
for n,d in skips: print('[SKIP]', n + ((' :: '+d) if d else ''))
for n,d in fails: print('[FAIL]', n + ((' :: '+d) if d else ''))
print(f'PASS={len(passes)} SKIP={len(skips)} FAIL={len(fails)}')
sys.exit(1 if fails else 0)
