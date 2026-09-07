#!/usr/bin/env python3
from pathlib import Path
import hashlib, os, sys

ROOT = Path(__file__).resolve().parents[1]
TX = Path(os.environ.get('P59_TX_ROOT', str(ROOT.parent/'nrf_verici_2SWITCH_LED_V4_TAHLIYE_ESTOP')))

fails=[]; passes=[]; skips=[]
def ck(name, cond, detail=''):
    (passes if cond else fails).append((name, detail))
def sk(name, detail=''): skips.append((name,detail))
def text(rel): return (ROOT/rel).read_text(errors='replace')
def sha(p):
    h=hashlib.sha256(); h.update(p.read_bytes()); return h.hexdigest()

cfg=text('App/Common/app_config.h'); app=text('App/app.c')
tasks=text('App/Core/Tasks/app_tasks.c'); remote=text('App/Modules/RemoteControl/remote_control.c')
nrf=text('App/Modules/NRF24/nrf24.c'); sol=text('App/Services/SolenoidOutput/solenoid_output.c')
ver=text('App/Common/app_version.h'); gpio=text('Core/Src/gpio.c')
pins=text('App/Platform/board_pins.h'); ioc=text('tgy.ioc')

ck('version P59', 'P59-NRF-TAHLIYE-ESTOP' in ver and '52' in ver)
ck('receiver address TGY01', "{'T', 'G', 'Y', '0', '1'}" in nrf)
ck('receiver channel 76', '#define APP_NRF24_CHANNEL                       76U' in cfg)
ck('receiver payload 4', '#define APP_NRF24_PAYLOAD_SIZE                  4U' in cfg)
ck('receiver 1Mbps 0dBm', 'NRF24_RF_SETUP_1MBPS_0DBM  0x06U' in nrf)
ck('receiver protocol magic/checksum', 'REMOTE_PACKET_MAGIC      0xA5U' in remote and 'REMOTE_PACKET_CHECK_XOR  0x5AU' in remote)
ck('invalid reason counters', all(x in remote for x in ['remote_rx_bad_magic_count','remote_rx_bad_flags_count','remote_rx_bad_checksum_count']))

ck('vent physical enabled', 'APP_REMOTE_VENT_PHYSICAL_ENABLED          1U' in cfg)
ck('vent ground only', 'APP_REMOTE_VENT_GROUND_ONLY                1U' in cfg)
ck('vent hold confirm', 'APP_REMOTE_VENT_HOLD_CONFIRM_MS          250UL' in cfg)
ck('vent fast RF freshness', 'APP_REMOTE_VENT_MAX_PACKET_AGE_MS       250UL' in cfg and 'remote_rx_last_packet_age_ms <= APP_REMOTE_VENT_MAX_PACKET_AGE_MS' in app)
ck('vent one-hot sequential masks', all(x in app for x in ['SOLENOID_VALVE_ROLL_POS_ERROR','SOLENOID_VALVE_ROLL_NEG_ERROR','SOLENOID_VALVE_PITCH_POS_ERROR','SOLENOID_VALVE_PITCH_NEG_ERROR']))
ck('vent one-hot enforcement', '(sanitized_mask & (uint8_t)(sanitized_mask - 1U))' in sol)
ck('vent fault/flight fail closed', 'PreflightTrigger_IsFlightActive()' in sol and 'PreflightTrigger_HasFault()' in sol and 'SystemMonitor_IsActuatorFaultActive()' in sol)
ck('vent override protects ownership', tasks.count('v30_vent_override_active') >= 5)

ck('STOP latch', 'v30_stop_latched = 1U;' in app and 'v30_stop_latch_count++' in app)
ck('STOP forces RCS safe', 'AttitudeControl_ForceSafe();' in app and 'SolenoidOutput_ForceSafe();' in app)
ck('STOP stops needle', 'NeedleValveController_Stop();' in app)
ck('STOP gates 1k/200Hz', tasks.count('v30_stop_latched') >= 4)
ck('STOP priority before vent', app.find('if (v30_stop_latched != 0U)') < app.find('if (v30_switch1_vent_request == 0U)'))
ck('LED PD12/13/14', all(x in pins for x in ['BOARD_STATUS_LED_PIN','BOARD_LINK_LED_PIN','BOARD_ESTOP_LED_PIN']) and 'PD13.GPIO_Label=LINK_LED' in ioc and 'PD14.GPIO_Label=ESTOP_LED' in ioc)
ck('Core GPIO initializes LEDs safe', 'STATUS_LED_Pin | LINK_LED_Pin | ESTOP_LED_Pin' in gpio)

# Portable P57 critical-subsystem integrity verification.
manifest=ROOT/'P57_CRITICAL_BASELINE_SHA256.txt'
if manifest.exists():
    bad=[]; checked=0
    for line in manifest.read_text().splitlines():
        if not line.strip(): continue
        expected, rel=line.split(None,1); rel=rel.strip(); p=ROOT/rel; checked+=1
        if (not p.exists()) or sha(p)!=expected: bad.append(rel)
    ck(f'P57 critical sources byte-identical ({checked} files)', not bad, ', '.join(bad[:12]))
else:
    ck('P57 critical baseline manifest present', False)

# TX validation is automatic in the combined bundle; standalone receiver package skips it.
if TX.exists():
    def tt(rel): return (TX/rel).read_text(errors='replace')
    txm=tt('Core/Src/main.c'); txh=tt('Core/Inc/main.h'); txg=tt('Core/Src/gpio.c'); txn=tt('Core/Src/nrf24l01p.c'); txioc=tt('nrf_verici.ioc')
    ck('TX vent PA1', '#define SWITCH_VENT_Pin GPIO_PIN_1' in txh and 'SWITCH_VENT_GPIO_Port GPIOA' in txh)
    ck('TX estop PA0', '#define SWITCH_ESTOP_Pin GPIO_PIN_0' in txh and 'SWITCH_ESTOP_GPIO_Port GPIOA' in txh)
    ck('TX semantic bit mapping', 'COMMAND_FLAG_SWITCH1        0x01U  /* VENT */' in txm and 'COMMAND_FLAG_ESTOP          0x02U  /* EMERGENCY STOP */' in txm)
    ck('TX address TGY01', "{'T', 'G', 'Y', '0', '1'}" in txm)
    ck('TX channel/payload', 'hnrf24.Channel = 76U;' in txm and 'hnrf24.PayloadSize = 4U;' in txm)
    ck('TX RF setup/CRC/autoack', 'NRF24_WriteReg(hnrf, NRF_RF_SETUP, 0x06U)' in txn and 'NRF24_WriteReg(hnrf, NRF_EN_AA, 0x01U)' in txn and 'NRF24_WriteReg(hnrf, NRF_CONFIG, 0x0CU)' in txn)
    ck('TX packet matches receiver', 'PACKET_MAGIC                0xA5U' in txm and 'PACKET_CHECK_XOR            0x5AU' in txm)
    ck('TX GPIO pullups', 'SWITCH_VENT_Pin' in txg and 'GPIO_PULLUP' in txg and 'SWITCH_ESTOP_Pin' in txg)
    ck('TX IOC pin mapping', 'PA0.GPIO_Label=SWITCH_ESTOP' in txioc and 'PA1.GPIO_Label=SWITCH_VENT' in txioc)
else:
    sk('TX project validation', 'standalone receiver package; set P59_TX_ROOT or use combined bundle')

print('P59 VALIDATION')
for n,d in passes: print('[PASS]', n + ((' :: '+d) if d else ''))
for n,d in skips: print('[SKIP]', n + ((' :: '+d) if d else ''))
for n,d in fails: print('[FAIL]', n + ((' :: '+d) if d else ''))
print(f'PASS={len(passes)} SKIP={len(skips)} FAIL={len(fails)}')
sys.exit(1 if fails else 0)
