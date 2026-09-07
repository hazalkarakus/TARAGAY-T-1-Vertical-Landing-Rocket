from pathlib import Path
import hashlib, sys

ROOT = Path(__file__).resolve().parents[1]
BASE = Path('/mnt/data/p63work/p60/TGY_V8_19M_P60_ESTOP_AUTH_HARDENING')
checks=[]
def ok(name, cond):
    checks.append((name, bool(cond)))

nrf=(ROOT/'App/Modules/NRF24/nrf24.c').read_text()
tlm=(ROOT/'App/Services/NRFTelemetry/nrf_telemetry.c').read_text()
remote=(ROOT/'App/Modules/RemoteControl/remote_control.c').read_bytes()
app_tasks=(ROOT/'App/Core/Tasks/app_tasks.c').read_bytes()

ok('P60 static FEATURE=0 retained', 'NRF24_WriteReg(NRF24_REG_FEATURE, 0x00U);' in nrf)
ok('P60 static DYNPD=0 retained', 'NRF24_WriteReg(NRF24_REG_DYNPD, 0x00U);' in nrf)
ok('No ACK-payload command', 'W_ACK_PAYLOAD' not in nrf and 'WriteAckPayload' not in nrf and 'WriteAckPayload' not in tlm)
ok('Explicit async downlink present', 'NRF24_AsyncTxStart' in nrf and 'NRF24_AsyncTxService' in nrf)
ok('Telemetry is 32 bytes', 'NRF_TELEMETRY_PACKET_SIZE' in tlm and 'TlmCrc16(out, 30U)' in tlm)
ok('19 telemetry pages retained', 'NRF_TELEMETRY_PAGE_COUNT' in (ROOT/'App/Services/NRFTelemetry/nrf_telemetry.h').read_text())
ok('Command parser unchanged from P60', BASE.exists() and hashlib.sha256(remote).digest() == hashlib.sha256((BASE/'App/Modules/RemoteControl/remote_control.c').read_bytes()).digest())
ok('Core scheduler/tasks unchanged from P60', BASE.exists() and hashlib.sha256(app_tasks).digest() == hashlib.sha256((BASE/'App/Core/Tasks/app_tasks.c').read_bytes()).digest())

# Runtime async section specifically must not add HAL_Delay.
a=nrf.index('uint8_t NRF24_AsyncTxStart')
b=nrf.index('void NRF24_Update(void)')
ok('Async TDD runtime path has no HAL_Delay', 'HAL_Delay(' not in nrf[a:b])

failed=[n for n,v in checks if not v]
for n,v in checks:
    print(('PASS' if v else 'FAIL') + ' - ' + n)
if failed:
    print(f'P64 validation FAIL ({len(failed)} failures)')
    sys.exit(1)
print(f'P64 validation PASS ({len(checks)}/{len(checks)})')
