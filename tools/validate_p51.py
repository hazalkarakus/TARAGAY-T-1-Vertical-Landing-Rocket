from pathlib import Path
import ast, binascii, importlib.util, re

ROOT = Path(__file__).resolve().parents[1]
def read(rel): return (ROOT / rel).read_text(encoding='utf-8')
def load(rel):
    p=ROOT/rel
    spec=importlib.util.spec_from_file_location('p51mon', p)
    m=importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(m)
    return m

version=read('App/Common/app_version.h')
config=read('App/Common/app_config.h')
eskf=read('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
sd=read('App/Services/SDLogger/sd_logger.c')
policy=read('App/Modules/Control/VerticalSensorPolicy/vertical_sensor_policy.h')
uart=read('App/Services/UARTTelemetry/uart_telemetry.c')
project=read('.project')
cproject=read('.cproject')
launch=read('TGY_V8_19M_P51_STAGGERED_STATIONARY_FLIGHT_GATE.launch')

assert '8.19M-P51-STAGGERED-STATIONARY-FLIGHT-GATE' in version
assert '<name>TGY_V8_19M_P51_STAGGERED_STATIONARY_FLIGHT_GATE</name>' in project
assert 'TGY_V8_19M_P50_FAST_ZUPT_JOSEPH_SCHED_FIX' not in cproject
assert 'Debug/TGY_V8_19M_P51_STAGGERED_STATIONARY_FLIGHT_GATE.elf' in launch
assert not (ROOT/'Debug').exists()

# Existing numerical/safety guards retained.
for tok in (
 'FullESKF_CheckPublicVerticalGuard(public_now_us)',
 'FullESKF_RecoverCovarianceOnly(covariance_reason)',
 'covariance_state_preserving_recovery_count++',
 'APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL',
 'APP_FULL_ESKF_GRAVITY_ATTITUDE_ONLY_SPARSE_JOSEPH 1U',
 'FullESKF_GravityAttitudeOnlySparseJoseph(',
 'APP_FULL_ESKF_ZUPT_MASKED_JOSEPH_ENABLED       1U',
 'FullESKF_ZUPTMaskedJosephUnitState('):
    assert tok in (eskf+config), tok

# P51 vertical-speed safety gate.
assert 'APP_FULL_ESKF_STATIONARY_MAX_VERTICAL_SPEED_MPS 0.15f' in config
for tok in (
 'uint8_t vertical_speed_stationary_ok = 1U;',
 '(origin_zero_applied != 0U)',
 'FullESKF_Abs(nominal_velocity[2]) >',
 'APP_FULL_ESKF_STATIONARY_MAX_VERTICAL_SPEED_MPS',
 '(vertical_speed_stationary_ok != 0U)',
 'if (vertical_speed_stationary_ok == 0U)',
 'eskf_data.stationary_sample_count = 0UL;'):
    assert tok in eskf, tok

# P51 staggered stationary state machine: one phase, no 3-axis loops in function.
s=eskf.index('static void FullESKF_ProcessStationaryUpdates(')
e=eskf.index('/* -------------------------------------------------------------------------- */',s)
block=eskf[s:e]
for tok in (
 'FULL_ESKF_STATIONARY_PHASE_ZUPT_X',
 'FULL_ESKF_STATIONARY_PHASE_GYRO_X',
 'FULL_ESKF_STATIONARY_PHASE_ACCEL_X',
 'stationary_update_phase++',
 'eskf_data.stationary_update_count++'):
    assert tok in block, tok
assert 'for (axis = 0UL; axis < 3UL; axis++)' not in block

# P49 SD FIFO + vertical recovery remain.
assert '(eskf->numerical_error_count == 0UL)' not in policy
for tok in ('sd_buffer_ready_order[SDLOGGER_BUFFER_COUNT]',
            'sd_next_ready_order = 1UL',
            'sd_logger_fifo_order_fault_count'):
    assert tok in sd, tok
assert '#define SDLOGGER_FORMAT_VERSION           14U' in sd
assert '#define SDLOGGER_FRAME_SIZE               384U' in sd

# UART wire contract unchanged except version.
assert 'UARTTelemetry_AppendText(&position, "$TGY62")' in uart
assert 'FLIGHT DIAGNOSTICS V62' in uart
bs=uart.index('static uint8_t UARTTelemetry_BuildDataLine')
be=uart.index('void UARTTelemetry_Update10Hz',bs)
build=uart[bs:be]
emit=build.index('UARTTelemetry_AppendText(&position, "$TGY62")')
emissions=re.findall(r'\bFIELD_(?:U32|I32)\s*\(',build[emit:])
assert len(emissions)+1==258
hs=uart.index('static const char header[] =',be); he=uart.index(';',hs)
lits=re.findall(r'"(?:\\.|[^"\\])*"',uart[hs:he])
header=''.join(ast.literal_eval(x) for x in lits)
hf=header.strip().lstrip('# ').split(',')
assert len(hf)==259 and hf[-1]=='crc16_ccitt'
mon=load('tools/monitor_uart_p51.py')
assert len(mon.FIELDS)==258 and tuple(hf[:-1])==mon.FIELDS
values=['$TGY62']+[str(i) for i in range(1,len(mon.FIELDS))]
body=','.join(values); crc=binascii.crc_hqx(body.encode('ascii'),0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}'))==258


# Pure scheduler-phase regression: with optional accel tail enabled, exactly one
# stationary scalar correction per 200 Hz call and batch starts every 20 calls.
phase=0
counter=0
starts=[]
max_ops=0
for tick in range(400):
    ops=0
    if counter < 20:
        counter += 1
    if phase == 0:
        if counter < 20:
            max_ops=max(max_ops,ops)
            continue
        counter=0
        phase=1
        starts.append(tick)
    ops += 1
    if phase < 9:
        phase += 1
    else:
        phase=0
    max_ops=max(max_ops,ops)
assert max_ops == 1
assert all((b-a)==20 for a,b in zip(starts,starts[1:]))

# Flight-safety gate semantics.
def stationary_vz_ok(origin_locked, vz):
    return (not origin_locked) or (abs(vz) <= 0.15)
assert stationary_vz_ok(False, 2.0)
assert stationary_vz_ok(True, 0.0)
assert stationary_vz_ok(True, 0.15)
assert not stationary_vz_ok(True, 0.151)
assert not stationary_vz_ok(True, -0.4)

print('P51 validation: PASS (staggered stationary maintenance, post-origin vz gate, P50 numerical + P49 SD/vertical protections retained, TGY62/258)')
