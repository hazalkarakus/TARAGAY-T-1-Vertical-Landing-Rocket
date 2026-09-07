from pathlib import Path
import ast, binascii, importlib.util, math, random, re, struct

ROOT = Path(__file__).resolve().parents[1]
def read(rel): return (ROOT / rel).read_text(encoding='utf-8')
def f32(x): return struct.unpack('f', struct.pack('f', float(x)))[0]
def load(rel):
    p = ROOT / rel
    spec = importlib.util.spec_from_file_location('p50mon', p)
    m = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(m)
    return m

version = read('App/Common/app_version.h')
config = read('App/Common/app_config.h')
eskf = read('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
sd = read('App/Services/SDLogger/sd_logger.c')
policy = read('App/Modules/Control/VerticalSensorPolicy/vertical_sensor_policy.h')
uart = read('App/Services/UARTTelemetry/uart_telemetry.c')
project = read('.project')
cproject = read('.cproject')
launch = read('TGY_V8_19M_P50_FAST_ZUPT_JOSEPH_SCHED_FIX.launch')

assert '8.19M-P50-FAST-ZUPT-JOSEPH-SCHED-FIX' in version
assert '<name>TGY_V8_19M_P50_FAST_ZUPT_JOSEPH_SCHED_FIX</name>' in project
assert 'TGY_V8_19M_P49R1_GENERATED_TYPES_BUILD_FIX' not in cproject
assert 'Debug/TGY_V8_19M_P50_FAST_ZUPT_JOSEPH_SCHED_FIX.elf' in launch
assert not (ROOT / 'Debug').exists()

# P45/P46 safety guards remain.
for tok in (
    'FullESKF_CheckPublicVerticalGuard(public_now_us)',
    'FullESKF_RecoverCovarianceOnly(covariance_reason)',
    'covariance_state_preserving_recovery_count++',
    'APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL'):
    assert tok in eskf, tok

# P48 gravity sparse Joseph remains enabled.
assert 'APP_FULL_ESKF_GRAVITY_ATTITUDE_ONLY_SPARSE_JOSEPH 1U' in config
assert 'FullESKF_GravityAttitudeOnlySparseJoseph(' in eskf

# P50 ZUPT: exact algebraic masked Joseph, hardware float only.
assert 'APP_FULL_ESKF_ZUPT_MASKED_JOSEPH_ENABLED       1U' in config
zs = eskf.index('static uint8_t FullESKF_ZUPTMaskedJosephUnitState(')
ze = eskf.index('static uint8_t FullESKF_ScalarUpdateUnitState(', zs)
z = eskf[zs:ze]
for tok in (
    'float prior_column[FULL_ESKF_STATE_COUNT]',
    'float scaled_column[FULL_ESKF_BG_X]',
    'inverse_innovation_variance = 1.0f / innovation_variance;',
    'for (row = 0UL; row < FULL_ESKF_BG_X; row++)',
    'float updated = covariance[row][column] -',
    '(scaled * prior_column[column])',
    'FullESKF_ClampCovarianceDiagonal(FULL_ESKF_COV_STAGE_ZUPT)',
    'eskf_data.zupt_joseph_update_count++',
    'eskf_data.zupt_joseph_fault_count++'):
    assert tok in z, tok
assert 'double prior_column' not in z
assert 'double gain' not in z

# Pure numerical regression: fast formula equals P49 masked Joseph algebra.
# Use realistic-ish SPD matrices and float32 at each P50 operation.
random.seed(50)
max_err = 0.0
for _ in range(50):
    n = 15
    M = [[random.uniform(-0.2,0.2) for _ in range(n)] for __ in range(n)]
    P = [[sum(M[i][k]*M[j][k] for k in range(n)) +
          (0.01 if i == j else 0.0) for j in range(n)] for i in range(n)]
    kstate = random.choice((3,4,5))
    R = 0.04**2
    S = P[kstate][kstate] + R
    prior = [P[i][kstate] for i in range(n)]
    K = [(prior[i]/S if i < 12 else 0.0) for i in range(n)]

    # Reference exact masked Joseph simplified algebra in double.
    ref = [row[:] for row in P]
    for i in range(n):
        for j in range(n):
            ref[i][j] = (P[i][j] - K[i]*prior[j] - prior[i]*K[j] + K[i]*S*K[j])

    # P50 implementation, emulating float32 intermediate storage.
    fast = [[f32(v) for v in row] for row in P]
    prior32 = [f32(fast[i][kstate]) for i in range(n)]
    invS32 = f32(1.0 / f32(fast[kstate][kstate] + f32(R)))
    scaled = [f32(prior32[i] * invS32) for i in range(12)]
    for i in range(12):
        for j in range(i,n):
            v = f32(fast[i][j] - f32(scaled[i]*prior32[j]))
            fast[i][j] = v
            fast[j][i] = v

    # BGxBG block must be bit-preserved from prior float matrix.
    for i in (12,13,14):
        for j in (12,13,14):
            assert fast[i][j] == f32(P[i][j])

    err = max(abs(fast[i][j]-ref[i][j]) for i in range(n) for j in range(n))
    max_err = max(max_err, err)
    assert err < 2e-6, err
    assert all(fast[i][i] > -1e-7 for i in range(n))

# P49 vertical recovery and SD FIFO fixes remain.
assert '(eskf->numerical_error_count == 0UL)' not in policy
for tok in ('(eskf->covariance_integrity_ok != 0U)', '(eskf->output_inhibited == 0U)', '(eskf->healthy != 0U)'):
    assert tok in policy, tok
for tok in ('sd_buffer_ready_order[SDLOGGER_BUFFER_COUNT]', 'sd_next_ready_order = 1UL', 'sd_logger_fifo_order_fault_count'):
    assert tok in sd, tok
assert '#define SDLOGGER_FORMAT_VERSION           14U' in sd
assert '#define SDLOGGER_FRAME_SIZE               384U' in sd

# UART wire contract: only frame version changes; field count stays 258.
assert 'UARTTelemetry_AppendText(&position, "$TGY61")' in uart
assert 'FLIGHT DIAGNOSTICS V61' in uart
bs=uart.index('static uint8_t UARTTelemetry_BuildDataLine')
be=uart.index('void UARTTelemetry_Update10Hz',bs)
build=uart[bs:be]
emit=build.index('UARTTelemetry_AppendText(&position, "$TGY61")')
emissions=re.findall(r'\bFIELD_(?:U32|I32)\s*\(', build[emit:])
assert len(emissions)+1 == 258, len(emissions)+1
hs=uart.index('static const char header[] =',be); he=uart.index(';',hs)
lits=re.findall(r'"(?:\\.|[^"\\])*"', uart[hs:he])
header=''.join(ast.literal_eval(x) for x in lits)
hf=header.strip().lstrip('# ').split(',')
assert len(hf)==259 and hf[-1]=='crc16_ccitt' and len(set(hf))==len(hf)
mon=load('tools/monitor_uart_p50.py')
assert len(mon.FIELDS)==258 and tuple(hf[:-1])==mon.FIELDS
values=['$TGY61']+[str(i) for i in range(1,len(mon.FIELDS))]
body=','.join(values); crc=binascii.crc_hqx(body.encode('ascii'),0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}'))==258

print(f'P50 validation: PASS (fast float masked Joseph equivalent, BG block preserved, max numeric diff={max_err:.3g}, P49 vertical/SD fixes retained, TGY61/258)')
