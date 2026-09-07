from pathlib import Path
import ast, binascii, importlib.util, math, random, re

ROOT = Path(__file__).resolve().parents[1]
def read(rel): return (ROOT / rel).read_text(encoding='utf-8')
def load(rel):
    p = ROOT / rel
    spec = importlib.util.spec_from_file_location('p48mon', p)
    m = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(m)
    return m

version = read('App/Common/app_version.h')
config = read('App/Common/app_config.h')
eskf = read('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
eskf_h = read('App/Modules/Estimation/FullStateESKF/full_state_eskf.h')
sd = read('App/Services/SDLogger/sd_logger.c')
system_monitor = read('App/Services/SystemMonitor/system_monitor.c')
preflight = read('App/Services/PreflightTrigger/preflight_trigger.c')
preflight_h = read('App/Services/PreflightTrigger/preflight_trigger.h')
uart = read('App/Services/UARTTelemetry/uart_telemetry.c')
project = read('.project')
cproject = read('.cproject')
launch = read('TGY_V8_19M_P48_GRAV_SPARSE_SD_STALL_BUFFER.launch')

assert '8.19M-P48-GRAV-SPARSE-SD-STALL-BUFFER' in version
assert '<name>TGY_V8_19M_P48_GRAV_SPARSE_SD_STALL_BUFFER</name>' in project
assert 'TGY_V8_19M_P47_GRAV_JOSEPH_SD_BACKPRESSURE' not in cproject
assert 'Debug/TGY_V8_19M_P48_GRAV_SPARSE_SD_STALL_BUFFER.elf' in launch
assert not (ROOT / 'Debug').exists()

# P45/P46 safety baseline must remain frozen.
for tok in (
    'FullESKF_CheckPublicVerticalGuard(public_now_us)',
    'FullESKF_RecoverCovarianceOnly(covariance_reason)',
    'covariance_state_preserving_recovery_count++',
    'APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL'):
    assert tok in eskf, tok

# Flight logger wire format stays V14/384/200 Hz; long-soak allocation stays 128/96 MiB.
for tok in ('(128UL * 1024UL * 1024UL)', '(96UL * 1024UL * 1024UL)',
            '#define APP_SDLOGGER_RING_FRAME_COUNT           128U'):
    assert tok in config, tok
assert '#define SDLOGGER_FORMAT_VERSION           14U' in sd
assert '#define SDLOGGER_FRAME_SIZE               384U' in sd
assert 'APP_SDLOGGER_SAMPLE_PERIOD_US' in config

# P48 gravity: gain/injection restricted to theta states, sparse exact Joseph covariance.
assert 'APP_FULL_ESKF_GRAVITY_ATTITUDE_ONLY_SPARSE_JOSEPH 1U' in config
helper_start = eskf.index('static uint8_t FullESKF_GravityAttitudeOnlySparseJoseph(')
helper_end = eskf.index('static uint8_t FullESKF_ScalarUpdateTwoStates(', helper_start)
helper = eskf[helper_start:helper_end]
for tok in (
    'first_state_index < FULL_ESKF_THETA_X',
    'second_state_index > FULL_ESKF_THETA_Z',
    'error_state[first_state_index] = gain_first * innovation;',
    'error_state[second_state_index] = gain_second * innovation;',
    'double p11',
    'double a11',
    'gain_first * covariance_times_h[index]',
    'FullESKF_ClampCovarianceDiagonal(FULL_ESKF_COV_STAGE_GRAVITY)',
    'eskf_data.gravity_joseph_update_count++'):
    assert tok in helper, tok
# No gravity code may directly inject BG/BA/P/V means through a full gain.
assert 'error_state[index]' not in helper
assert 'nominal_gyro_bias' not in helper
assert 'nominal_accel_bias' not in helper
scalar = eskf[eskf.index('static uint8_t FullESKF_ScalarUpdateTwoStates('):
              eskf.index('static uint8_t FullESKF_ConstrainedSingleStateUpdate(')]
assert scalar.index('covariance_stage == FULL_ESKF_COV_STAGE_GRAVITY') < scalar.index('for (row = 0UL; row < FULL_ESKF_STATE_COUNT; row++)')
assert 'FullESKF_GravityAttitudeOnlySparseJoseph(' in scalar
assert 'FullESKF_JosephTwoStateCovariance(' not in eskf

# Independent numeric regression: sparse implementation equals the full Joseph equation
# with K constrained to the two observed attitude states, and remains PSD.
def mm(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(len(B))) for j in range(len(B[0]))] for i in range(len(A))]
def tr(A): return [list(x) for x in zip(*A)]
def sparse(P, a, b, h1, h2, R):
    n=len(P); c=[P[i][a]*h1 + P[i][b]*h2 for i in range(n)]
    S=R+h1*c[a]+h2*c[b]; k1=c[a]/S; k2=c[b]/S
    O=[row[:] for row in P]
    for j in range(n):
        if j not in (a,b):
            O[a][j]=O[j][a]=P[a][j]-k1*c[j]
            O[b][j]=O[j][b]=P[b][j]-k2*c[j]
    A2=[[1-k1*h1,-k1*h2],[-k2*h1,1-k2*h2]]
    P2=[[P[a][a],P[a][b]],[P[a][b],P[b][b]]]
    AP=mm(A2,P2); J=mm(AP,tr(A2))
    J[0][0]+=k1*R*k1; J[0][1]+=k1*R*k2; J[1][0]=J[0][1]; J[1][1]+=k2*R*k2
    O[a][a],O[a][b],O[b][a],O[b][b]=J[0][0],J[0][1],J[1][0],J[1][1]
    return O

def full(P,a,b,h1,h2,R):
    n=len(P); H=[0.0]*n; H[a]=h1; H[b]=h2
    c=[sum(P[i][j]*H[j] for j in range(n)) for i in range(n)]
    S=R+sum(H[i]*c[i] for i in range(n))
    K=[0.0]*n; K[a]=c[a]/S; K[b]=c[b]/S
    A=[[float(i==j)-K[i]*H[j] for j in range(n)] for i in range(n)]
    AP=mm(A,P); O=mm(AP,tr(A))
    for i in range(n):
        for j in range(n): O[i][j]+=K[i]*R*K[j]
    return O

random.seed(48)
for _ in range(12):
    n=15
    M=[[random.uniform(-1,1) for _ in range(n)] for __ in range(n)]
    P=[[sum(M[i][k]*M[j][k] for k in range(n)) + (0.01 if i==j else 0.0) for j in range(n)] for i in range(n)]
    h1=random.uniform(-1,1); h2=random.uniform(-1,1); R=0.035**2
    A=sparse(P,6,7,h1,h2,R); B=full(P,6,7,h1,h2,R)
    err=max(abs(A[i][j]-B[i][j]) for i in range(n) for j in range(n))
    assert err < 1e-9, err
    assert all(A[i][i] > 0.0 for i in range(n))

# P48 SD stall buffering: fixed CCM ring + four DMA writer buffers + earlier backpressure.
for tok in (
    '#define APP_SDLOGGER_DMA_BUFFER_COUNT           4U',
    'APP_SDLOGGER_RING_BACKPRESSURE_CLEAR_FRAMES     16U',
    'APP_SDLOGGER_RING_BACKPRESSURE_START_FRAMES     32U',
    'APP_SDLOGGER_RING_BACKPRESSURE_CRITICAL_FRAMES  72U',
    'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_HIGH          8U',
    'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_CRITICAL     12U',
    'APP_SDLOGGER_GUARD_RESUME_MAX_RING_FRAMES        8U'):
    assert tok in config, tok
assert '#define SDLOGGER_BUFFER_COUNT             APP_SDLOGGER_DMA_BUFFER_COUNT' in sd
assert 'buffer_index < SDLOGGER_BUFFER_COUNT' in sd
assert 'sd_buffers[SDLOGGER_BUFFER_COUNT][APP_SDLOGGER_BUFFER_SIZE]' in sd
# Memory design bounds: 49,152-byte CCM ring and 36,864-byte normal-SRAM writer pool.
assert 128*384 == 49152 and 49152 < 0xF000
assert 4*9216 == 36864 and 36864 < 48*1024
# DATA still has strict priority over guard in main update.
update = sd[sd.index('void SDLogger_Update(void)'):sd.index('void SDLogger_Stop(void)')]
assert update.index('SDLogger_StartReadyBufferDMA()') < update.index('SDLogger_StartGuardDMA()')
assert 'APP_SDLOGGER_GUARD_RESUME_MAX_RING_FRAMES' in update


# P48 flight-interlock reconciliation: dual vertical fallback must remain usable
# after separation, SD is required before separation but never kills active control.
for tok in (
    'VerticalSensorPolicy_Evaluate(eskf)',
    'flight_active = PreflightTrigger_IsFlightActive()',
    'if (vertical.usable == 0U)',
    'recorder health blocks arming/preflight, never active stabilization',
    'latched_fault == SYS_FAULT_SD_LOGGING'):
    assert tok in system_monitor, tok
assert 'uint8_t sd_ready;' in preflight_h
assert 'SDLogger_IsReady() != 0U' in preflight
assert 'SDLogger_IsLogging() != 0U' in preflight
assert '(preflight_trigger_status.sd_ready != 0U)' in preflight
# Preflight stays strict on both vertical references; in-flight monitor uses V50 policy.
assert '(preflight_trigger_status.lidar_reference_ready != 0U)' in preflight
assert '(preflight_trigger_status.barometer_reference_ready != 0U)' in preflight

# TGY59 uses the same 253 data fields, so diagnostics remain directly comparable with P47.
assert 'UARTTelemetry_AppendText(&position, "$TGY59")' in uart
assert 'FLIGHT DIAGNOSTICS V59' in uart
bs=uart.index('static uint8_t UARTTelemetry_BuildDataLine'); be=uart.index('void UARTTelemetry_Update10Hz',bs)
build=uart[bs:be]; emit=build.index('UARTTelemetry_AppendText(&position, "$TGY59")')
emissions=re.findall(r'\bFIELD_(?:U32|I32)\s*\(', build[emit:])
assert len(emissions)+1 == 253, len(emissions)+1
hs=uart.index('static const char header[] =',be); he=uart.index(';',hs)
lits=re.findall(r'"(?:\\.|[^"\\])*"', uart[hs:he])
header=''.join(ast.literal_eval(x) for x in lits)
hf=header.strip().lstrip('# ').split(',')
assert len(hf)==254 and hf[-1]=='crc16_ccitt' and len(set(hf))==len(hf)
mon=load('monitor_uart_p48.py')
assert len(mon.FIELDS)==253 and tuple(hf[:-1])==mon.FIELDS
values=['$TGY59']+[str(i) for i in range(1,len(mon.FIELDS))]
body=','.join(values); crc=binascii.crc_hqx(body.encode('ascii'),0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}'))==253

# V14 decoder intentionally remains byte-for-byte identical.
assert read('decode_flight_v14_p48.py') == read('decode_flight_v14_p47.py')

print('P48 validation: PASS (attitude-only sparse Joseph + P46 rollback + 4x SD writer buffering + early backpressure + TGY59/253 + V14 unchanged)')
