from pathlib import Path
import ast, binascii, importlib.util, random, re

ROOT = Path(__file__).resolve().parents[1]
def read(rel): return (ROOT / rel).read_text(encoding='utf-8')
def load(rel):
    p = ROOT / rel
    spec = importlib.util.spec_from_file_location('p49mon', p)
    m = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(m)
    return m

version = read('App/Common/app_version.h')
config = read('App/Common/app_config.h')
eskf = read('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
eskf_h = read('App/Modules/Estimation/FullStateESKF/full_state_eskf.h')
sd = read('App/Services/SDLogger/sd_logger.c')
policy = read('App/Modules/Control/VerticalSensorPolicy/vertical_sensor_policy.h')
system_monitor = read('App/Services/SystemMonitor/system_monitor.c')
preflight = read('App/Services/PreflightTrigger/preflight_trigger.c')
uart = read('App/Services/UARTTelemetry/uart_telemetry.c')
project = read('.project')
cproject = read('.cproject')
launch = read('TGY_V8_19M_P49_ZUPT_JOSEPH_VERTICAL_RECOVERY_SD_FIFO.launch')

assert '8.19M-P49-ZUPT-JOSEPH-VERT-REC-SD-FIFO' in version
assert '<name>TGY_V8_19M_P49_ZUPT_JOSEPH_VERTICAL_RECOVERY_SD_FIFO</name>' in project
assert 'TGY_V8_19M_P48_GRAV_SPARSE_SD_STALL_BUFFER' not in cproject
assert 'Debug/TGY_V8_19M_P49_ZUPT_JOSEPH_VERTICAL_RECOVERY_SD_FIFO.elf' in launch
assert not (ROOT / 'Debug').exists()

# P45/P46 guards remain.
for tok in (
    'FullESKF_CheckPublicVerticalGuard(public_now_us)',
    'FullESKF_RecoverCovarianceOnly(covariance_reason)',
    'covariance_state_preserving_recovery_count++',
    'APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL'):
    assert tok in eskf, tok

# P48 gravity fix stays enabled.
assert 'APP_FULL_ESKF_GRAVITY_ATTITUDE_ONLY_SPARSE_JOSEPH 1U' in config
assert 'FullESKF_GravityAttitudeOnlySparseJoseph(' in eskf

# P49 ZUPT: exact masked Joseph, BG gain is zero, direct BG stationary path remains.
assert 'APP_FULL_ESKF_ZUPT_MASKED_JOSEPH_ENABLED       1U' in config
zs = eskf.index('static uint8_t FullESKF_ZUPTMaskedJosephUnitState(')
ze = eskf.index('static uint8_t FullESKF_ScalarUpdateUnitState(', zs)
z = eskf[zs:ze]
for tok in (
    'double prior_column[FULL_ESKF_STATE_COUNT]',
    'double gain[FULL_ESKF_STATE_COUNT]',
    'if (row < FULL_ESKF_BG_X)',
    'gain[row] = 0.0;',
    '(gain[row] * prior_column[column])',
    '(prior_column[row] * gain[column])',
    '(gain[row] * innovation_variance * gain[column])',
    'FullESKF_ClampCovarianceDiagonal(FULL_ESKF_COV_STAGE_ZUPT)',
    'eskf_data.zupt_joseph_update_count++',
    'eskf_data.zupt_joseph_fault_count++'):
    assert tok in z, tok
unit = eskf[ze:eskf.index('static uint8_t FullESKF_ConstrainedSingleStateUpdate(', ze)]
assert unit.index('covariance_stage == FULL_ESKF_COV_STAGE_ZUPT') < unit.index('innovation_variance =')
assert 'FullESKF_ZUPTMaskedJosephUnitState(' in unit
for tok in ('full_eskf_zupt_joseph_update_count', 'full_eskf_zupt_joseph_fault_count'):
    assert tok in eskf, tok
assert 'FullESKF_ConstrainedSingleStateUpdate(' in eskf
assert 'FULL_ESKF_COV_STAGE_GYRO_BIAS' in eskf

# Pure numerical regression: simplified masked Joseph equals direct Joseph.
def mm(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(len(B)))
             for j in range(len(B[0]))] for i in range(len(A))]
def tr(A): return [list(x) for x in zip(*A)]
random.seed(49)
for _ in range(12):
    n=15
    M=[[random.uniform(-1,1) for _ in range(n)] for __ in range(n)]
    P=[[sum(M[i][k]*M[j][k] for k in range(n)) +
        (0.02 if i==j else 0.0) for j in range(n)] for i in range(n)]
    kstate=random.choice((3,4,5))
    R=0.04**2
    S=P[kstate][kstate]+R
    K=[(P[i][kstate]/S if i < 12 else 0.0) for i in range(n)]

    simp=[[0.0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            simp[i][j]=(P[i][j] - K[i]*P[kstate][j] -
                        P[i][kstate]*K[j] + K[i]*S*K[j])

    H=[0.0]*n; H[kstate]=1.0
    A=[[float(i==j)-K[i]*H[j] for j in range(n)] for i in range(n)]
    AP=mm(A,P); direct=mm(AP,tr(A))
    for i in range(n):
        for j in range(n):
            direct[i][j]+=K[i]*R*K[j]
    err=max(abs(simp[i][j]-direct[i][j]) for i in range(n) for j in range(n))
    assert err < 1e-9, err
    # BG diagonal is untouched by ZUPT by construction.
    for i in (12,13,14):
        assert abs(simp[i][i]-P[i][i]) < 1e-12
    assert all(simp[i][i] > 0.0 for i in range(n))

# P49 vertical recovery: historical numerical_error_count is diagnostic only.
assert '(eskf->numerical_error_count == 0UL)' not in policy
for tok in (
    '(eskf->covariance_integrity_ok != 0U)',
    '(eskf->output_inhibited == 0U)',
    '(eskf->healthy != 0U)',
    '(eskf->vertical_position_valid != 0U)'):
    assert tok in policy, tok

# Flight fault policy from P48 stays: one vertical sensor may carry flight.
for tok in ('VerticalSensorPolicy_Evaluate(eskf)', 'if (vertical.usable == 0U)',
            'latched_fault == SYS_FAULT_SD_LOGGING'):
    assert tok in system_monitor, tok

# P49 SD: 4 writer buffers remain, but READY selection is chronological ticket FIFO.
for tok in (
    '#define APP_SDLOGGER_DMA_BUFFER_COUNT           4U',
    '#define APP_SDLOGGER_RING_FRAME_COUNT           128U'):
    assert tok in config, tok
for tok in (
    'static uint32_t sd_buffer_ready_order[SDLOGGER_BUFFER_COUNT]',
    'static uint32_t sd_next_ready_order = 1UL',
    'SDLogger_MarkBufferReady(sd_active_buffer)',
    'uint32_t best_order = 0UL',
    '(order < best_order)',
    'ready_order = sd_buffer_ready_order[ready_buffer]',
    'sd_last_started_ready_order = ready_order',
    'sd_buffer_ready_order[completed_buffer] = 0UL',
    'sd_logger_fifo_order_fault_count'):
    assert tok in sd, tok
# Both full buffers and final partial buffer must receive a ticket.
assert sd.count('SDLogger_MarkBufferReady(sd_active_buffer);') >= 2

# Small regression reproducing why index order is unsafe and ticket order is FIFO.
# READY: buffer 0 contains newer ticket 4, buffer 2 contains older ticket 3.
states={0:('READY',4), 1:('WRITING',2), 2:('READY',3), 3:('FILLING',5)}
index_choice=min(i for i,(state,_) in states.items() if state=='READY')
ticket_choice=min((order,i) for i,(state,order) in states.items() if state=='READY')[1]
assert index_choice == 0 and ticket_choice == 2

# Wire/binary format intentionally unchanged except UART version.
assert '#define SDLOGGER_FORMAT_VERSION           14U' in sd
assert '#define SDLOGGER_FRAME_SIZE               384U' in sd
assert '(128UL * 1024UL * 1024UL)' in config
assert '(96UL * 1024UL * 1024UL)' in config
assert 'UARTTelemetry_AppendText(&position, "$TGY60")' in uart
assert 'FLIGHT DIAGNOSTICS V60' in uart
bs=uart.index('static uint8_t UARTTelemetry_BuildDataLine')
be=uart.index('void UARTTelemetry_Update10Hz',bs)
build=uart[bs:be]
emit=build.index('UARTTelemetry_AppendText(&position, "$TGY60")')
emissions=re.findall(r'\bFIELD_(?:U32|I32)\s*\(', build[emit:])
assert len(emissions)+1 == 258, len(emissions)+1
hs=uart.index('static const char header[] =',be); he=uart.index(';',hs)
lits=re.findall(r'"(?:\\.|[^"\\])*"', uart[hs:he])
header=''.join(ast.literal_eval(x) for x in lits)
hf=header.strip().lstrip('# ').split(',')
assert len(hf)==259 and hf[-1]=='crc16_ccitt' and len(set(hf))==len(hf)
mon=load('tools/monitor_uart_p49.py')
assert len(mon.FIELDS)==258 and tuple(hf[:-1])==mon.FIELDS
values=['$TGY60']+[str(i) for i in range(1,len(mon.FIELDS))]
body=','.join(values); crc=binascii.crc_hqx(body.encode('ascii'),0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}'))==258

# V14 decoder is unchanged.
assert read('tools/decode_flight_v14_p49.py') == read('tools/decode_flight_v14_p48.py')

print('P49 validation: PASS (masked Joseph ZUPT + live vertical recovery + chronological 4-buffer SD FIFO + P48 guards + TGY60/258 + V14 unchanged)')
