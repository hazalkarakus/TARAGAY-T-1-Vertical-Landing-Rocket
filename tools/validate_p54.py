from pathlib import Path
import ast, binascii, importlib.util, re

ROOT=Path(__file__).resolve().parents[1]
def read(rel): return (ROOT/rel).read_text(encoding='utf-8')
def load(rel):
    p=ROOT/rel
    spec=importlib.util.spec_from_file_location('p54mon',p)
    m=importlib.util.module_from_spec(spec); assert spec.loader; spec.loader.exec_module(m); return m

version=read('App/Common/app_version.h')
config=read('App/Common/app_config.h')
baro_h=read('App/Modules/Sensors/Barometer/barometer.h')
baro=read('App/Modules/Sensors/Barometer/barometer.c')
imu_h=read('App/Modules/Sensors/IMU/imu.h')
imu=read('App/Modules/Sensors/IMU/imu.c')
tasks=read('App/Core/Tasks/app_tasks.c')
eskf=read('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
sd=read('App/Services/SDLogger/sd_logger.c')
uart=read('App/Services/UARTTelemetry/uart_telemetry.c')
project=read('.project'); cproject=read('.cproject')
launch=read('TGY_V8_19M_P54_BARO_RATE_GUARD_IMU_REG_DIAG.launch')

assert '8.19M-P54-BARO-RATE-GUARD-IMU-REG-DIAG' in version
assert '<name>TGY_V8_19M_P54_BARO_RATE_GUARD_IMU_REG_DIAG</name>' in project
assert 'TGY_V8_19M_P53_BARO_REF_TRACK_IMU_PATTERN_DIAG' not in cproject
assert 'Debug/TGY_V8_19M_P54_BARO_RATE_GUARD_IMU_REG_DIAG.elf' in launch
assert not (ROOT/'Debug').exists()

# P52/P51/P49 protections retained.
for tok in ('FullESKF_CheckPublicVerticalGuard(public_now_us)',
            'FullESKF_RecoverCovarianceOnly(covariance_reason)',
            'FullESKF_CovarianceTouchesUnobservedHorizontalPosition(',
            'APP_FULL_ESKF_GRAVITY_ATTITUDE_ONLY_SPARSE_JOSEPH 1U',
            'APP_FULL_ESKF_ZUPT_MASKED_JOSEPH_ENABLED       1U',
            'APP_FULL_ESKF_STATIONARY_MAX_VERTICAL_SPEED_MPS 0.15f'):
    assert tok in (eskf+config), tok
for tok in ('sd_buffer_ready_order[SDLOGGER_BUFFER_COUNT]',
            'sd_next_ready_order = 1UL','sd_logger_fifo_order_fault_count'):
    assert tok in sd, tok
assert '#define SDLOGGER_FORMAT_VERSION           14U' in sd
assert '#define SDLOGGER_FRAME_SIZE               384U' in sd

# P53 ground reference tracking/freeze retained.
for tok in ('APP_BARO_GROUND_TRACKING_ENABLED          1U',
            'APP_BARO_GROUND_TRACK_ALPHA               0.0025f',
            'APP_BARO_GROUND_TRACK_MAX_STEP_PA         0.050f',
            'Barometer_UpdateGroundReferenceTracking(',
            'Barometer_SetGroundReferenceTrackingAllowed(',
            'Barometer_FreezeGroundReference('):
    assert tok in (config+baro+baro_h), tok
for tok in ('preflight.debounced_open != 0U','preflight.flight_active != 0U',
            'eskf->stationary_detected != 0U','fabsf(eskf->velocity_z_mps)',
            'lidar->distance_valid != 0U'):
    assert tok in tasks, tok

# P54 time/rate-aware raw pressure guard.
for tok in ('APP_BARO_RAW_BASE_JUMP_PA                 4.0f',
            'APP_BARO_RAW_MAX_RATE_PA_PER_S            180.0f',
            'APP_BARO_RAW_HARD_JUMP_PA                 180.0f',
            'baro_last_plausible_timestamp_us',
            'Barometer_RawPressurePlausible(',
            'delta_pa > APP_BARO_RAW_HARD_JUMP_PA',
            'APP_BARO_RAW_MAX_RATE_PA_PER_S * ((float)dt_us * 0.000001f)'):
    assert tok in (config+baro), tok
assert 'APP_BARO_RAW_MAX_SINGLE_JUMP_PA' not in config

# Numeric guard regression using P53-observed classes: normal 1 Pa change must pass,
# 48 Pa two-frame transport glitch must stay rejected, ~1 kPa glitch can never pass.
BASE=4.0; RATE=180.0; HARD=180.0
last=101384.0; last_t=0

def plausible(p,t):
    global last,last_t
    delta=abs(p-last)
    if delta > HARD:
        return False
    allowed=min(HARD, BASE + RATE*((t-last_t)/1_000_000.0))
    if delta <= allowed:
        last=p; last_t=t; return True
    return False

assert plausible(101385.0,5_000)
assert not plausible(101433.0,10_000)   # +48 Pa class
assert not plausible(101433.0,15_000)   # repeated second bad sample
assert not plausible(102399.0,20_000)   # +~1 kPa class
# A physically gradual change remains passable.
for k in range(1,101):
    assert plausible(101385.0 + 0.25*k, 20_000 + 5_000*k)

# P54 IMU pattern-time register snapshot, no auto repair yet.
for tok in ('register_snapshot_valid','imu_pattern_register_snapshot_pending',
            'IMU_CapturePatternRegisterSnapshot(',
            'imu_pattern_diagnostic.whoami = IMU_ReadReg(IMU_REG_WHO_AM_I)',
            'imu_pattern_diagnostic.ctrl3_c = IMU_ReadReg(IMU_REG_CTRL3_C)',
            'if (imu_pattern_register_snapshot_pending != 0U)'):
    assert tok in (imu_h+imu), tok
# Snapshot is diagnostic-only: do not write config from the snapshot helper.
ss=imu.index('static void IMU_CapturePatternRegisterSnapshot(void)')
se=imu.index('static uint8_t IMU_IsStaleSample(',ss)
snap=imu[ss:se]
assert 'IMU_WriteReg' not in snap

# UART wire contract: same 297 fields, six redundant corrected-pattern fields
# replaced by six high-value register-snapshot fields.
assert 'UARTTelemetry_AppendText(&position, "$TGY65")' in uart
assert 'FLIGHT DIAGNOSTICS V65' in uart
for tok in ('imu_pat_reg_valid','imu_pat_whoami','imu_pat_ctrl1_xl',
            'imu_pat_ctrl2_g','imu_pat_ctrl3_c','imu_pat_ctrl4_c'):
    assert tok in uart, tok
assert 'imu_pat_corr_gx' not in uart
bs=uart.index('static uint8_t UARTTelemetry_BuildDataLine'); be=uart.index('void UARTTelemetry_Update10Hz',bs)
build=uart[bs:be]; emit=build.index('UARTTelemetry_AppendText(&position, "$TGY65")')
emissions=re.findall(r'\bFIELD_(?:U32|I32)\s*\(',build[emit:])
assert len(emissions)+1==297
hs=uart.index('static const char header[] =',be); he=uart.index(';',hs)
lits=re.findall(r'"(?:\\.|[^"\\])*"',uart[hs:he]); header=''.join(ast.literal_eval(x) for x in lits)
hf=header.strip().lstrip('# ').split(',')
assert len(hf)==298 and hf[-1]=='crc16_ccitt'
mon=load('tools/monitor_uart_p54.py')
assert len(mon.FIELDS)==297 and tuple(hf[:-1])==mon.FIELDS
values=['$TGY65']+[str(i) for i in range(1,len(mon.FIELDS))]
body=','.join(values); crc=binascii.crc_hqx(body.encode('ascii'),0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}'))==297

print('P54 validation: PASS (P53 datum tracking retained + rate-aware baro spike guard + pattern-time IMU register snapshot; P52 protections retained; TGY65/297)')
