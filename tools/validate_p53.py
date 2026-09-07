from pathlib import Path
import ast, binascii, importlib.util, math, re

ROOT=Path(__file__).resolve().parents[1]
def read(rel): return (ROOT/rel).read_text(encoding='utf-8')
def load(rel):
    p=ROOT/rel
    spec=importlib.util.spec_from_file_location('p53mon',p)
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
launch=read('TGY_V8_19M_P53_BARO_REF_TRACK_IMU_PATTERN_DIAG.launch')

assert '8.19M-P53-BARO-REF-TRACK-IMU-PATTERN-DIAG' in version
assert '<name>TGY_V8_19M_P53_BARO_REF_TRACK_IMU_PATTERN_DIAG</name>' in project
assert 'TGY_V8_19M_P52_OBSERVABILITY_AWARE_COVARIANCE' not in cproject
assert 'Debug/TGY_V8_19M_P53_BARO_REF_TRACK_IMU_PATTERN_DIAG.elf' in launch
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

# P53 baro policy source gates.
for tok in ('APP_BARO_GROUND_TRACKING_ENABLED          1U',
            'APP_BARO_GROUND_TRACK_ALPHA               0.0025f',
            'APP_BARO_GROUND_TRACK_MAX_STEP_PA         0.050f',
            'Barometer_UpdateGroundReferenceTracking(',
            'Barometer_SetGroundReferenceTrackingAllowed(',
            'Barometer_FreezeGroundReference(',
            'ground_reference_frozen',
            'ground_reference_update_count'):
    assert tok in (config+baro+baro_h), tok
for tok in ('preflight.debounced_open != 0U',
            'preflight.flight_active != 0U',
            'eskf->stationary_detected != 0U',
            'fabsf(eskf->velocity_z_mps)',
            'lidar->distance_valid != 0U',
            'APP_BARO_GROUND_TRACK_LIDAR_MAX_AGE_US'):
    assert tok in tasks, tok
for tok in ('barometer->ground_reference_tracking_active != 0U',
            'eskf_data.baro_reference_m = barometer->filtered_altitude_m',
            'baro_reference_locked = 1U'):
    assert tok in eskf, tok
# one-way freeze must block re-enable
freeze=baro[baro.index('void Barometer_SetGroundReferenceTrackingAllowed'):baro.index('uint8_t Barometer_IsConnected')]
assert 'ground_reference_frozen != 0U' in freeze
assert 'ground_reference_tracking_allowed = 0U' in freeze

# Pure drift model regression: +20 Pa warmup drift must be followed when stationary,
# then datum must freeze and a -12 Pa pressure delta must show ~+1m altitude.
g=101380.0
alpha=0.0025; maxstep=0.05
for k in range(120000): # 10 min @200Hz
    p=101380.0 + 20.0*(k/119999.0)
    e=p-g; step=max(-maxstep,min(maxstep,alpha*e)); g+=step
stationary_alt=(p-g)/-12.0  # sign-independent magnitude check below
assert abs(stationary_alt) < 0.05, stationary_alt
frozen=g
p2=p-12.0
h=(frozen-p2)/12.0
assert 0.95 < h < 1.05, h

# IMU last-pattern diagnostic capture.
for tok in ('IMU_PatternDiagnostic_t','IMU_GetPatternDiagnostic(',
            'IMU_CapturePatternDiagnostic(',
            'imu_pattern_diagnostic.burst[i] = samples[i]',
            'imu_pattern_diagnostic.corrected = *corrected',
            'imu_pattern_diagnostic.timestamp_us = micros()'):
    assert tok in (imu_h+imu), tok
pattern_branch=imu[imu.index('if (IMU_HasRepeatedWordPattern(&corrected) != 0U)'):]
assert pattern_branch.index('IMU_CapturePatternDiagnostic(samples, &corrected);') < pattern_branch.index('imu_pattern_error_count++;')

# UART wire contract.
assert 'UARTTelemetry_AppendText(&position, "$TGY64")' in uart
assert 'FLIGHT DIAGNOSTICS V64' in uart
bs=uart.index('static uint8_t UARTTelemetry_BuildDataLine'); be=uart.index('void UARTTelemetry_Update10Hz',bs)
build=uart[bs:be]; emit=build.index('UARTTelemetry_AppendText(&position, "$TGY64")')
emissions=re.findall(r'\bFIELD_(?:U32|I32)\s*\(',build[emit:])
assert len(emissions)+1==297
hs=uart.index('static const char header[] =',be); he=uart.index(';',hs)
lits=re.findall(r'"(?:\\.|[^"\\])*"',uart[hs:he]); header=''.join(ast.literal_eval(x) for x in lits)
hf=header.strip().lstrip('# ').split(',')
assert len(hf)==298 and hf[-1]=='crc16_ccitt'
mon=load('tools/monitor_uart_p53.py')
assert len(mon.FIELDS)==297 and tuple(hf[:-1])==mon.FIELDS
values=['$TGY64']+[str(i) for i in range(1,len(mon.FIELDS))]
body=','.join(values); crc=binascii.crc_hqx(body.encode('ascii'),0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}'))==297

print('P53 validation: PASS (preflight baro datum tracking + flight freeze + IMU raw pattern capture; P52 protections retained; TGY64/297)')
