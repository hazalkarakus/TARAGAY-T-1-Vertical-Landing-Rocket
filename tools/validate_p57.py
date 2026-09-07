from pathlib import Path
import ast, binascii, importlib.util, re

ROOT = Path(__file__).resolve().parents[1]
def read(rel): return (ROOT/rel).read_text(encoding='utf-8')
def load(rel):
    p = ROOT/rel
    spec = importlib.util.spec_from_file_location('p57mon', p)
    m = importlib.util.module_from_spec(spec); assert spec.loader
    spec.loader.exec_module(m)
    return m

version = read('App/Common/app_version.h')
config = read('App/Common/app_config.h')
baro = read('App/Modules/Sensors/Barometer/barometer.c')
imu_h = read('App/Modules/Sensors/IMU/imu.h')
imu = read('App/Modules/Sensors/IMU/imu.c')
sd = read('App/Services/SDLogger/sd_logger.c')
uart = read('App/Services/UARTTelemetry/uart_telemetry.c')
project = read('.project')
cproject = read('.cproject')
launch = read('TGY_V8_19M_P57_IMU_STALE_STATUS_WARMUP.launch')

assert '8.19M-P57-IMU-STALE-STATUS-WARMUP' in version
assert '#define APP_VERSION_PATCH                       51' in version
assert '<name>TGY_V8_19M_P57_IMU_STALE_STATUS_WARMUP</name>' in project
assert '${workspace_loc:/TGY_V8_19M_P57_IMU_STALE_STATUS_WARMUP}/Debug' in cproject
assert 'Debug/TGY_V8_19M_P57_IMU_STALE_STATUS_WARMUP.elf' in launch
assert not (ROOT/'Debug').exists()

# Barometer, SD wire format and existing safety architecture stay untouched.
for tok in ('APP_BARO_RAW_BASE_JUMP_PA                 4.0f',
            'APP_BARO_RAW_MAX_RATE_PA_PER_S            180.0f',
            'APP_BARO_RAW_HARD_JUMP_PA                 180.0f',
            'Barometer_RawPressurePlausible('):
    assert tok in (config+baro), tok
assert '#define SDLOGGER_FORMAT_VERSION           14U' in sd
assert '#define SDLOGGER_FRAME_SIZE               384U' in sd

# P55 repeated-pattern register repair remains intact.
for tok in ('IMU_ServicePatternFastConfigRepair(',
            'imu_fast_config_repair_success_count',
            'imu_pattern_recovery_escalation_count',
            'IMU_SPI_ResynchronizeBus();'):
    assert tok in imu, tok

# P56 stale register diagnosis/repair remains intact.
for tok in ('IMU_ServiceStaleFastConfigRepair(',
            'imu_stale_fast_config_check_count',
            'imu_stale_fast_config_repair_success_count',
            'imu_stale_diagnostic.whoami = whoami',
            'IMU_WriteReg(IMU_REG_CTRL1_XL, IMU_CTRL1_XL_1666HZ_8G)',
            'IMU_WriteReg(IMU_REG_CTRL2_G, IMU_CTRL2_G_1666HZ_1000DPS)'):
    assert tok in imu, tok

# P57: successful power-down config repair arms a bounded STATUS_REG warmup.
ss = imu.index('static uint8_t IMU_ServiceStaleFastConfigRepair(void)\n{')
se = imu.index('static uint8_t IMU_IsStaleSample(', ss)
stale_helper = imu[ss:se]
for tok in ('imu_stale_warmup_active = 1U;',
            'imu_stale_warmup_start_us = warmup_now_us;',
            'imu_stale_warmup_event_count++;',
            'return IMU_FAST_CONFIG_REPAIR_DEFER;'):
    assert tok in stale_helper, tok
assert 'HAL_Delay(' not in stale_helper

# Warmup gate: one STATUS_REG read per scheduler call, both XLDA and GDA
# required, bounded 40 ms timeout and full recovery fallback retained.
as_ = imu.index('static uint8_t IMU_AcquireValidatedSample(void)')
ae = imu.index('/* -------------------------------------------------------------------------- */\n/* Register access', as_)
acq = imu[as_:ae]
for tok in ('if (imu_stale_warmup_active != 0U)',
            'IMU_ReadReg(IMU_REG_STATUS_REG)',
            'IMU_STATUS_XLDA | IMU_STATUS_GDA',
            'IMU_STALE_WARMUP_TIMEOUT_US',
            'imu_stale_warmup_poll_count++',
            'imu_stale_warmup_first_ready_us = elapsed_us',
            'imu_stale_warmup_timeout_count++',
            'imu_stale_warmup_success_count++',
            'IMU_StartRecovery(now_us);'):
    assert tok in acq, tok
assert '#define IMU_STALE_WARMUP_TIMEOUT_US          40000UL' in imu
assert 'HAL_Delay(' not in acq

# Same stale episode may retry after DATA_READY while bounded; pattern corruption
# or timeout still escalates, so the safety net is not weakened.
assert '(imu_last_commit_pattern_error == 0U)' in acq
assert '(warmup_elapsed_us < IMU_STALE_WARMUP_TIMEOUT_US)' in acq
assert '#define IMU_RECOVERY_SETTLE_US                 40000UL' in imu
assert 'case IMU_RECOVERY_STEP_RESET_WRITE:' in imu
assert 'IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_SW_RESET);' in imu

# New diagnostics are exported through the public IMU API.
for tok in ('IMU_GetStaleWarmupEventCount', 'IMU_GetStaleWarmupPollCount',
            'IMU_GetStaleWarmupSuccessCount', 'IMU_GetStaleWarmupTimeoutCount',
            'IMU_GetStaleWarmupFirstReadyUs', 'IMU_GetStaleWarmupMaxReadyUs',
            'IMU_GetStaleWarmupLastStatus', 'IMU_GetStaleWarmupActive'):
    assert tok in imu_h and tok in imu, tok

# UART V68 / 329-field contract.
assert 'UARTTelemetry_AppendText(&position, "$TGY68")' in uart
assert 'FLIGHT DIAGNOSTICS V68' in uart
for tok in ('imu_stale_warmup_events','imu_stale_warmup_polls',
            'imu_stale_warmup_success','imu_stale_warmup_timeouts',
            'imu_stale_warmup_first_ready_us','imu_stale_warmup_max_ready_us',
            'imu_stale_warmup_last_status','imu_stale_warmup_active'):
    assert tok in uart, tok
bs = uart.index('static uint8_t UARTTelemetry_BuildDataLine')
be = uart.index('void UARTTelemetry_Update10Hz', bs)
build = uart[bs:be]
emit = build.index('UARTTelemetry_AppendText(&position, "$TGY68")')
emissions = re.findall(r'\bFIELD_(?:U32|I32)\s*\(', build[emit:])
assert len(emissions)+1 == 329, len(emissions)+1
hs2 = uart.index('static const char header[] =', be)
he2 = uart.index(';', hs2)
lits = re.findall(r'"(?:\\.|[^"\\])*"', uart[hs2:he2])
header = ''.join(ast.literal_eval(x) for x in lits)
hf = header.strip().lstrip('# ').split(',')
assert len(hf) == 330 and hf[-1] == 'crc16_ccitt', len(hf)
mon = load('tools/monitor_uart_p57.py')
assert len(mon.FIELDS) == 329 and tuple(hf[:-1]) == mon.FIELDS
values = ['$TGY68'] + [str(i) for i in range(1, len(mon.FIELDS))]
body = ','.join(values)
crc = binascii.crc_hqx(body.encode('ascii'), 0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}')) == 329

print('P57 validation: PASS (P55 pattern repair + P56 stale register repair retained; bounded STATUS_REG XLDA|GDA warmup added; full recovery fallback retained; SD V14 fixed; TGY68/329)')
