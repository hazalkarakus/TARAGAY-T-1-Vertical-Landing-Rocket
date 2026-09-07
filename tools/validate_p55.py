from pathlib import Path
import ast, binascii, importlib.util, re

ROOT = Path(__file__).resolve().parents[1]
def read(rel): return (ROOT/rel).read_text(encoding='utf-8')
def load(rel):
    p = ROOT/rel
    spec = importlib.util.spec_from_file_location('p55mon', p)
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
launch = read('TGY_V8_19M_P55_IMU_FAST_CONFIG_REPAIR.launch')

assert '8.19M-P55-IMU-FAST-CONFIG-REPAIR' in version
assert '<name>TGY_V8_19M_P55_IMU_FAST_CONFIG_REPAIR</name>' in project
assert '${workspace_loc:/TGY_V8_19M_P55_IMU_FAST_CONFIG_REPAIR}/Debug' in cproject
assert 'Debug/TGY_V8_19M_P55_IMU_FAST_CONFIG_REPAIR.elf' in launch
assert not (ROOT/'Debug').exists()

# P54 barometer protection retained unchanged.
for tok in ('APP_BARO_RAW_BASE_JUMP_PA                 4.0f',
            'APP_BARO_RAW_MAX_RATE_PA_PER_S            180.0f',
            'APP_BARO_RAW_HARD_JUMP_PA                 180.0f',
            'Barometer_RawPressurePlausible('):
    assert tok in (config+baro), tok

# SD wire contract remains fixed.
assert '#define SDLOGGER_FORMAT_VERSION           14U' in sd
assert '#define SDLOGGER_FRAME_SIZE               384U' in sd

# P55 fast config repair flow.
for tok in ('IMU_ServicePatternFastConfigRepair(',
            'imu_fast_config_check_count',
            'imu_fast_config_repair_attempt_count',
            'imu_fast_config_repair_success_count',
            'imu_fast_config_repair_failure_count',
            'imu_fast_config_repair_last_duration_us',
            'imu_fast_config_repair_max_duration_us',
            'IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_BDU_IF_INC)',
            'IMU_WriteReg(IMU_REG_CTRL1_XL, IMU_CTRL1_XL_1666HZ_8G)',
            'IMU_WriteReg(IMU_REG_CTRL2_G, IMU_CTRL2_G_1666HZ_1000DPS)',
            'imu_drv_fullscale_config_ok != 0U',
            'IMU_FAST_CONFIG_REPAIR_ESCALATE'):
    assert tok in (imu+imu_h), tok

hs = imu.index('static uint8_t IMU_ServicePatternFastConfigRepair(void)\n{')
he = imu.index('static uint8_t IMU_IsStaleSample(', hs)
helper = imu[hs:he]
assert 'HAL_Delay(' not in helper
assert helper.index('whoami = IMU_ReadReg(IMU_REG_WHO_AM_I)') < helper.index('IMU_WriteReg(IMU_REG_CTRL3_C')
assert 'imu_pattern_diagnostic.whoami = whoami' in helper
assert 'if (whoami_valid == 0U)' in helper
assert 'if (config_valid != 0U)' in helper

# First pattern still soft-resyncs and arms a deferred retry; fast repair runs
# at the start of that next acquisition before the 3-burst retry.
as_ = imu.index('static uint8_t IMU_AcquireValidatedSample(void)')
ae = imu.index('/* -------------------------------------------------------------------------- */\n/* Register access', as_)
acq = imu[as_:ae]
assert acq.index('IMU_ServicePatternFastConfigRepair()') < acq.index('for (uint8_t i = 0U; i < IMU_REDUNDANT_BURST_COUNT; i++)')
assert 'IMU_SPI_ResynchronizeBus();' in acq
assert 'imu_pattern_retry_pending = 1U;' in acq
assert 'IMU_StartRecovery(now_us);' in acq

# Existing full reset safety net is retained.
assert '#define IMU_RECOVERY_SETTLE_US                 40000UL' in imu
assert 'case IMU_RECOVERY_STEP_RESET_WRITE:' in imu
assert 'IMU_WriteReg(IMU_REG_CTRL3_C, IMU_CTRL3_C_SW_RESET);' in imu

# UART V66 / 303-field contract.
assert 'UARTTelemetry_AppendText(&position, "$TGY66")' in uart
assert 'FLIGHT DIAGNOSTICS V66' in uart
for tok in ('imu_fast_cfg_checks','imu_fast_cfg_attempts','imu_fast_cfg_success',
            'imu_fast_cfg_failures','imu_fast_cfg_last_us','imu_fast_cfg_max_us',
            'imu_pat_reg_valid','imu_pat_whoami','imu_pat_ctrl3_c'):
    assert tok in uart, tok
bs = uart.index('static uint8_t UARTTelemetry_BuildDataLine')
be = uart.index('void UARTTelemetry_Update10Hz', bs)
build = uart[bs:be]
emit = build.index('UARTTelemetry_AppendText(&position, "$TGY66")')
emissions = re.findall(r'\bFIELD_(?:U32|I32)\s*\(', build[emit:])
assert len(emissions)+1 == 303
hs2 = uart.index('static const char header[] =', be)
he2 = uart.index(';', hs2)
lits = re.findall(r'"(?:\\.|[^"\\])*"', uart[hs2:he2])
header = ''.join(ast.literal_eval(x) for x in lits)
hf = header.strip().lstrip('# ').split(',')
assert len(hf) == 304 and hf[-1] == 'crc16_ccitt'
mon = load('tools/monitor_uart_p55.py')
assert len(mon.FIELDS) == 303 and tuple(hf[:-1]) == mon.FIELDS
values = ['$TGY66'] + [str(i) for i in range(1, len(mon.FIELDS))]
body = ','.join(values)
crc = binascii.crc_hqx(body.encode('ascii'), 0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}')) == 303

print('P55 validation: PASS (P54 baro/SD protections retained + repeated-pattern soft resync + verified fast IMU config repair + full recovery fallback; TGY66/303)')
