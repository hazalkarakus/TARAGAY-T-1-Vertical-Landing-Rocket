from pathlib import Path
import ast, binascii, importlib.util, re
ROOT=Path(__file__).resolve().parents[1]
def read(rel): return (ROOT/rel).read_text(encoding='utf-8')
def load(rel):
    p=ROOT/rel; spec=importlib.util.spec_from_file_location('p47mon',p)
    m=importlib.util.module_from_spec(spec); assert spec.loader; spec.loader.exec_module(m); return m

version=read('App/Common/app_version.h')
config=read('App/Common/app_config.h')
eskf=read('App/Modules/Estimation/FullStateESKF/full_state_eskf.c')
eskf_h=read('App/Modules/Estimation/FullStateESKF/full_state_eskf.h')
sd=read('App/Services/SDLogger/sd_logger.c')
sd_h=read('App/Services/SDLogger/sd_logger.h')
uart=read('App/Services/UARTTelemetry/uart_telemetry.c')
project=read('.project'); cproject=read('.cproject')

assert '8.19M-P47-GRAV-JOSEPH-SD-BACKPRESSURE' in version
assert '<name>TGY_V8_19M_P47_GRAV_JOSEPH_SD_BACKPRESSURE</name>' in project
assert 'TGY_V8_19M_P46_COV_ROOTCAUSE_STATE_PRESERVE' not in cproject
assert not (ROOT/'Debug').exists()

# Frozen P46 safety/integrity baseline.
for tok in ('FullESKF_CheckPublicVerticalGuard(public_now_us)',
            'FullESKF_RecoverCovarianceOnly(covariance_reason)',
            'covariance_state_preserving_recovery_count++',
            'APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL'):
    assert tok in eskf, tok
assert '(128UL * 1024UL * 1024UL)' in config
assert '(96UL * 1024UL * 1024UL)' in config
assert '#define APP_SDLOGGER_RING_FRAME_COUNT           128U' in config
assert '#define SDLOGGER_FORMAT_VERSION           14U' in sd
assert '#define SDLOGGER_FRAME_SIZE               384U' in sd

# P47 Joseph gravity path.
for tok in ('gravity_joseph_update_count','gravity_joseph_fault_count'):
    assert tok in eskf_h and tok in eskf
helper=eskf[eskf.index('static uint8_t FullESKF_JosephTwoStateCovariance('):eskf.index('static uint8_t FullESKF_ScalarUpdateTwoStates(')]
for tok in ('covariance_temp[row][column] = covariance[row][column] -',
            'm_h_row * kalman_gain[column]',
            'kalman_gain[row] * measurement_variance',
            '0.5f * (from_row + from_column)',
            'FullESKF_ClampCovarianceDiagonal(covariance_stage)'):
    assert tok in helper, tok
frag=eskf[eskf.index('static uint8_t FullESKF_ScalarUpdateTwoStates('):eskf.index('static uint8_t FullESKF_ConstrainedSingleStateUpdate(')]
assert 'covariance_stage == FULL_ESKF_COV_STAGE_GRAVITY' in frag
assert 'FullESKF_JosephTwoStateCovariance(' in frag
assert frag.index('FullESKF_JosephTwoStateCovariance(') < frag.index('FullESKF_InjectError(error_state)')
assert 'eskf_data.gravity_joseph_fault_count++' in frag
assert 'eskf_data.gravity_joseph_update_count++' in frag
# cumulative diagnostics survive full safe reinit
safe=eskf[eskf.index('static void FullESKF_SafeReinitialize'):eskf.index('static uint8_t FullESKF_CheckPublicVerticalGuard')]
assert re.search(r'gravity_joseph_update_count\s*=\s*eskf_data\.gravity_joseph_update_count', safe)
assert re.search(r'eskf_data\.gravity_joseph_update_count\s*=\s*gravity_joseph_update_count', safe)

# P47 SD backpressure + data-first guard scheduling.
for tok in ('APP_SDLOGGER_RING_BACKPRESSURE_CLEAR_FRAMES     24U',
            'APP_SDLOGGER_RING_BACKPRESSURE_START_FRAMES     48U',
            'APP_SDLOGGER_RING_BACKPRESSURE_CRITICAL_FRAMES  96U',
            'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_HIGH          6U',
            'APP_SDLOGGER_RING_DRAIN_BUDGET_US_HIGH          480UL',
            'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_CRITICAL      8U',
            'APP_SDLOGGER_RING_DRAIN_BUDGET_US_CRITICAL      650UL',
            'APP_SDLOGGER_GUARD_RESUME_MAX_RING_FRAMES       16U'):
    assert tok in config, tok
for tok in ('sd_logger_backpressure_level','sd_logger_backpressure_entry_count',
            'sd_logger_backpressure_critical_entry_count','sd_logger_guard_pending',
            'sd_logger_guard_deferred_count','sd_logger_async_card_busy_poll_count',
            'sd_logger_max_write_duration_us','sd_logger_max_guard_write_duration_us'):
    assert tok in sd_h and tok in sd, tok
assert 'static uint8_t SDLogger_UpdateBackpressureLevel(uint32_t ring_count)' in sd
drain=sd[sd.index('static void SDLogger_DrainRingToBuffers(void)'):sd.index('static uint8_t SDLogger_FindReadyBuffer',sd.index('static void SDLogger_DrainRingToBuffers(void)'))]
assert 'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_CRITICAL' in drain
assert 'APP_SDLOGGER_RING_DRAIN_BUDGET_US_CRITICAL' in drain
assert 'APP_SDLOGGER_RING_DRAIN_MAX_FRAMES_HIGH' in drain

svc_start=sd.rindex('static void SDLogger_ServiceAsyncDMA(void)')
service=sd[svc_start:sd.index('static uint8_t SDLogger_HasPendingWriteWork',svc_start)]
data_start=service.index('if (sd_async_state == SDLOGGER_ASYNC_DATA_DMA)')
data_end=service.index('else if (sd_async_state == SDLOGGER_ASYNC_GUARD_DMA)', data_start)
data=service[data_start:data_end]
assert 'SDLogger_StartGuardDMA()' not in data
assert 'sd_async_state = SDLOGGER_ASYNC_IDLE;' in data
assert 'sd_logger_guard_pending = 1U;' in data
guard=service[data_end:]
assert 'sd_logger_guard_pending = 0U;' in guard
update=sd[sd.index('void SDLogger_Update(void)'):sd.index('void SDLogger_Stop(void)')]
assert update.index('SDLogger_StartReadyBufferDMA()') < update.index('SDLogger_StartGuardDMA()')
assert 'APP_SDLOGGER_GUARD_RESUME_MAX_RING_FRAMES' in update
pending_start=sd.index('static uint8_t SDLogger_HasPendingWriteWork',svc_start)
pending_end=sd.index('static uint8_t SDLogger_DrainAllBlocking',pending_start)
pending=sd[pending_start:pending_end]
assert 'sd_logger_guard_pending != 0U' in pending

# TGY58 contract.
assert 'UARTTelemetry_AppendText(&position, "$TGY58")' in uart
assert 'FLIGHT DIAGNOSTICS V58' in uart
for tok in ('grav_joseph_updates','grav_joseph_faults','sd_ring_count','sd_ring_high_water',
            'sd_backpressure_level','sd_guard_pending','sd_card_busy_polls','sd_write_max_us','sd_guard_max_us'):
    assert tok in uart, tok
bs=uart.index('static uint8_t UARTTelemetry_BuildDataLine'); be=uart.index('void UARTTelemetry_Update10Hz',bs)
build=uart[bs:be]; emit=build.index('UARTTelemetry_AppendText(&position, "$TGY58")')
emissions=re.findall(r'\bFIELD_(?:U32|I32)\s*\(',build[emit:])
assert len(emissions)+1==253, len(emissions)+1
hs=uart.index('static const char header[] =',be); he=uart.index(';',hs)
lits=re.findall(r'"(?:\\.|[^"\\])*"',uart[hs:he])
header=''.join(ast.literal_eval(x) for x in lits)
hf=header.strip().lstrip('# ').split(',')
assert len(hf)==254 and hf[-1]=='crc16_ccitt' and len(set(hf))==len(hf)
mon=load('monitor_uart_p47.py')
assert len(mon.FIELDS)==253 and tuple(hf[:-1])==mon.FIELDS
values=['$TGY58']+[str(i) for i in range(1,len(mon.FIELDS))]
body=','.join(values); crc=binascii.crc_hqx(body.encode('ascii'),0xFFFF)
assert len(mon.decode_frame(f'{body}*{crc:04X}'))==253

# V14 decoder remains unchanged.
assert read('decode_flight_v14_p47.py') == read('decode_flight_v14_p46.py')

print('P47 validation: PASS (GRAV Joseph + P46 rollback + SD adaptive backpressure/data-first guard + TGY58/253 + V14 unchanged)')
