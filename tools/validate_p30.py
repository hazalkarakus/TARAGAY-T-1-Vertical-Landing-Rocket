#!/usr/bin/env python3
from pathlib import Path
import re,sys
R=Path(__file__).resolve().parents[1]
def t(p): return (R/p).read_text(errors='ignore')
def req(c,m):
    if not c: print('P30 validation: FAIL - '+m); sys.exit(1)
cfg=t('App/Common/app_config.h'); baro=t('App/Modules/Sensors/Barometer/ms5611_spi.c'); bh=t('App/Modules/Sensors/Barometer/barometer.c'); eskf=t('App/Modules/Estimation/FullStateESKF/full_state_eskf.c'); sol=t('App/Services/SolenoidOutput/solenoid_output.c'); nd=t('App/Services/NeedleValveIntegrationTest/needle_valve_integration_test.c'); ctrl=t('App/Modules/Control/NeedleValve/needle_valve_controller.c'); app=t('App/app.c'); tasks=t('App/Core/Tasks/app_tasks.c'); sch=t('App/Core/Scheduler/scheduler.c')
req('#define BMP585_SPI_TIMEOUT_MS        2U' in baro,'baro SPI timeout')
req('BMP585_RequestRecovery' in baro and 'BMP585_ServiceRecovery' in baro,'baro recovery')
req('HAL_SPI_Abort' in baro and 'HAL_SPI_DeInit' in baro and 'HAL_SPI_Init' in baro,'SPI2 recovery')
service=baro[baro.index('static void BMP585_ServiceRecovery'):baro.index('/* -------------------------------------------------------------------------- */\n/* V8.19E non-invasive',baro.index('static void BMP585_ServiceRecovery'))]
req('HAL_Delay' not in service,'runtime recovery blocking delay')
req('APP_BARO_STALE_TIMEOUT_US' in bh and 'pressure_valid = 0U' in bh,'stale baro invalidation')
req(re.search(r'APP_TASK_FULL_ESKF_CORRECTION_PERIOD_US\s+5000UL',cfg),'ESKF task not 5ms')
req('Task_FullESKFCorrection_200Hz' in sch,'ESKF scheduler entry')
pred=eskf[eskf.index('void FullStateESKF_Predict(void)'):eskf.index('void FullStateESKF_CorrectMeasurements(void)')]; cor=eskf[eskf.index('void FullStateESKF_CorrectMeasurements(void)'):eskf.index('uint8_t FullStateESKF_IsInitialized(void)')]
req('public_output_count++' not in pred and 'public_output_count++' in cor,'public output is not scheduler-driven 200Hz')
req('APP_FULL_ESKF_COVARIANCE_DECIMATION         40U' in cfg,'25Hz covariance')
req('APP_FULL_ESKF_HEALTH_CHECK_DECIMATION        4U' in cfg and 'APP_FULL_ESKF_LIVE_DEBUG_DECIMATION          4U' in cfg,'50Hz diagnostics decimation')
req('APP_RELAY_SEQUENCE_TEST_MODE' in cfg and 'RELAY_BENCH_STEP_MS 500UL' in sol,'relay profile')
for x in ('case 1U:','case 2U:','case 3U:','case 4U:'): req(x in sol,'relay channel sequence')
req('SolenoidOutput_BenchTestUpdate();' in app,'relay owner integration')
req('APP_NEEDLE_FOUR_TURN_TEST_MODE' in cfg and 'V819J_TARGET_TRAVEL_ADC 780U' in nd,'4-turn profile')
req('NeedleValveController_SetCommand(1.0f)' in nd and 'NeedleValveController_SetCommand(0.0f)' in nd,'4-turn open/close commands')
req('V819J_TEST_TRAVEL_MISMATCH' in nd and 'V819J_TEST_CLOSE_MISMATCH' in nd,'4-turn verification')
req('APP_NEEDLE_FOUR_TURN_TEST_MODE == 0U' in ctrl,'bounded needle interlock bypass')
req(re.search(r'APP_OPTIONAL_NRF24_ENABLED\s+1U',cfg) and re.search(r'APP_OPTIONAL_SDLOGGER_ENABLED\s+1U',cfg),'nRF/SD full-system profile not enabled')
req('#if (APP_NRF24_ENABLED != 0U)' in tasks and '#if (APP_SDLOGGER_ENABLED != 0U)' in app,'optional bus compile isolation')
print('P31 full-system validation: PASS')
