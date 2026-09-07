#include "Services/NRFTelemetry/nrf_telemetry.h"

#include "app.h"
#include "Common/app_config.h"
#include "Modules/NRF24/nrf24.h"
#include "Modules/RemoteControl/remote_control.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Control/NeedleValve/needle_valve_controller.h"
#include "Modules/Control/GeneratedFlightControl/generated_flight_control.h"
#include "Services/PreflightTrigger/preflight_trigger.h"
#include "Services/SystemMonitor/system_monitor.h"
#include "Services/SDLogger/sd_logger.h"
#include "Services/Timebase/timebase.h"
#include "Services/SolenoidOutput/solenoid_output.h"

#include <string.h>

volatile uint8_t nrf_tlm_page_id = 0U;
volatile uint8_t nrf_tlm_sequence = 0U;
volatile uint32_t nrf_tlm_schedule_count = 0UL;
volatile uint32_t nrf_tlm_tx_start_count = 0UL;
volatile uint32_t nrf_tlm_tx_success_count = 0UL;
volatile uint32_t nrf_tlm_tx_fail_count = 0UL;
volatile uint32_t nrf_tlm_pending_replace_count = 0UL;
volatile uint32_t nrf_tlm_last_tx_duration_us = 0UL;
volatile uint32_t nrf_tlm_max_tx_duration_us = 0UL;
volatile uint32_t nrf_tlm_dup_schedule_count = 0UL;
volatile uint32_t nrf_tlm_dup_tx_start_count = 0UL;
volatile uint32_t nrf_tlm_dup_tx_success_count = 0UL;
volatile uint32_t nrf_tlm_dup_tx_fail_count = 0UL;
volatile uint32_t nrf_tlm_dup_cancel_count = 0UL;

#define NRF_TLM_COMMAND_TO_DOWNLINK_DELAY_US  4500UL
#define NRF_TLM_DUPLICATE_DELAY_US            5000UL

static uint8_t next_page = 0U;
static uint8_t fast_since_detail = 0U;
static uint8_t tx_fast_in_flight = 0U;
static uint8_t pending = 0U;
static uint8_t pending_command_sequence = 0U;
static uint32_t pending_due_us = 0UL;
static uint32_t last_seen_valid_packet_count = 0UL;
static uint32_t tx_started_us = 0UL;
static uint8_t tx_page_in_flight = 0U;
static uint8_t duplicate_pending = 0U;
static uint8_t tx_duplicate_in_flight = 0U;
static uint32_t duplicate_due_us = 0UL;
static uint8_t duplicate_packet[NRF_TELEMETRY_PACKET_SIZE];

static uint16_t TlmCrc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0U; b < 8U; b++)
        {
            crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                          : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void PutU32(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v & 0xFFU);
    dst[1] = (uint8_t)((v >> 8) & 0xFFU);
    dst[2] = (uint8_t)((v >> 16) & 0xFFU);
    dst[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void PutI16Pair(uint8_t *dst, int16_t a, int16_t b)
{
    uint16_t ua = (uint16_t)a;
    uint16_t ub = (uint16_t)b;
    dst[0] = (uint8_t)(ua & 0xFFU);
    dst[1] = (uint8_t)((ua >> 8) & 0xFFU);
    dst[2] = (uint8_t)(ub & 0xFFU);
    dst[3] = (uint8_t)((ub >> 8) & 0xFFU);
}

static void PutFloat(uint8_t *dst, float v)
{
    uint32_t u = 0U;
    memcpy(&u, &v, sizeof(u));
    PutU32(dst, u);
}

static void PutU16(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFU);
    dst[1] = (uint8_t)(v >> 8);
}

static void PutI16(uint8_t *dst, int16_t v)
{
    PutU16(dst, (uint16_t)v);
}

static int16_t ScaleI16(float v, float scale)
{
    float x = v * scale;
    if (x != x) x = 0.0f;
    if (x > 32767.0f) x = 32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    x += (x >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)x;
}

static uint16_t ScaleU16(float v, float scale)
{
    float x = v * scale;
    if (x != x) x = 0.0f;
    if (x < 0.0f) x = 0.0f;
    if (x > 65535.0f) x = 65535.0f;
    x += 0.5f;
    return (uint16_t)x;
}

static uint8_t CommonFlags(const SensorData_t *sensor,
                           const BarometerData_t *baro,
                           const LidarData_t *lidar,
                           const FullStateESKFData_t *eskf)
{
    uint8_t f = 0U;
    SystemStatus_t sys = SystemMonitor_GetStatus();
    if (sys.system_ok != 0U) f |= 0x01U;
    if ((sensor != 0) && (sensor->imu_valid != 0U)) f |= 0x02U;
    if ((baro != 0) && (baro->pressure_valid != 0U)) f |= 0x04U;
    if ((lidar != 0) && (lidar->distance_valid != 0U)) f |= 0x08U;
    if ((eskf != 0) && (eskf->healthy != 0U)) f |= 0x10U;
    if (PreflightTrigger_IsFlightActive() != 0U) f |= 0x20U;
    if (App_IsActuatorAuthorized() != 0U) f |= 0x40U;
    if (App_IsStopLatched() != 0U) f |= 0x80U;
    return f;
}


/*
 * P72 FAST flight telemetry.
 *
 * Header is identical to the proven page protocol:
 *   [0] magic, [1] version, [2] 0xFC, [3] telemetry sequence,
 *   [4] command sequence echo, [5] common flags, [6..9] rocket time ms.
 *
 * 20-byte payload:
 *   0..1   height AGL [cm]              int16
 *   2..3   ESKF vertical velocity [cm/s]int16
 *   4..5   pitch [0.01 deg]             int16
 *   6..7   yaw [0.01 deg]               int16
 *   8..9   pitch rate [0.01 deg/s]      int16 (gyro Y filtered)
 *   10..11 yaw rate [0.01 deg/s]        int16 (gyro Z filtered)
 *   12     RCS requested mask
 *   13     RCS applied mask
 *   14..15 needle ADC                    uint16
 *   16..17 main valve command [0..10000] uint16
 *   18..19 estimated thrust [0.1 N]      uint16
 *
 * The control/sensor modules are read-only here. No actuator authority,
 * sensor timing, ESKF timing, command parser, or safety behavior is changed.
 */
static void BuildFast(uint8_t command_sequence, uint8_t out[NRF_TELEMETRY_PACKET_SIZE])
{
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const BarometerData_t *baro = Barometer_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();
    const AttitudeEstimatorData_t *att = AttitudeEstimator_GetDataPtr();
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    GeneratedFlightControlStatus_t main_control = GeneratedFlightControl_GetStatus();
    uint8_t *d = &out[10];
    uint16_t crc;
    float height_agl_m = 0.0f;

    if ((eskf != 0) &&
        (eskf->lidar_reference_m == eskf->lidar_reference_m) &&
        (eskf->position_z_m == eskf->position_z_m))
    {
        height_agl_m = eskf->lidar_reference_m + eskf->position_z_m;
        if (height_agl_m < 0.0f) height_agl_m = 0.0f;
    }

    memset(out, 0, NRF_TELEMETRY_PACKET_SIZE);
    out[0] = NRF_TELEMETRY_MAGIC;
    out[1] = NRF_TELEMETRY_PROTOCOL_VERSION;
    out[2] = NRF_TELEMETRY_FAST_TYPE;
    out[3] = nrf_tlm_sequence;
    out[4] = command_sequence;
    out[5] = CommonFlags(sensor, baro, lidar, eskf);
    PutU32(&out[6], millis());

    PutI16(&d[0], ScaleI16(height_agl_m, 100.0f));
    PutI16(&d[2], ScaleI16((eskf != 0) ? eskf->velocity_z_mps : 0.0f, 100.0f));
    PutI16(&d[4], ScaleI16((att != 0) ? att->pitch_deg : 0.0f, 100.0f));
    PutI16(&d[6], ScaleI16((att != 0) ? att->yaw_deg : 0.0f, 100.0f));
    PutI16(&d[8], ScaleI16((sensor != 0) ? sensor->gyro_y_filtered_dps : 0.0f, 100.0f));
    PutI16(&d[10], ScaleI16((sensor != 0) ? sensor->gyro_z_filtered_dps : 0.0f, 100.0f));
    d[12] = SolenoidOutput_GetRequestedMask();
    d[13] = SolenoidOutput_GetAppliedMask();
    PutU16(&d[14], needle_valve_raw_adc);
    PutU16(&d[16], ScaleU16(main_control.vertical_200hz_valve_cmd, 10000.0f));
    PutU16(&d[18], ScaleU16(main_control.vertical_estimated_thrust_n, 10.0f));

    crc = TlmCrc16(out, 30U);
    out[30] = (uint8_t)(crc & 0xFFU);
    out[31] = (uint8_t)(crc >> 8);
}

static void BuildPage(uint8_t page, uint8_t command_sequence, uint8_t out[NRF_TELEMETRY_PACKET_SIZE])
{
    const SensorData_t *sensor = SensorManager_GetDataPtr();
    const BarometerData_t *baro = Barometer_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();
    const AttitudeEstimatorData_t *att = AttitudeEstimator_GetDataPtr();
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    PreflightTriggerStatus_t pre = PreflightTrigger_GetStatus();
    SystemStatus_t sys = SystemMonitor_GetStatus();
    NRF24_Data_t nrf = NRF24_GetData();
    uint8_t *d = &out[10];
    uint16_t crc;

    memset(out, 0, NRF_TELEMETRY_PACKET_SIZE);
    out[0] = NRF_TELEMETRY_MAGIC;
    out[1] = NRF_TELEMETRY_PROTOCOL_VERSION;
    out[2] = page;
    out[3] = nrf_tlm_sequence;
    out[4] = command_sequence;
    out[5] = CommonFlags(sensor, baro, lidar, eskf);
    PutU32(&out[6], millis());

    switch (page)
    {
        case 0U: /* System / command / actuator */
            d[0] = pre.state;
            d[1] = pre.fault_latched;
            d[2] = RemoteControl_GetCommandFlags();
            d[3] = App_IsStopLatched();
            d[4] = SolenoidOutput_GetRequestedMask();
            d[5] = SolenoidOutput_GetAppliedMask();
            d[6] = needle_valve_enabled;
            d[7] = needle_valve_fault;
            d[8] = (uint8_t)(needle_valve_raw_adc & 0xFFU);
            d[9] = (uint8_t)(needle_valve_raw_adc >> 8);
            d[10] = (uint8_t)(needle_valve_target_adc & 0xFFU);
            d[11] = (uint8_t)(needle_valve_target_adc >> 8);
            d[12] = needle_valve_rpwm;
            d[13] = needle_valve_lpwm;
            d[14] = SDLogger_IsReady();
            d[15] = SDLogger_IsLogging();
            PutU32(&d[16], (uint32_t)sys.fault_code);
            break;

        case 1U: /* IMU raw counts + timestamps */
            PutI16Pair(&d[0], sensor->gyro_x_raw, sensor->gyro_y_raw);
            PutI16Pair(&d[4], sensor->gyro_z_raw, sensor->accel_x_raw);
            PutI16Pair(&d[8], sensor->accel_y_raw, sensor->accel_z_raw);
            PutU32(&d[12], sensor->imu_sample_timestamp_us);
            PutU32(&d[16], sensor->update_count);
            break;

        case 2U: /* IMU gyro scaled + filtered */
            PutFloat(&d[0], sensor->gyro_x_dps);
            PutFloat(&d[4], sensor->gyro_y_dps);
            PutFloat(&d[8], sensor->gyro_z_dps);
            PutFloat(&d[12], sensor->gyro_x_filtered_dps);
            PutFloat(&d[16], sensor->gyro_y_filtered_dps);
            break;

        case 3U: /* IMU acceleration + last filtered gyro */
            PutFloat(&d[0], sensor->gyro_z_filtered_dps);
            PutFloat(&d[4], sensor->accel_x_g);
            PutFloat(&d[8], sensor->accel_y_g);
            PutFloat(&d[12], sensor->accel_z_g);
            PutFloat(&d[16], sensor->accel_norm_g);
            break;

        case 4U: /* IMU filtered acceleration */
            PutFloat(&d[0], sensor->accel_x_filtered_g);
            PutFloat(&d[4], sensor->accel_y_filtered_g);
            PutFloat(&d[8], sensor->accel_z_filtered_g);
            PutFloat(&d[12], sensor->accel_filtered_norm_g);
            PutFloat(&d[16], sensor->baro_temperature_c);
            break;

        case 5U: /* Barometer primary */
            PutFloat(&d[0], baro->pressure_pa);
            PutFloat(&d[4], baro->filtered_pressure_pa);
            PutFloat(&d[8], baro->ground_pressure_pa);
            PutFloat(&d[12], baro->altitude_m);
            PutFloat(&d[16], baro->filtered_altitude_m);
            break;

        case 6U: /* Barometer median/raw/misc */
            PutFloat(&d[0], baro->vertical_speed_mps);
            PutFloat(&d[4], baro->temperature_c);
            PutFloat(&d[8], baro->median_pressure_pa);
            PutU32(&d[12], baro->d1_raw);
            PutU32(&d[16], baro->d2_raw);
            break;

        case 7U: /* LIDAR */
            PutFloat(&d[0], lidar->distance_m);
            PutFloat(&d[4], lidar->median_distance_m);
            PutFloat(&d[8], lidar->filtered_distance_m);
            PutU32(&d[12], lidar->update_count);
            d[16] = lidar->state;
            d[17] = lidar->distance_valid;
            d[18] = (uint8_t)(lidar->distance_cm & 0xFFU);
            d[19] = (uint8_t)(lidar->distance_cm >> 8);
            break;

        case 8U: /* Attitude quaternion */
            PutFloat(&d[0], att->q_w);
            PutFloat(&d[4], att->q_x);
            PutFloat(&d[8], att->q_y);
            PutFloat(&d[12], att->q_z);
            PutFloat(&d[16], att->accel_correction_weight);
            break;

        case 9U: /* Attitude Euler + world linear accel X/Y */
            PutFloat(&d[0], att->roll_deg);
            PutFloat(&d[4], att->pitch_deg);
            PutFloat(&d[8], att->yaw_deg);
            PutFloat(&d[12], att->world_linear_accel_x_mps2);
            PutFloat(&d[16], att->world_linear_accel_y_mps2);
            break;

        case 10U: /* Attitude/world specific force */
            PutFloat(&d[0], att->world_linear_accel_z_mps2);
            PutFloat(&d[4], att->world_specific_force_x_g);
            PutFloat(&d[8], att->world_specific_force_y_g);
            PutFloat(&d[12], att->world_specific_force_z_g);
            PutU32(&d[16], att->update_count);
            break;

        case 11U: /* ESKF position + velocity X/Y */
            PutFloat(&d[0], eskf->position_x_m);
            PutFloat(&d[4], eskf->position_y_m);
            PutFloat(&d[8], eskf->position_z_m);
            PutFloat(&d[12], eskf->velocity_x_mps);
            PutFloat(&d[16], eskf->velocity_y_mps);
            break;

        case 12U: /* ESKF Vz + world acceleration */
            PutFloat(&d[0], eskf->velocity_z_mps);
            PutFloat(&d[4], eskf->world_linear_accel_x_mps2);
            PutFloat(&d[8], eskf->world_linear_accel_y_mps2);
            PutFloat(&d[12], eskf->world_linear_accel_z_mps2);
            PutFloat(&d[16], eskf->vibration_metric_g);
            break;

        case 13U: /* ESKF bias set A */
            PutFloat(&d[0], eskf->accel_bias_x_mps2);
            PutFloat(&d[4], eskf->accel_bias_y_mps2);
            PutFloat(&d[8], eskf->accel_bias_z_mps2);
            PutFloat(&d[12], eskf->gyro_bias_x_dps);
            PutFloat(&d[16], eskf->gyro_bias_y_dps);
            break;

        case 14U: /* ESKF bias/innovation set B */
            PutFloat(&d[0], eskf->gyro_bias_z_dps);
            PutFloat(&d[4], eskf->baro_innovation_m);
            PutFloat(&d[8], eskf->lidar_innovation_m);
            PutFloat(&d[12], eskf->vertical_sensor_consistency_m);
            PutFloat(&d[16], eskf->vertical_reacquire_target_m);
            break;

        case 15U: /* ESKF quaternion + roll */
            PutFloat(&d[0], eskf->q_w);
            PutFloat(&d[4], eskf->q_x);
            PutFloat(&d[8], eskf->q_y);
            PutFloat(&d[12], eskf->q_z);
            PutFloat(&d[16], eskf->roll_deg);
            break;

        case 16U: /* ESKF attitude/reference */
            PutFloat(&d[0], eskf->pitch_deg);
            PutFloat(&d[4], eskf->yaw_deg);
            PutFloat(&d[8], eskf->baro_reference_m);
            PutFloat(&d[12], eskf->lidar_reference_m);
            PutFloat(&d[16], eskf->gravity_correction_weight);
            break;

        case 17U: /* Freshness + source update counts */
            PutU32(&d[0], sys.imu_sample_age_us);
            PutU32(&d[4], sys.lidar_sample_age_us);
            PutU32(&d[8], sys.eskf_public_output_age_us);
            PutU32(&d[12], baro->update_count);
            PutU32(&d[16], lidar->update_count);
            break;

        default: /* 18: estimator/radio diagnostic counters */
            PutU32(&d[0], eskf->public_output_count);
            PutU32(&d[4], eskf->numerical_error_count);
            PutU32(&d[8], eskf->covariance_fault_count);
            PutU32(&d[12], eskf->reset_count);
            PutU32(&d[16], nrf.rx_count);
            break;
    }

    crc = TlmCrc16(out, 30U);
    out[30] = (uint8_t)(crc & 0xFFU);
    out[31] = (uint8_t)(crc >> 8);
}

void NRFTelemetry_Init(void)
{
    next_page = 0U;
    fast_since_detail = 0U;
    tx_fast_in_flight = 0U;
    pending = 0U;
    pending_command_sequence = 0U;
    pending_due_us = 0UL;
    last_seen_valid_packet_count = remote_rx_valid_packet_count;
    tx_started_us = 0UL;
    tx_page_in_flight = 0U;
    duplicate_pending = 0U;
    tx_duplicate_in_flight = 0U;
    duplicate_due_us = 0UL;
    memset(duplicate_packet, 0, sizeof(duplicate_packet));

    nrf_tlm_page_id = 0U;
    nrf_tlm_sequence = 0U;
    nrf_tlm_schedule_count = 0UL;
    nrf_tlm_tx_start_count = 0UL;
    nrf_tlm_tx_success_count = 0UL;
    nrf_tlm_tx_fail_count = 0UL;
    nrf_tlm_pending_replace_count = 0UL;
    nrf_tlm_last_tx_duration_us = 0UL;
    nrf_tlm_max_tx_duration_us = 0UL;
    nrf_tlm_dup_schedule_count = 0UL;
    nrf_tlm_dup_tx_start_count = 0UL;
    nrf_tlm_dup_tx_success_count = 0UL;
    nrf_tlm_dup_tx_fail_count = 0UL;
    nrf_tlm_dup_cancel_count = 0UL;
}

void NRFTelemetry_Service(void)
{
    NRF24_AsyncTxResult_t result;
    uint32_t now_us = micros();
    uint8_t flight_rx_only =
        (PreflightTrigger_IsFlightActive() != 0U) ? 1U : 0U;

    /* Always service the role-switch/TX/RX-return state machine first. */
    NRF24_AsyncTxService();

    result = NRF24_AsyncTxTakeResult();
    if (result != NRF24_ASYNC_TX_RESULT_NONE)
    {
        uint32_t dt = (uint32_t)(now_us - tx_started_us);
        nrf_tlm_last_tx_duration_us = dt;
        if (dt > nrf_tlm_max_tx_duration_us) nrf_tlm_max_tx_duration_us = dt;

        if (tx_duplicate_in_flight != 0U)
        {
            tx_duplicate_in_flight = 0U;
            if (result == NRF24_ASYNC_TX_RESULT_SUCCESS)
            {
                nrf_tlm_dup_tx_success_count++;
            }
            else
            {
                nrf_tlm_dup_tx_fail_count++;
            }
        }
        else if (result == NRF24_ASYNC_TX_RESULT_SUCCESS)
        {
            nrf_tlm_tx_success_count++;

#if (APP_NRF_FLIGHT_MINIMAL_FAST_TDD != 0U)
            if (flight_rx_only != 0U)
            {
                /* R3R10R4: one response is enough to release the existing
                 * ground TDD receive window. Do not schedule a second copy or
                 * rotate detail pages in flight; return to PRX immediately. */
                duplicate_pending = 0U;
                tx_duplicate_in_flight = 0U;
            }
            else
#endif
            {
                /* P73 preflight behavior is retained byte-for-byte in spirit:
                 * a duplicate may improve downlink observability before flight. */
                duplicate_pending = 1U;
                duplicate_due_us = now_us + NRF_TLM_DUPLICATE_DELAY_US;
                nrf_tlm_dup_schedule_count++;

                if (tx_fast_in_flight != 0U)
                {
                    if (fast_since_detail < NRF_TELEMETRY_FAST_PER_DETAIL)
                    {
                        fast_since_detail++;
                    }
                }
                else
                {
                    next_page = (uint8_t)(tx_page_in_flight + 1U);
                    if (next_page >= NRF_TELEMETRY_PAGE_COUNT) next_page = 0U;
                    fast_since_detail = 0U;
                }
            }
        }
        else
        {
            nrf_tlm_tx_fail_count++;
            /* Keep next_page unchanged: the failed page is retried after the
             * next valid command rather than silently creating a page hole. */
        }
    }

#if (APP_NRF_FLIGHT_MINIMAL_FAST_TDD != 0U)
    if (flight_rx_only != 0U)
    {
        /* R8R35R3R10R4: keep the ground station's explicit-TDD heartbeat
         * alive. R3R10R3 RX-only mode proved the rocket receiver itself was
         * healthy (STOP arrived), but the ground station stopped its normal
         * heartbeat while waiting for a downlink response.
         *
         * In flight, each newly accepted 4-byte command schedules exactly one
         * FAST 32-byte response at the proven +4.5 ms point. No duplicate, no
         * 19-page rotation. The async radio state machine returns to PRX before
         * the next command opportunity. */
        if ((duplicate_pending != 0U) && (tx_duplicate_in_flight == 0U))
        {
            duplicate_pending = 0U;
            nrf_tlm_dup_cancel_count++;
        }

        if (remote_rx_valid_packet_count != last_seen_valid_packet_count)
        {
            last_seen_valid_packet_count = remote_rx_valid_packet_count;
            if (pending != 0U) nrf_tlm_pending_replace_count++;
            pending = 1U;
            pending_command_sequence = remote_rx_last_sequence;
            pending_due_us = now_us + NRF_TLM_COMMAND_TO_DOWNLINK_DELAY_US;
            nrf_tlm_schedule_count++;
        }

        if ((pending != 0U) &&
            (NRF24_AsyncTxIsBusy() == 0U) &&
            ((int32_t)(now_us - pending_due_us) >= 0))
        {
            uint8_t packet[NRF_TELEMETRY_PACKET_SIZE];
            BuildFast(pending_command_sequence, packet);
            nrf_tlm_page_id = NRF_TELEMETRY_FAST_TYPE;
            tx_fast_in_flight = 1U;

            if (NRF24_AsyncTxStart(packet, NRF_TELEMETRY_PACKET_SIZE) != 0U)
            {
                pending = 0U;
                tx_started_us = now_us;
                nrf_tlm_sequence++;
                nrf_tlm_tx_start_count++;
            }
        }
        return;
    }
#else
    (void)flight_rx_only;
#endif

    /* Detect a newly accepted command without changing the proven P60 parser. */
    if (remote_rx_valid_packet_count != last_seen_valid_packet_count)
    {
        last_seen_valid_packet_count = remote_rx_valid_packet_count;

        /* A newly received command always has priority over a duplicate that
         * has not started yet. This preserves command-side responsiveness. */
        if ((duplicate_pending != 0U) && (tx_duplicate_in_flight == 0U))
        {
            duplicate_pending = 0U;
            nrf_tlm_dup_cancel_count++;
        }

        if (pending != 0U) nrf_tlm_pending_replace_count++;
        pending = 1U;
        pending_command_sequence = remote_rx_last_sequence;
        pending_due_us = now_us + NRF_TLM_COMMAND_TO_DOWNLINK_DELAY_US;
        nrf_tlm_schedule_count++;
    }

    if ((pending != 0U) &&
        (NRF24_AsyncTxIsBusy() == 0U) &&
        ((int32_t)(now_us - pending_due_us) >= 0))
    {
        uint8_t packet[NRF_TELEMETRY_PACKET_SIZE];

        if (fast_since_detail < NRF_TELEMETRY_FAST_PER_DETAIL)
        {
            BuildFast(pending_command_sequence, packet);
            nrf_tlm_page_id = NRF_TELEMETRY_FAST_TYPE;
            tx_fast_in_flight = 1U;
        }
        else
        {
            BuildPage(next_page, pending_command_sequence, packet);
            nrf_tlm_page_id = next_page;
            tx_page_in_flight = next_page;
            tx_fast_in_flight = 0U;
        }

        if (NRF24_AsyncTxStart(packet, NRF_TELEMETRY_PACKET_SIZE) != 0U)
        {
            memcpy(duplicate_packet, packet, NRF_TELEMETRY_PACKET_SIZE);
            pending = 0U;
            tx_started_us = now_us;
            nrf_tlm_sequence++;
            nrf_tlm_tx_start_count++;
        }
    }
    else if ((duplicate_pending != 0U) &&
             (NRF24_AsyncTxIsBusy() == 0U) &&
             ((int32_t)(now_us - duplicate_due_us) >= 0))
    {
        if (NRF24_AsyncTxStart(duplicate_packet, NRF_TELEMETRY_PACKET_SIZE) != 0U)
        {
            duplicate_pending = 0U;
            tx_duplicate_in_flight = 1U;
            tx_started_us = now_us;
            nrf_tlm_dup_tx_start_count++;
        }
    }
}
