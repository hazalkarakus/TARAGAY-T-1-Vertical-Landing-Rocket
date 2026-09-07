#include "Core/Tasks/app_tasks.h"
#include "app.h"

#include "Common/app_config.h"

#include "Core/Scheduler/scheduler.h"

#include "Modules/Sensors/IMU/imu.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Lidar/Lidar.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Modules/NRF24/nrf24.h"
#include "Modules/RemoteControl/remote_control.h"
#include "Modules/Estimation/AttitudeEstimator/attitude_estimator.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"

#include "Modules/Control/NeedleValve/needle_valve_controller.h"
#include "Modules/Control/GNCActiveControl/gnc_active_control.h"
#include "Modules/Control/ControlInputProvider/control_input_provider.h"
#include "Modules/Control/GeneratedFlightControl/generated_flight_control.h"
#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"
#include "Services/PreflightTrigger/preflight_trigger.h"
#include "../../Modules/Control/VerticalLandingControl/vertical_landing_control.h"
#include "Modules/Control/AttitudeControl/attitude_control.h"
#include "Services/SolenoidOutput/solenoid_output.h"
#include "Services/NeedleValveHardware/needle_valve_hw.h"
#include "Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.h"
#include "Services/SDLogger/sd_logger.h"
#include "Services/BusManager/bus_manager.h"
#include "Services/ServoOutput/servo_output.h"
#include "Services/SensorQualification/sensor_qualification.h"
#include "Services/SystemMonitor/system_monitor.h"

#include "Services/Timebase/timebase.h"

#include <stdint.h>

extern uint8_t _ebss;
extern volatile uint8_t v30_vent_override_active;
#include <math.h>
#include <limits.h>

/*
 * Selected LIDAR live-debug globals already produced by Lidar.c.
 * They are not yet part of the public LidarData_t structure.
 */
extern volatile uint8_t lidar_calibration_complete;
extern volatile uint16_t lidar_calibration_sample_count;
extern volatile uint32_t lidar_probe_attempt_count;
extern volatile uint32_t lidar_probe_success_count;
extern volatile uint32_t lidar_probe_failure_count;
extern volatile uint32_t lidar_bus_recovery_count;
extern volatile uint32_t p34_covariance_last_us;
extern volatile uint32_t p34_covariance_max_us;
extern volatile uint32_t p34_covariance_service_count;
extern volatile uint32_t lidar_last_i2c_error_code;

/* -------------------------------------------------------------------------- */
/* V49 Full-State ESKF signed-PD RCS diagnostics                              */
/* -------------------------------------------------------------------------- */

volatile uint8_t v23_rcs_direct_enabled = 1U;
volatile uint8_t v23_rcs_estimator_healthy = 0U;
volatile uint8_t v23_rcs_requested_mask = 0U;
volatile uint8_t v23_rcs_applied_mask = 0U;
volatile uint8_t v23_rcs_fault = 0U;
volatile uint32_t v23_rcs_control_update_count = 0UL;
volatile uint32_t v23_rcs_output_update_count = 0UL;

static uint32_t v49_last_eskf_public_output_count = 0UL;
static uint32_t v49_last_eskf_reset_count = 0UL;

/* -------------------------------------------------------------------------- */
/* V8.7A diagnostics                                                           */
/* -------------------------------------------------------------------------- */

volatile uint32_t v87_imu_task_counter = 0UL;
volatile uint32_t v87_baro_task_counter = 0UL;
volatile uint32_t v87_lidar_task_counter = 0UL;
volatile uint32_t v87_monitor_task_counter = 0UL;

volatile uint8_t v87_scheduler_ok = 0U;
volatile uint8_t v87_imu_ok = 0U;
volatile uint8_t v87_baro_ok = 0U;
volatile uint8_t v87_lidar_ok = 0U;
volatile uint8_t v87_valve_tick_ok = 0U;
volatile uint8_t v87_adc_ok = 0U;
volatile uint8_t v87_integration_ok = 0U;


/* -------------------------------------------------------------------------- */
/* V8.9 NRF24 / SPI3 monitor-only diagnostics                                 */
/* -------------------------------------------------------------------------- */

volatile uint8_t v89_scheduler_ok = 0U;
volatile uint8_t v89_nrf_hw_ok = 0U;
volatile uint8_t v89_integration_ok = 0U;

volatile uint8_t v89_nrf_initialized = 0U;
volatile uint8_t v89_nrf_connected = 0U;
volatile uint8_t v89_nrf_mode = 0U;
volatile uint8_t v89_nrf_status_reg = 0U;
volatile uint8_t v89_nrf_config_reg = 0U;
volatile uint8_t v89_nrf_rf_ch_reg = 0U;
volatile uint8_t v89_nrf_rf_setup_reg = 0U;
volatile uint8_t v89_nrf_fifo_status_reg = 0U;

volatile uint8_t v89_remote_link_active = 0U;
volatile uint8_t v89_remote_command = 0U;
volatile uint8_t v89_remote_last_sequence = 0U;

volatile uint32_t v89_nrf_rx_count = 0UL;
volatile uint32_t v89_nrf_tx_count = 0UL;
volatile uint32_t v89_nrf_tx_fail_count = 0UL;
volatile uint32_t v89_nrf_error_count = 0UL;

volatile uint32_t v89_remote_valid_packet_count = 0UL;
volatile uint32_t v89_remote_invalid_packet_count = 0UL;
volatile uint32_t v89_remote_timeout_count = 0UL;
volatile uint32_t v89_remote_irq_count = 0UL;
volatile uint32_t v89_remote_last_packet_age_ms = 0xFFFFFFFFUL;

volatile uint32_t v89_nrf_task_counter = 0UL;
volatile uint32_t v89_nrf_task_delta_100ms = 0UL;
volatile uint32_t v89_nrf_task_last_exec_us = 0UL;
volatile uint32_t v89_nrf_task_max_exec_us = 0UL;
volatile uint32_t v89_nrf_task_overrun_count = 0UL;
volatile uint32_t v89_nrf_task_deadline_miss_count = 0UL;

volatile uint8_t v89_spi3_ok = 0U;
volatile uint8_t v89_spi3_fault = 0U;
volatile uint8_t v89_spi3_last_error = 0U;
volatile uint32_t v89_spi3_transaction_count = 0UL;
volatile uint32_t v89_spi3_error_count = 0UL;
volatile uint32_t v89_spi3_timeout_count = 0UL;
volatile uint32_t v89_spi3_busy_count = 0UL;
volatile uint32_t v89_spi3_slow_count = 0UL;
volatile uint32_t v89_spi3_last_duration_us = 0UL;
volatile uint32_t v89_spi3_max_duration_us = 0UL;


/* -------------------------------------------------------------------------- */
/* V8.10 NRF -> tahliye servo diagnostics                                     */
/* -------------------------------------------------------------------------- */

volatile uint8_t v810_servo_ok = 0U;
volatile uint8_t v810_integration_ok = 0U;

volatile uint8_t v810_servo_init_ok = 0U;
volatile uint8_t v810_servo_remote_command = 0U;
volatile uint8_t v810_servo_link_active = 0U;
volatile uint8_t v810_servo_target_open = 0U;
volatile uint16_t v810_servo_pulse_us = 0U;

volatile uint32_t v810_servo_update_count = 0UL;
volatile uint32_t v810_servo_command_change_count = 0UL;


/* -------------------------------------------------------------------------- */
/* V8.11 cleanup diagnostics                                                  */
/* -------------------------------------------------------------------------- */

volatile uint8_t v811_lidar_sample_rate_ok = 0U;
volatile uint32_t v811_lidar_sample_delta_100ms = 0UL;
volatile uint8_t v811_integration_ok = 0U;


volatile uint8_t v813_lidar_profile_config_ok = 0U;
volatile uint8_t v813_lidar_200hz_ok = 0U;
volatile uint32_t v813_lidar_sample_delta_100ms = 0UL;
volatile float v813_lidar_sample_rate_hz = 0.0f;
volatile uint8_t v813_integration_ok = 0U;

volatile uint8_t v813_core_without_baro_ok = 0U;
volatile uint8_t v813_sd_accounting_ok = 0U;


/* V8.14 robust 1-second LIDAR cadence diagnostics. */
volatile uint32_t v814_lidar_sample_delta_1s = 0UL;
volatile float v814_lidar_sample_rate_1s_hz = 0.0f;
volatile float v814_lidar_sample_rate_1s_min_hz = 0.0f;
volatile float v814_lidar_sample_rate_1s_max_hz = 0.0f;
volatile uint32_t v814_lidar_low_rate_window_count = 0UL;
volatile uint8_t v814_lidar_rate_1s_ok = 0U;
volatile uint8_t v814_integration_ok = 0U;


/* -------------------------------------------------------------------------- */
/* V8.15 Full-State ESKF SHADOW integration diagnostics                       */
/* -------------------------------------------------------------------------- */

volatile uint8_t v815_attitude_ok = 0U;
volatile uint8_t v815_eskf_initialized = 0U;
volatile uint8_t v815_eskf_healthy = 0U;
volatile uint8_t v815_eskf_shadow_mode = 0U;
volatile uint8_t v815_eskf_lidar_reference_ready = 0U;
volatile uint8_t v815_eskf_baro_reference_ready = 0U;
volatile uint8_t v815_eskf_stationary_detected = 0U;

volatile uint8_t v815_eskf_rate_ok = 0U;
volatile uint8_t v815_eskf_shadow_core_ok = 0U;
volatile uint8_t v815_integration_ok = 0U;


/* V8.15C firmware/RAM identity diagnostics. */
const uint32_t v815c_flash_magic __attribute__((used)) = 0x815C15C1UL;
volatile uint32_t v815c_ram_magic = 0x815C15C2UL;

volatile uint8_t v815c_firmware_identity_ok = 0U;
volatile uint8_t v815c_ram_magic_ok = 0U;
volatile uint8_t v815c_sd_ring_in_ccm = 0U;
volatile uint8_t v815c_estimator_basic_sanity_ok = 0U;

volatile uint32_t v815c_bss_end_address = 0UL;
volatile uint32_t v815c_msp_address = 0UL;
volatile uint32_t v815c_main_sram_gap_bytes = 0UL;
volatile uint32_t v815c_main_sram_gap_min_bytes = 0xFFFFFFFFUL;




/* -------------------------------------------------------------------------- */
/* V8.16 fixed-address GNC diagnostic block @ 0x1000F100                     */
/* -------------------------------------------------------------------------- */

#define V816_GNC_DIAG_MAGIC             0x819C1909UL
#define V816_GNC_DIAG_VERSION           0x00081909UL
#define V816_GNC_DIAG_WORD_COUNT        32U
#define V816_GNC_DIAG_BYTES             (V816_GNC_DIAG_WORD_COUNT * 4U)

typedef struct
{
    /* 0x00 */ uint32_t magic;
    /* 0x04 */ uint32_t version;
    /* 0x08 */ uint32_t block_size;
    /* 0x0C */ uint32_t sequence_begin;

    /* 0x10 */ uint32_t flags;
    /* 0x14 */ uint32_t button_stage;
    /* 0x18 */ uint32_t fault_code;
    /* 0x1C */ uint32_t attitude_ctrl_state;

    /* 0x20 */ uint32_t rcs_requested_mask;
    /* 0x24 */ uint32_t rcs_applied_mask;
    /* 0x28 */ uint32_t needle_requested_x10000;
    /* 0x2C */ uint32_t needle_limited_x10000;

    /* 0x30 */ uint32_t vertical_cmd_x10000;
    /* 0x34 */ int32_t height_agl_mm;
    /* 0x38 */ int32_t z_cg_mm;
    /* 0x3C */ int32_t vertical_velocity_mmps;

    /* 0x40 */ int32_t roll_mdeg;
    /* 0x44 */ int32_t pitch_mdeg;
    /* 0x48 */ int32_t yaw_mdeg;
    /* 0x4C */ int32_t roll_rate_mdps;
    /* 0x50 */ int32_t pitch_rate_mdps;

    /* 0x54 */ int32_t vertical_ref_speed_mmps;
    /* 0x58 */ int32_t vertical_speed_error_mmps;

    /* 0x5C */ int32_t rcs_roll_predicted_mdeg;
    /* 0x60 */ int32_t rcs_pitch_predicted_mdeg;

    /* 0x64 */ uint32_t needle_raw_adc;
    /* 0x68 */ uint32_t needle_target_adc;
    /* 0x6C */ int32_t needle_error_adc;

    /* 0x70 */ uint32_t update_200hz_count;
    /* 0x74 */ uint32_t service_1khz_count;

    /* 0x78 */ uint32_t sequence_end;
    /* 0x7C */ uint32_t checksum_xor;

} V816GNCDiagnostic_t;

typedef char V816GNCDiagnosticSizeCheck[
    (sizeof(V816GNCDiagnostic_t) == V816_GNC_DIAG_BYTES) ? 1 : -1
];

volatile V816GNCDiagnostic_t v816_gnc_diag
    __attribute__((section(".diag_gnc"), aligned(4), used));

static uint32_t v816_gnc_diag_sequence = 0UL;

static int32_t V816_ScaleFloat(float value, float scale)
{
    float scaled;

    if (isfinite(value) == 0)
    {
        return INT32_MIN;
    }

    scaled = value * scale;

    if (scaled > 2147483000.0f)
    {
        return INT32_MAX;
    }

    if (scaled < -2147483000.0f)
    {
        return INT32_MIN + 1;
    }

    return (int32_t)scaled;
}

static uint32_t V816_ClampCmdX10000(float value)
{
    if (isfinite(value) == 0)
    {
        return 0xFFFFFFFFUL;
    }

    if (value <= 0.0f)
    {
        return 0UL;
    }

    if (value >= 1.0f)
    {
        return 10000UL;
    }

    return (uint32_t)(value * 10000.0f + 0.5f);
}

static uint32_t V816_GNCDiagnosticChecksum(void)
{
    const volatile uint32_t *words =
        (const volatile uint32_t *)(const void *)&v816_gnc_diag;
    uint32_t checksum = 0x16C1A55AUL;
    uint32_t i;

    /* Check words 0..30; word31 is checksum. */
    for (i = 0UL; i < 31UL; i++)
    {
        checksum ^= words[i];
    }

    return checksum;
}

static void V816_UpdateGNCDiagnostic(void)
{
    NeedleValveStatus_t needle =
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
        NeedleValveAutonomousControl_GetTelemetryStatus();
#else
        NeedleValveController_GetStatus();
#endif
    AttitudeControlStatus_t attitude =
        AttitudeControl_GetStatus();

    uint32_t flags = 0UL;
    uint32_t seq = v816_gnc_diag_sequence + 2UL;

    if ((seq & 1UL) != 0UL)
    {
        seq++;
    }
    v816_gnc_diag_sequence = seq;

    v816_gnc_diag.sequence_begin = seq | 1UL;
    v816_gnc_diag.sequence_end = 0UL;

    v816_gnc_diag.magic = V816_GNC_DIAG_MAGIC;
    v816_gnc_diag.version = V816_GNC_DIAG_VERSION;
    v816_gnc_diag.block_size = V816_GNC_DIAG_BYTES;

    flags |= (1UL << 0); /* alive */

    if (gnc_v816_armed != 0U)                flags |= (1UL << 1);
    if (gnc_v816_fault_latched != 0U)        flags |= (1UL << 2);
    if (gnc_v816_estimator_ok != 0U)         flags |= (1UL << 3);
    if (gnc_v816_attitude_state_ok != 0U)    flags |= (1UL << 4);
    if (gnc_v816_vertical_state_ok != 0U)    flags |= (1UL << 5);
    if (gnc_v816_needle_ok != 0U)            flags |= (1UL << 6);
    if (gnc_v816_disarm_closing != 0U)       flags |= (1UL << 7);
    if (gnc_v816_rcs_dry_run != 0U)          flags |= (1UL << 8);

    if (needle.zero_valid != 0U)             flags |= (1UL << 9);
    if (needle.enabled != 0U)                flags |= (1UL << 10);
    if (needle.fault == NEEDLE_VALVE_FAULT_NONE)
                                                flags |= (1UL << 11);

    if (attitude.estimator_healthy != 0U)     flags |= (1UL << 12);
    if (attitude.attitude_fresh != 0U)        flags |= (1UL << 13);
    if (attitude.landing_prediction_valid != 0U)
                                                flags |= (1UL << 14);
    if (attitude.fault == ATT_CTRL_FAULT_NONE) flags |= (1UL << 15);

    /* V8.19F+: GNC roll/pitch/yaw source is the classic AttitudeEstimator. */
    flags |= (1UL << 16);

#if (APP_GNC_RCS_RELAY_BENCH_MODE != 0U)
    /* Legacy relay-only bench-mode identity bit. */
    flags |= (1UL << 17);
#endif

#if (APP_GNC_COMBINED_DRY_RUN_MODE != 0U)
    /* V8.19I: physical RCS + compute-only vertical controller. */
    flags |= (1UL << 18);
#endif

    v816_gnc_diag.flags = flags;
    v816_gnc_diag.button_stage = gnc_v816_button_stage;
    v816_gnc_diag.fault_code = gnc_v816_fault_code;
    v816_gnc_diag.attitude_ctrl_state = (uint32_t)attitude.state;

    v816_gnc_diag.rcs_requested_mask =
        gnc_v816_rcs_requested_mask;
    v816_gnc_diag.rcs_applied_mask =
        gnc_v816_rcs_applied_mask;

    v816_gnc_diag.needle_requested_x10000 =
        V816_ClampCmdX10000(needle.requested_cmd);
    v816_gnc_diag.needle_limited_x10000 =
        V816_ClampCmdX10000(needle.limited_cmd);

    v816_gnc_diag.vertical_cmd_x10000 =
        V816_ClampCmdX10000(gnc_v816_vertical_valve_cmd);

    v816_gnc_diag.height_agl_mm =
        V816_ScaleFloat(gnc_v816_height_agl_m, 1000.0f);
    v816_gnc_diag.z_cg_mm =
        V816_ScaleFloat(gnc_v816_z_cg_m, 1000.0f);
    v816_gnc_diag.vertical_velocity_mmps =
        V816_ScaleFloat(gnc_v816_vertical_velocity_mps, 1000.0f);

    v816_gnc_diag.roll_mdeg =
        V816_ScaleFloat(gnc_v816_roll_deg, 1000.0f);
    v816_gnc_diag.pitch_mdeg =
        V816_ScaleFloat(gnc_v816_pitch_deg, 1000.0f);
    v816_gnc_diag.yaw_mdeg =
        V816_ScaleFloat(gnc_v816_yaw_deg, 1000.0f);
    v816_gnc_diag.roll_rate_mdps =
        V816_ScaleFloat(gnc_v816_roll_rate_dps, 1000.0f);
    v816_gnc_diag.pitch_rate_mdps =
        V816_ScaleFloat(gnc_v816_pitch_rate_dps, 1000.0f);

    v816_gnc_diag.vertical_ref_speed_mmps =
        V816_ScaleFloat(
            gnc_v816_vertical_reference_speed_mps,
            1000.0f
        );
    v816_gnc_diag.vertical_speed_error_mmps =
        V816_ScaleFloat(
            gnc_v816_vertical_speed_error_mps,
            1000.0f
        );

    v816_gnc_diag.rcs_roll_predicted_mdeg =
        V816_ScaleFloat(
            attitude.roll_predicted_touch_angle_deg,
            1000.0f
        );
    v816_gnc_diag.rcs_pitch_predicted_mdeg =
        V816_ScaleFloat(
            attitude.pitch_predicted_touch_angle_deg,
            1000.0f
        );

    v816_gnc_diag.needle_raw_adc = needle.raw_adc;
    v816_gnc_diag.needle_target_adc = needle.target_adc;
    v816_gnc_diag.needle_error_adc = needle.error_adc;

    v816_gnc_diag.update_200hz_count =
        gnc_v816_update_200hz_count;
    v816_gnc_diag.service_1khz_count =
        gnc_v816_service_1khz_count;

    v816_gnc_diag.sequence_begin = seq;
    v816_gnc_diag.sequence_end = seq;
    v816_gnc_diag.checksum_xor =
        V816_GNCDiagnosticChecksum();
}


/* V8.19I fixed vertical-controller diagnostic @ 0x1000F180 */
#define V819_VERTICAL_DIAG_MAGIC      0x819C19D1UL
#define V819_VERTICAL_DIAG_VERSION    0x00081909UL
#define V819_VERTICAL_DIAG_BYTES      64U

typedef struct
{
    uint32_t magic, version, block_size, flags;
    int32_t z_mm, v_mmps, mass_g, pressure_mbar;
    uint32_t valve_cmd_x10000;
    uint32_t z_source, v_source, mass_source, pressure_source;
    uint32_t sequence_begin, sequence_end, checksum_xor;
} V819VerticalDiagnostic_t;

typedef char V819VerticalDiagnosticSizeCheck[
    (sizeof(V819VerticalDiagnostic_t) == V819_VERTICAL_DIAG_BYTES) ? 1 : -1
];

volatile V819VerticalDiagnostic_t v819_vertical_diag
    __attribute__((section(".diag_matlab"), aligned(4), used));
static uint32_t v819_vertical_diag_sequence = 0UL;

static uint32_t V819_VerticalChecksum(void)
{
    const volatile uint32_t *w=(const volatile uint32_t*)(const void*)&v819_vertical_diag;
    uint32_t c=0x17C1A55AUL, i;
    for(i=0UL;i<15UL;i++) c^=w[i];
    return c;
}

static void V819_UpdateVerticalDiagnostic(void)
{
    ControlInputProviderData_t in=ControlInputProvider_GetData();
    uint32_t flags=0UL;
    uint32_t seq=v819_vertical_diag_sequence+2UL;
    if(seq&1UL) seq++;
    v819_vertical_diag_sequence=seq;
    if(in.z_valid) flags|=(1UL<<0);
    if(in.v_valid) flags|=(1UL<<1);
    if(in.mass_valid) flags|=(1UL<<2);
    if(in.pressure_valid) flags|=(1UL<<3);
    if(in.all_valid) flags|=(1UL<<4);
    if(gnc_v816_vertical_state_ok) flags|=(1UL<<5);
    if(APP_GNC_VERTICAL_CONTROLLER_IMPLEMENTED) flags|=(1UL<<6);
    if(in.all_valid && isfinite(gnc_v816_vertical_valve_cmd)) flags|=(1UL<<7);
    v819_vertical_diag.magic=V819_VERTICAL_DIAG_MAGIC;
    v819_vertical_diag.version=V819_VERTICAL_DIAG_VERSION;
    v819_vertical_diag.block_size=V819_VERTICAL_DIAG_BYTES;
    v819_vertical_diag.flags=flags;
    v819_vertical_diag.z_mm=V816_ScaleFloat(in.z_m,1000.0f);
    v819_vertical_diag.v_mmps=V816_ScaleFloat(in.v_mps,1000.0f);
    v819_vertical_diag.mass_g=V816_ScaleFloat(in.mass_kg,1000.0f);
    v819_vertical_diag.pressure_mbar=V816_ScaleFloat(in.main_pressure_bar,1000.0f);
    v819_vertical_diag.valve_cmd_x10000=V816_ClampCmdX10000(gnc_v816_vertical_valve_cmd);
    v819_vertical_diag.z_source=in.z_source;
    v819_vertical_diag.v_source=in.v_source;
    v819_vertical_diag.mass_source=in.mass_source;
    v819_vertical_diag.pressure_source=in.pressure_source;
    v819_vertical_diag.sequence_begin=seq;
    v819_vertical_diag.sequence_end=seq;
    v819_vertical_diag.checksum_xor=V819_VerticalChecksum();
}

/* -------------------------------------------------------------------------- */
/* V8.15D fixed-address raw diagnostic block                                  */
/* -------------------------------------------------------------------------- */

#define V815D_DIAG_FIXED_ADDRESS       0x1000F000UL
#define V815D_DIAG_MAGIC               0x815D15D1UL
#define V815D_DIAG_ENDIAN_MAGIC        0x11223344UL
#define V815D_DIAG_VERSION             0x0008150DUL
#define V815D_DIAG_WORD_COUNT          48U
#define V815D_DIAG_BLOCK_BYTES         (V815D_DIAG_WORD_COUNT * 4U)
#define V815D_DIAG_FLOAT_INVALID       ((int32_t)INT32_MIN)

typedef struct
{
    /* 0x00 */ uint32_t magic;
    /* 0x04 */ uint32_t endian_magic;
    /* 0x08 */ uint32_t version;
    /* 0x0C */ uint32_t block_size_bytes;
    /* 0x10 */ uint32_t sequence_begin;
    /* 0x14 */ uint32_t flags;

    /* 0x18 */ uint32_t attitude_update_count;
    /* 0x1C */ uint32_t eskf_predict_count;
    /* 0x20 */ uint32_t eskf_public_count;
    /* 0x24 */ uint32_t eskf_covariance_count;
    /* 0x28 */ uint32_t eskf_correction_count;

    /* 0x2C */ uint32_t cpu_load_x100;
    /* 0x30 */ uint32_t cpu_idle_x100;

    /* 0x34 */ uint32_t baro_update_count;
    /* 0x38 */ int32_t baro_pressure_pa;
    /* 0x3C */ int32_t baro_temperature_mC;
    /* 0x40 */ int32_t baro_altitude_mm;

    /* 0x44 */ uint32_t lidar_update_count;
    /* 0x48 */ int32_t lidar_distance_mm;

    /* 0x4C */ uint32_t baro_age_us;
    /* 0x50 */ uint32_t lidar_age_us;

    /* 0x54 */ uint32_t origin_zero_count;
    /* 0x58 */ uint32_t eskf_numerical_error_count;

    /* 0x5C */ uint32_t sd_error_count;
    /* 0x60 */ uint32_t sd_write_error_count;

    /* 0x64 */ uint32_t sd_ring_address;
    /* 0x68 */ uint32_t sd_ring_end_address;

    /* 0x6C */ uint32_t bss_end_address;
    /* 0x70 */ uint32_t msp_address;
    /* 0x74 */ uint32_t main_sram_gap_bytes;

    /* 0x78 */ int32_t position_x_mm;
    /* 0x7C */ int32_t position_y_mm;
    /* 0x80 */ int32_t position_z_mm;

    /* 0x84 */ int32_t velocity_x_mmps;
    /* 0x88 */ int32_t velocity_y_mmps;
    /* 0x8C */ int32_t velocity_z_mmps;

    /* 0x90 */ int32_t roll_mdeg;
    /* 0x94 */ int32_t pitch_mdeg;
    /* 0x98 */ int32_t yaw_mdeg;

    /* 0x9C */ int32_t baro_reference_mm;
    /* 0xA0 */ int32_t lidar_reference_mm;
    /* 0xA4 */ int32_t baro_innovation_mm;
    /* 0xA8 */ int32_t lidar_innovation_mm;

    /* 0xAC */ uint32_t eskf_reset_count;
    /* 0xB0 */ uint32_t eskf_gap_skip_count;

    /* 0xB4 */ uint32_t sequence_end;
    /* 0xB8 */ uint32_t checksum_xor;
    /* 0xBC */ uint32_t checksum_xor_inv;

} V815DFixedDiagnostic_t;

typedef char V815DFixedDiagnosticSizeCheck[
    (sizeof(V815DFixedDiagnostic_t) == V815D_DIAG_BLOCK_BYTES) ? 1 : -1
];

/*
 * NO initializer on purpose: .diag_fixed is NOLOAD.
 * Firmware fills every word at runtime.
 */
volatile V815DFixedDiagnostic_t v815d_diag
    __attribute__((section(".diag_fixed"), aligned(4), used));

static uint32_t v815d_diag_sequence = 0UL;

static int32_t V815D_ScaleFloatToI32(float value, float scale)
{
    float scaled;

    if (isfinite(value) == 0)
    {
        return V815D_DIAG_FLOAT_INVALID;
    }

    scaled = value * scale;

    if (scaled >= 2147483000.0f)
    {
        return INT32_MAX;
    }

    if (scaled <= -2147483000.0f)
    {
        return INT32_MIN + 1;
    }

    return (int32_t)scaled;
}

static uint32_t V815D_AgeUs(uint32_t now_us, uint32_t timestamp_us)
{
    if (timestamp_us == 0UL)
    {
        return 0xFFFFFFFFUL;
    }

    return (uint32_t)(now_us - timestamp_us);
}

static uint32_t V815D_ComputeChecksum(void)
{
    const volatile uint32_t *word =
        (const volatile uint32_t *)(const void *)&v815d_diag;

    uint32_t checksum = 0xA5A55A5AUL;
    uint32_t i;

    /*
     * Words 0..45 are protected.
     * 46 = checksum
     * 47 = checksum inverse
     */
    for (i = 0UL; i < 46UL; i++)
    {
        checksum ^= word[i];
    }

    return checksum;
}

static void V815D_UpdateFixedDiagnostic(void)
{
    const AttitudeEstimatorData_t *attitude =
        AttitudeEstimator_GetDataPtr();
    const FullStateESKFData_t *eskf =
        FullStateESKF_GetDataPtr();
    const BarometerData_t *barometer =
        Barometer_GetDataPtr();
    const LidarData_t *lidar =
        Lidar_GetDataPtr();

    uint32_t now_us = micros();
    uint32_t msp_now;
    uint32_t flags = 0UL;
    uint32_t stable_sequence;
    uint8_t boolean_sanity = 1U;
    uint8_t state_finite = 1U;

    __asm volatile ("mrs %0, msp" : "=r" (msp_now));

    /*
     * Seqlock-style marker. An odd sequence means the block was being updated
     * when memory was sampled. Stable snapshots have begin == end and EVEN.
     */
    stable_sequence = v815d_diag_sequence + 2UL;
    if ((stable_sequence & 1UL) != 0UL)
    {
        stable_sequence++;
    }
    v815d_diag_sequence = stable_sequence;

    v815d_diag.sequence_begin = stable_sequence | 1UL;
    v815d_diag.sequence_end = 0UL;

    v815d_diag.magic = V815D_DIAG_MAGIC;
    v815d_diag.endian_magic = V815D_DIAG_ENDIAN_MAGIC;
    v815d_diag.version = V815D_DIAG_VERSION;
    v815d_diag.block_size_bytes = V815D_DIAG_BLOCK_BYTES;

    flags |= (1UL << 0); /* block alive */

    if (attitude != 0)
    {
        if (attitude->enabled == 1U)     flags |= (1UL << 1);
        if (attitude->initialized == 1U) flags |= (1UL << 2);
        if (attitude->healthy == 1U)     flags |= (1UL << 3);

        if ((attitude->enabled > 1U) ||
            (attitude->initialized > 1U) ||
            (attitude->healthy > 1U))
        {
            boolean_sanity = 0U;
        }
    }
    else
    {
        boolean_sanity = 0U;
    }

    if (eskf != 0)
    {
        if (eskf->enabled == 1U)                     flags |= (1UL << 4);
        if (eskf->shadow_mode == 1U)                 flags |= (1UL << 5);
        if (eskf->initialized == 1U)                 flags |= (1UL << 6);
        if (eskf->healthy == 1U)                     flags |= (1UL << 7);
        if (eskf->stationary_detected == 1U)         flags |= (1UL << 8);
        if (eskf->baro_fresh == 1U)                  flags |= (1UL << 11);
        if (eskf->baro_reference_ready == 1U)        flags |= (1UL << 12);
        if (eskf->lidar_fresh == 1U)                 flags |= (1UL << 15);
        if (eskf->lidar_reference_ready == 1U)       flags |= (1UL << 16);
        if (eskf->origin_zeroed == 1U)               flags |= (1UL << 17);
        if (eskf->horizontal_position_valid == 1U)   flags |= (1UL << 18);
        if (eskf->vertical_position_valid == 1U)     flags |= (1UL << 19);

        if ((eskf->enabled > 1U) ||
            (eskf->shadow_mode > 1U) ||
            (eskf->initialized > 1U) ||
            (eskf->healthy > 1U) ||
            (eskf->stationary_detected > 1U) ||
            (eskf->baro_fresh > 1U) ||
            (eskf->baro_reference_ready > 1U) ||
            (eskf->lidar_fresh > 1U) ||
            (eskf->lidar_reference_ready > 1U) ||
            (eskf->origin_zeroed > 1U) ||
            (eskf->horizontal_position_valid > 1U) ||
            (eskf->vertical_position_valid > 1U))
        {
            boolean_sanity = 0U;
        }

        if ((isfinite(eskf->position_x_m) == 0) ||
            (isfinite(eskf->position_y_m) == 0) ||
            (isfinite(eskf->position_z_m) == 0) ||
            (isfinite(eskf->velocity_x_mps) == 0) ||
            (isfinite(eskf->velocity_y_mps) == 0) ||
            (isfinite(eskf->velocity_z_mps) == 0) ||
            (isfinite(eskf->roll_deg) == 0) ||
            (isfinite(eskf->pitch_deg) == 0) ||
            (isfinite(eskf->yaw_deg) == 0))
        {
            state_finite = 0U;
        }
    }
    else
    {
        boolean_sanity = 0U;
        state_finite = 0U;
    }

    if ((barometer != 0) &&
        (barometer->connected == 1U))
    {
        flags |= (1UL << 9);
    }

    if ((barometer != 0) &&
        (barometer->healthy == 1U))
    {
        flags |= (1UL << 10);
    }

    if ((barometer != 0) &&
        (barometer->pressure_valid == 1U))
    {
        flags |= (1UL << 26);
    }

    if ((lidar != 0) &&
        (lidar->connected == 1U))
    {
        flags |= (1UL << 13);
    }

    if ((lidar != 0) &&
        (lidar->distance_valid == 1U))
    {
        flags |= (1UL << 14);
        flags |= (1UL << 27);
    }

    if (state_finite != 0U) flags |= (1UL << 20);
    if (boolean_sanity != 0U) flags |= (1UL << 21);

    if ((sd_logger_ring_address >= 0x10000000UL) &&
        (sd_logger_ring_end_address <= 0x1000F000UL) &&
        (sd_logger_ring_end_address > sd_logger_ring_address))
    {
        flags |= (1UL << 22);
    }

    if (sd_logger_ready == 1U)          flags |= (1UL << 23);
    if (sd_logger_logging_active == 1U) flags |= (1UL << 24);

    if (v815c_ram_magic == 0x815C15C2UL)
    {
        flags |= (1UL << 25);
    }

    v815d_diag.flags = flags;

    v815d_diag.attitude_update_count =
        (attitude != 0) ? attitude->update_count : 0UL;
    v815d_diag.eskf_predict_count =
        (eskf != 0) ? eskf->predict_count : 0UL;
    v815d_diag.eskf_public_count =
        (eskf != 0) ? eskf->public_output_count : 0UL;
    v815d_diag.eskf_covariance_count =
        (eskf != 0) ? eskf->covariance_predict_count : 0UL;
    v815d_diag.eskf_correction_count =
        v815_eskf_correction_task_counter;

    /* Scheduler already maintains exact percent-x100 counters. */
    v815d_diag.cpu_load_x100 = cpu_load_percent_x100;
    v815d_diag.cpu_idle_x100 = cpu_idle_percent_x100;

    if (barometer != 0)
    {
        v815d_diag.baro_update_count = barometer->update_count;

        if ((barometer->connected == 1U) &&
            (barometer->pressure_valid == 1U) &&
            (isfinite(barometer->pressure_pa) != 0))
        {
            v815d_diag.baro_pressure_pa =
                V815D_ScaleFloatToI32(barometer->pressure_pa, 1.0f);
        }
        else
        {
            v815d_diag.baro_pressure_pa = V815D_DIAG_FLOAT_INVALID;
        }

        v815d_diag.baro_temperature_mC =
            V815D_ScaleFloatToI32(barometer->temperature_c, 1000.0f);

        v815d_diag.baro_altitude_mm =
            V815D_ScaleFloatToI32(
                barometer->filtered_altitude_m,
                1000.0f
            );

        v815d_diag.baro_age_us =
            V815D_AgeUs(now_us, barometer->last_sample_timestamp_us);
    }
    else
    {
        v815d_diag.baro_update_count = 0UL;
        v815d_diag.baro_pressure_pa = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.baro_temperature_mC = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.baro_altitude_mm = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.baro_age_us = 0xFFFFFFFFUL;
    }

    if (lidar != 0)
    {
        v815d_diag.lidar_update_count = lidar->update_count;
        v815d_diag.lidar_distance_mm =
            V815D_ScaleFloatToI32(lidar->filtered_distance_m, 1000.0f);
        v815d_diag.lidar_age_us =
            V815D_AgeUs(now_us, lidar->last_sample_timestamp_us);
    }
    else
    {
        v815d_diag.lidar_update_count = 0UL;
        v815d_diag.lidar_distance_mm = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.lidar_age_us = 0xFFFFFFFFUL;
    }

    if (eskf != 0)
    {
        v815d_diag.origin_zero_count = eskf->origin_zero_count;
        v815d_diag.eskf_numerical_error_count =
            eskf->numerical_error_count;

        v815d_diag.position_x_mm =
            V815D_ScaleFloatToI32(eskf->position_x_m, 1000.0f);
        v815d_diag.position_y_mm =
            V815D_ScaleFloatToI32(eskf->position_y_m, 1000.0f);
        v815d_diag.position_z_mm =
            V815D_ScaleFloatToI32(eskf->position_z_m, 1000.0f);

        v815d_diag.velocity_x_mmps =
            V815D_ScaleFloatToI32(eskf->velocity_x_mps, 1000.0f);
        v815d_diag.velocity_y_mmps =
            V815D_ScaleFloatToI32(eskf->velocity_y_mps, 1000.0f);
        v815d_diag.velocity_z_mmps =
            V815D_ScaleFloatToI32(eskf->velocity_z_mps, 1000.0f);

        v815d_diag.roll_mdeg =
            V815D_ScaleFloatToI32(eskf->roll_deg, 1000.0f);
        v815d_diag.pitch_mdeg =
            V815D_ScaleFloatToI32(eskf->pitch_deg, 1000.0f);
        v815d_diag.yaw_mdeg =
            V815D_ScaleFloatToI32(eskf->yaw_deg, 1000.0f);

        v815d_diag.baro_reference_mm =
            V815D_ScaleFloatToI32(eskf->baro_reference_m, 1000.0f);
        v815d_diag.lidar_reference_mm =
            V815D_ScaleFloatToI32(eskf->lidar_reference_m, 1000.0f);
        v815d_diag.baro_innovation_mm =
            V815D_ScaleFloatToI32(eskf->baro_innovation_m, 1000.0f);
        v815d_diag.lidar_innovation_mm =
            V815D_ScaleFloatToI32(eskf->lidar_innovation_m, 1000.0f);

        v815d_diag.eskf_reset_count = eskf->reset_count;
        v815d_diag.eskf_gap_skip_count = eskf->gap_skip_count;
    }
    else
    {
        v815d_diag.origin_zero_count = 0UL;
        v815d_diag.eskf_numerical_error_count = 0UL;

        v815d_diag.position_x_mm = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.position_y_mm = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.position_z_mm = V815D_DIAG_FLOAT_INVALID;

        v815d_diag.velocity_x_mmps = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.velocity_y_mmps = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.velocity_z_mmps = V815D_DIAG_FLOAT_INVALID;

        v815d_diag.roll_mdeg = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.pitch_mdeg = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.yaw_mdeg = V815D_DIAG_FLOAT_INVALID;

        v815d_diag.baro_reference_mm = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.lidar_reference_mm = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.baro_innovation_mm = V815D_DIAG_FLOAT_INVALID;
        v815d_diag.lidar_innovation_mm = V815D_DIAG_FLOAT_INVALID;

        v815d_diag.eskf_reset_count = 0UL;
        v815d_diag.eskf_gap_skip_count = 0UL;
    }

    v815d_diag.sd_error_count = sd_logger_error_count;
    v815d_diag.sd_write_error_count = sd_logger_write_error_count;

    v815d_diag.sd_ring_address = sd_logger_ring_address;
    v815d_diag.sd_ring_end_address = sd_logger_ring_end_address;

    v815d_diag.bss_end_address =
        (uint32_t)(uintptr_t)&_ebss;
    v815d_diag.msp_address = msp_now;

    if (msp_now > v815d_diag.bss_end_address)
    {
        v815d_diag.main_sram_gap_bytes =
            msp_now - v815d_diag.bss_end_address;
    }
    else
    {
        v815d_diag.main_sram_gap_bytes = 0UL;
    }

    /*
     * Publish stable sequence last, then checksum.
     */
    v815d_diag.sequence_begin = stable_sequence;
    v815d_diag.sequence_end = stable_sequence;

    v815d_diag.checksum_xor = V815D_ComputeChecksum();
    v815d_diag.checksum_xor_inv = ~v815d_diag.checksum_xor;
}

volatile uint32_t v815_attitude_delta_100ms = 0UL;
volatile uint32_t v815_eskf_predict_delta_100ms = 0UL;
volatile uint32_t v815_eskf_public_delta_100ms = 0UL;
volatile uint32_t v815_eskf_covariance_delta_100ms = 0UL;
volatile uint32_t v815_eskf_correction_delta_100ms = 0UL;

volatile uint32_t v815_eskf_correction_task_counter = 0UL;
volatile uint32_t v815_eskf_correction_task_delta_100ms = 0UL;
volatile uint32_t v815_eskf_correction_task_last_exec_us = 0UL;
volatile uint32_t v815_eskf_correction_task_max_exec_us = 0UL;
volatile uint32_t v815_eskf_correction_task_overrun_count = 0UL;
volatile uint32_t v815_eskf_correction_task_deadline_miss_count = 0UL;

volatile uint32_t v815_eskf_predict_max_exec_us = 0UL;
volatile uint32_t v815_eskf_correction_max_exec_us = 0UL;
volatile uint32_t v815_eskf_numerical_error_count = 0UL;
volatile uint32_t v815_eskf_gap_skip_count = 0UL;
volatile uint32_t v815_eskf_reset_count = 0UL;

volatile float v815_eskf_position_x_m = 0.0f;
volatile float v815_eskf_position_y_m = 0.0f;
volatile float v815_eskf_position_z_m = 0.0f;
volatile float v815_eskf_velocity_x_mps = 0.0f;
volatile float v815_eskf_velocity_y_mps = 0.0f;
volatile float v815_eskf_velocity_z_mps = 0.0f;

volatile float v815_eskf_roll_deg = 0.0f;
volatile float v815_eskf_pitch_deg = 0.0f;
volatile float v815_eskf_yaw_deg = 0.0f;

volatile float v815_eskf_gyro_bias_x_dps = 0.0f;
volatile float v815_eskf_gyro_bias_y_dps = 0.0f;
volatile float v815_eskf_gyro_bias_z_dps = 0.0f;


/* V8.15A pointer/ABI corruption diagnostics. */
volatile uint32_t v815a_magic_pre = 0x815A15A1UL;
volatile uint32_t v815a_magic_post = 0x815A15A2UL;
volatile uint8_t v815a_canary_ok = 0U;
volatile uint8_t v815a_pointer_api_ok = 0U;
volatile uint8_t v815a_state_finite_ok = 0U;
volatile uint8_t v815a_shadow_core_ok = 0U;

volatile uint32_t v815a_sensor_struct_size = sizeof(SensorData_t);
volatile uint32_t v815a_attitude_struct_size = sizeof(AttitudeEstimatorData_t);
volatile uint32_t v815a_eskf_struct_size = sizeof(FullStateESKFData_t);

volatile uint8_t v87_sd_ok = 0U;
volatile uint8_t v87_sd_progress_ok = 0U;
volatile uint32_t v87_sd_frame_delta_100ms = 0UL;
volatile uint32_t v87_sd_bytes_delta_100ms = 0UL;


/* IMU */
volatile uint8_t v87_imu_connected = 0U;
volatile uint8_t v87_imu_device_id = 0U;
volatile uint8_t v87_imu_driver_sample_valid = 0U;
volatile uint8_t v87_imu_progress_ok = 0U;
volatile uint32_t v87_imu_sample_age_us = 0UL;

volatile uint32_t v87_imu_task_delta_100ms = 0UL;
volatile uint32_t v87_imu_task_last_exec_us = 0UL;
volatile uint32_t v87_imu_task_max_exec_us = 0UL;
volatile uint32_t v87_imu_task_overrun_count = 0UL;
volatile uint32_t v87_imu_task_deadline_miss_count = 0UL;

/* Barometer */
volatile uint8_t v87_baro_connected = 0U;
volatile uint8_t v87_baro_data_ready = 0U;
volatile uint8_t v87_baro_pressure_valid = 0U;
volatile uint8_t v87_baro_calibrated = 0U;
volatile uint8_t v87_baro_healthy = 0U;
volatile uint8_t v87_baro_progress_ok = 0U;

volatile float v87_baro_pressure_pa = 0.0f;
volatile float v87_baro_temperature_c = 0.0f;
volatile float v87_baro_filtered_altitude_m = 0.0f;

volatile uint32_t v87_baro_source_update_count = 0UL;
volatile uint32_t v87_baro_source_delta_100ms = 0UL;

volatile uint32_t v87_baro_task_delta_100ms = 0UL;
volatile uint32_t v87_baro_task_last_exec_us = 0UL;
volatile uint32_t v87_baro_task_max_exec_us = 0UL;
volatile uint32_t v87_baro_task_overrun_count = 0UL;
volatile uint32_t v87_baro_task_deadline_miss_count = 0UL;

/* LIDAR */
volatile uint8_t v87_lidar_initialized = 0U;
volatile uint8_t v87_lidar_connected = 0U;
volatile uint8_t v87_lidar_data_ready = 0U;
volatile uint8_t v87_lidar_distance_valid = 0U;
volatile uint8_t v87_lidar_progress_ok = 0U;

volatile uint16_t v87_lidar_distance_cm = 0U;
volatile float v87_lidar_distance_m = 0.0f;
volatile float v87_lidar_median_distance_m = 0.0f;
volatile float v87_lidar_filtered_distance_m = 0.0f;

volatile uint8_t v87_lidar_calibration_complete = 0U;
volatile uint16_t v87_lidar_calibration_sample_count = 0U;

volatile uint32_t v87_lidar_update_count = 0UL;
volatile uint32_t v87_lidar_update_delta_100ms = 0UL;
volatile uint32_t v87_lidar_read_count = 0UL;
volatile uint32_t v87_lidar_error_count = 0UL;
volatile uint32_t v87_lidar_timeout_count = 0UL;

volatile uint32_t v87_lidar_dma_tx_start_count = 0UL;
volatile uint32_t v87_lidar_dma_tx_complete_count = 0UL;
volatile uint32_t v87_lidar_dma_rx_start_count = 0UL;
volatile uint32_t v87_lidar_dma_rx_complete_count = 0UL;
volatile uint32_t v87_lidar_dma_error_count = 0UL;
volatile uint32_t v87_lidar_dma_busy_count = 0UL;

volatile uint32_t v87_lidar_last_sample_interval_us = 0UL;
volatile uint32_t v87_lidar_min_sample_interval_us = 0UL;
volatile uint32_t v87_lidar_max_sample_interval_us = 0UL;
volatile uint32_t v87_lidar_sample_age_us = 0UL;

volatile uint32_t v87_lidar_probe_attempt_count = 0UL;
volatile uint32_t v87_lidar_probe_success_count = 0UL;
volatile uint32_t v87_lidar_probe_failure_count = 0UL;
volatile uint32_t v87_lidar_bus_recovery_count = 0UL;
volatile uint32_t v87_lidar_last_i2c_error_code = 0UL;

volatile uint32_t v87_lidar_task_delta_100ms = 0UL;
volatile uint32_t v87_lidar_task_last_exec_us = 0UL;
volatile uint32_t v87_lidar_task_max_exec_us = 0UL;
volatile uint32_t v87_lidar_task_overrun_count = 0UL;
volatile uint32_t v87_lidar_task_deadline_miss_count = 0UL;

/* Needle valve */
volatile uint32_t v87_valve_tick_delta_100ms = 0UL;
volatile uint32_t v87_adc_timeout_count = 0UL;
volatile uint8_t v87_valve_fault_snapshot = 0U;

/* -------------------------------------------------------------------------- */
/* Private monitor state                                                      */
/* -------------------------------------------------------------------------- */

static uint32_t last_valve_tick = 0UL;

static uint32_t last_imu_task_run_count = 0UL;
static uint32_t last_baro_task_run_count = 0UL;
static uint32_t last_lidar_task_run_count = 0UL;

static uint32_t last_imu_sample_timestamp_us = 0UL;
static uint32_t last_baro_source_update_count = 0UL;
static uint32_t last_lidar_update_count = 0UL;

static uint32_t last_sd_frame_count = 0UL;
static uint32_t last_sd_total_bytes = 0UL;
static uint32_t last_sd_timer_irq_count = 0UL;

static uint32_t last_nrf_task_run_count = 0UL;

static uint32_t v814_last_lidar_update_count_1s = 0UL;
static uint32_t v814_last_lidar_rate_ms = 0UL;


static uint32_t v815_last_attitude_update_count = 0UL;
static uint32_t v815_last_eskf_predict_count = 0UL;
static uint32_t v815_last_eskf_public_count = 0UL;
static uint32_t v815_last_eskf_covariance_count = 0UL;
static uint32_t v815_last_eskf_correction_count = 0UL;
static uint32_t v815_last_eskf_task_run_count = 0UL;

/* -------------------------------------------------------------------------- */

static uint32_t V87_SampleAgeUs(uint32_t now_us, uint32_t timestamp_us)
{
    if (timestamp_us == 0UL)
    {
        return 0xFFFFFFFFUL;
    }

    if ((uint32_t)(now_us - timestamp_us) >= 1000000UL)
    {
        return 0xFFFFFFFFUL;
    }

    return (uint32_t)(now_us - timestamp_us);
}

/* -------------------------------------------------------------------------- */

void AppTasks_Init(void)
{
    SensorManager_Init();

    IMU_Init();

    /*
     * Keep the known-working small-board LiDAR startup sequence:
     * I2C2/LiDAR is initialized before the independent SPI2 barometer.
     * No LiDAR, I2C2 or DMA driver code is changed.
     */
    Lidar_Init();
    Barometer_Init();
    SensorQualification_Init();

    /*
     * V50 keeps every estimator and sensor-qualification path online.
     * Full-State ESKF feeds the physical four-relay RCS path and the generated
     * vertical controller. P112R5 owns authorized physical needle actuation.
     */
    AttitudeEstimator_Init();
    FullStateESKF_Init();
    GeneratedFlightControl_Init();
    TaragayFlightLogic_Init();
    GNCActiveControl_Init();

    /* Legacy GNCActiveControl remains disabled. GeneratedFlightControl owns
     * the P112R5 needle command; Full-State ESKF signed-PD owns RCS relays. */
    AttitudeControl_Init(HAL_GetTick());
    {
        FullStateESKFData_t eskf_init = FullStateESKF_GetData();
        v49_last_eskf_public_output_count = eskf_init.public_output_count;
        v49_last_eskf_reset_count = eskf_init.reset_count;
    }

    /*
     * RemoteControl_Init() is intentionally performed earlier in App_Init(),
     * matching the known-working small-board nRF/servo project.
     */

    v87_imu_task_counter = 0UL;
    v87_baro_task_counter = 0UL;
    v87_lidar_task_counter = 0UL;
    v87_monitor_task_counter = 0UL;

    v87_scheduler_ok = 0U;
    v87_imu_ok = 0U;
    v87_baro_ok = 0U;
    v87_lidar_ok = 0U;
    v87_valve_tick_ok = 0U;
    v87_adc_ok = 0U;
    v87_integration_ok = 0U;

    v89_scheduler_ok = 0U;
    v89_nrf_hw_ok = 0U;
    v89_integration_ok = 0U;

    v89_nrf_initialized = 0U;
    v89_nrf_connected = 0U;
    v89_nrf_mode = 0U;
    v89_nrf_status_reg = 0U;
    v89_nrf_config_reg = 0U;
    v89_nrf_rf_ch_reg = 0U;
    v89_nrf_rf_setup_reg = 0U;
    v89_nrf_fifo_status_reg = 0U;

    v89_remote_link_active = 0U;
    v89_remote_command = 0U;
    v89_remote_last_sequence = 0U;

    v89_nrf_rx_count = 0UL;
    v89_nrf_tx_count = 0UL;
    v89_nrf_tx_fail_count = 0UL;
    v89_nrf_error_count = 0UL;

    v89_remote_valid_packet_count = 0UL;
    v89_remote_invalid_packet_count = 0UL;
    v89_remote_timeout_count = 0UL;
    v89_remote_irq_count = 0UL;
    v89_remote_last_packet_age_ms = 0xFFFFFFFFUL;

    v89_nrf_task_counter = 0UL;
    v89_nrf_task_delta_100ms = 0UL;
    v89_nrf_task_last_exec_us = 0UL;
    v89_nrf_task_max_exec_us = 0UL;
    v89_nrf_task_overrun_count = 0UL;
    v89_nrf_task_deadline_miss_count = 0UL;

    v89_spi3_ok = 0U;
    v89_spi3_fault = 0U;
    v89_spi3_last_error = 0U;
    v89_spi3_transaction_count = 0UL;
    v89_spi3_error_count = 0UL;
    v89_spi3_timeout_count = 0UL;
    v89_spi3_busy_count = 0UL;
    v89_spi3_slow_count = 0UL;
    v89_spi3_last_duration_us = 0UL;
    v89_spi3_max_duration_us = 0UL;


    v810_servo_ok = 0U;
    v810_integration_ok = 0U;

    v810_servo_init_ok = vent_servo_init_ok;
    v810_servo_remote_command = 0U;
    v810_servo_link_active = 0U;
    v810_servo_target_open = 0U;
    v810_servo_pulse_us = vent_servo_pulse_us;

    v810_servo_update_count = 0UL;
    v810_servo_command_change_count = 0UL;


    v811_lidar_sample_rate_ok = 0U;
    v811_lidar_sample_delta_100ms = 0UL;
    v811_integration_ok = 0U;

    v813_lidar_profile_config_ok = 0U;
    v813_lidar_200hz_ok = 0U;
    v813_lidar_sample_delta_100ms = 0UL;
    v813_lidar_sample_rate_hz = 0.0f;
    v813_integration_ok = 0U;
    v813_core_without_baro_ok = 0U;
    v813_sd_accounting_ok = 0U;

    v814_lidar_sample_delta_1s = 0UL;
    v814_lidar_sample_rate_1s_hz = 0.0f;
    v814_lidar_sample_rate_1s_min_hz = 0.0f;
    v814_lidar_sample_rate_1s_max_hz = 0.0f;
    v814_lidar_low_rate_window_count = 0UL;
    v814_lidar_rate_1s_ok = 0U;
    v814_integration_ok = 0U;


    v815_attitude_ok = 0U;
    v815_eskf_initialized = 0U;
    v815_eskf_healthy = 0U;
    v815_eskf_shadow_mode = 0U;
    v815_eskf_lidar_reference_ready = 0U;
    v815_eskf_baro_reference_ready = 0U;
    v815_eskf_stationary_detected = 0U;

    v815_eskf_rate_ok = 0U;
    v815_eskf_shadow_core_ok = 0U;
    v815_integration_ok = 0U;

    v815c_firmware_identity_ok = 0U;
    v815c_ram_magic_ok = 0U;
    v815c_sd_ring_in_ccm = 0U;
    v815c_estimator_basic_sanity_ok = 0U;

    v815c_bss_end_address = (uint32_t)(uintptr_t)&_ebss;
    v815c_msp_address = 0UL;
    v815c_main_sram_gap_bytes = 0UL;
    v815c_main_sram_gap_min_bytes = 0xFFFFFFFFUL;

    v815_attitude_delta_100ms = 0UL;
    v815_eskf_predict_delta_100ms = 0UL;
    v815_eskf_public_delta_100ms = 0UL;
    v815_eskf_covariance_delta_100ms = 0UL;
    v815_eskf_correction_delta_100ms = 0UL;

    v815_eskf_correction_task_counter = 0UL;
    v815_eskf_correction_task_delta_100ms = 0UL;
    v815_eskf_correction_task_last_exec_us = 0UL;
    v815_eskf_correction_task_max_exec_us = 0UL;
    v815_eskf_correction_task_overrun_count = 0UL;
    v815_eskf_correction_task_deadline_miss_count = 0UL;

    v815_eskf_predict_max_exec_us = 0UL;
    v815_eskf_correction_max_exec_us = 0UL;
    v815_eskf_numerical_error_count = 0UL;
    v815_eskf_gap_skip_count = 0UL;
    v815_eskf_reset_count = 0UL;

    v815a_canary_ok = 0U;
    v815a_pointer_api_ok = 0U;
    v815a_state_finite_ok = 0U;
    v815a_shadow_core_ok = 0U;

    {
        LidarData_t lidar_init_data = Lidar_GetData();
        v814_last_lidar_update_count_1s = lidar_init_data.update_count;
    }
    v814_last_lidar_rate_ms = millis();

    v87_sd_ok = 0U;
    v87_sd_progress_ok = 0U;
    v87_sd_frame_delta_100ms = 0UL;
    v87_sd_bytes_delta_100ms = 0UL;

    v87_imu_connected = IMU_IsConnected();
    v87_imu_device_id = IMU_GetDeviceID();
    v87_imu_driver_sample_valid = IMU_IsLastSampleValid();
    v87_imu_progress_ok = 0U;

    {
        BarometerData_t baro = Barometer_GetData();
        last_baro_source_update_count = baro.source_update_count;
    }

    {
        LidarData_t lidar = Lidar_GetData();
        last_lidar_update_count = lidar.update_count;
    }

    last_valve_tick = needle_valve_control_tick_count;
    last_imu_task_run_count = 0UL;
    last_baro_task_run_count = 0UL;
    last_lidar_task_run_count = 0UL;
    last_imu_sample_timestamp_us = IMU_GetLastSampleTimestampUs();

    last_sd_frame_count = sd_logger_frame_count;
    last_sd_total_bytes = sd_logger_total_bytes_written;
    last_sd_timer_irq_count = sd_logger_timer_irq_count;

    v87_adc_timeout_count = needle_valve_hw_adc_timeout_count;
    v87_valve_fault_snapshot = needle_valve_fault;

    {
        const AttitudeEstimatorData_t *attitude_init =
            AttitudeEstimator_GetDataPtr();
        const FullStateESKFData_t *eskf_init =
            FullStateESKF_GetDataPtr();

        v815_last_attitude_update_count = attitude_init->update_count;
        v815_last_eskf_predict_count = eskf_init->predict_count;
        v815_last_eskf_public_count = eskf_init->public_output_count;
        v815_last_eskf_covariance_count =
            eskf_init->covariance_predict_count;
        v815_last_eskf_correction_count = 0UL;
        v815_last_eskf_task_run_count = 0UL;
    }
}

/* -------------------------------------------------------------------------- */
/* 1 kHz IMU                                                                  */
/* -------------------------------------------------------------------------- */

void Task_IMU_1kHz(void)
{
    SensorManager_UpdateIMU();

    /*
     * Exact estimator ordering for every new IMU sample:
     * 1) attitude bootstrap / relative attitude
     * 2) full-state nominal ESKF propagation
     * 3) push only the raw IMU sample into the lightweight SD fast-history
     */
    AttitudeEstimator_Update();
    FullStateESKF_Predict();

    /* V49: 1 kHz RCS fail-safe service.  The actual signed-PD command is
     * generated only from the fresh 200 Hz Full-State ESKF output below. */
#if (APP_RELAY_SEQUENCE_TEST_MODE != 0U)
    v23_rcs_requested_mask = relay_bench_test_last_mask;
    v23_rcs_applied_mask = relay_bench_test_last_mask;
#elif (APP_V49_ESKF_RCS_PHYSICAL_ENABLED != 0U)
    if ((App_IsActuatorAuthorized() != 0U) &&
        (v30_vent_override_active == 0U))
    {
        const SensorData_t *sensor = SensorManager_GetDataPtr();
        const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
        uint8_t estimator_healthy =
            ((eskf != 0) &&
             (sensor != 0) &&
             (eskf->enabled != 0U) &&
             (eskf->initialized != 0U) &&
             (eskf->healthy != 0U) &&
             (eskf->origin_zeroed != 0U) &&
             (eskf->output_inhibited == 0U) &&
             (sensor->imu_valid != 0U) &&
             (SystemMonitor_IsActuatorFaultActive() == 0U)) ? 1U : 0U;
        uint32_t now_ms = HAL_GetTick();
        SolenoidOutputStatus_t solenoid;
        AttitudeControlStatus_t ctrl;

        v23_rcs_estimator_healthy = estimator_healthy;

        if ((eskf != 0) &&
            (eskf->reset_count != v49_last_eskf_reset_count))
        {
            v49_last_eskf_reset_count = eskf->reset_count;
            v49_last_eskf_public_output_count = eskf->public_output_count;
            AttitudeControl_NotifyEstimatorReset(now_ms);
        }

        AttitudeControl_ServiceESKF(estimator_healthy, now_ms);

        solenoid = SolenoidOutput_GetStatus();
        ctrl = AttitudeControl_GetStatus();
        v23_rcs_requested_mask = solenoid.requested_mask;
        v23_rcs_applied_mask = solenoid.applied_mask;
        v23_rcs_fault = (uint8_t)ctrl.fault;
        v23_rcs_control_update_count = ctrl.control_update_count;
        v23_rcs_output_update_count = solenoid.update_count;
    }
    else if (v30_vent_override_active == 0U)
    {
        AttitudeControl_ForceSafe();
        v23_rcs_requested_mask = 0U;
        v23_rcs_applied_mask = 0U;
    }
    else
    {
        SolenoidOutputStatus_t vent_output = SolenoidOutput_GetStatus();
        v23_rcs_requested_mask = vent_output.requested_mask;
        v23_rcs_applied_mask = vent_output.applied_mask;
    }
#else
#if (APP_P112R12R8R33_REAL_FLIGHT_LOGIC_PHYSICAL_RCS_REV != 0U)
    /* R8R33: the 1 kHz legacy V49 service must not erase the V7.13.4 mask.
     * It acts only as an independent fast safety cut. Normal mask timing is
     * owned by TaragayFlightLogic at the control-service rate. */
    /* Manual ground-vent owns the four relay GPIOs while its override is
     * active. Do not let the 1 kHz flight-RCS safety service erase a valid
     * vent pulse. All normal RCS safety cuts remain unchanged whenever the
     * vent override is not active. */
    if (v30_vent_override_active == 0U)
    {
        if ((PreflightTrigger_IsFlightActive() == 0U) ||
            (PreflightTrigger_HasFault() != 0U) ||
            (App_IsActuatorAuthorized() == 0U) ||
            (App_IsStopLatched() != 0U) ||
            (SystemMonitor_IsActuatorFaultActive() != 0U))
        {
            SolenoidOutput_ForceSafe();
        }
    }
    {
        const SolenoidOutputStatus_t r8r33_output = SolenoidOutput_GetStatus();
        v23_rcs_requested_mask = r8r33_output.requested_mask;
        v23_rcs_applied_mask = r8r33_output.applied_mask;
    }
#else
    if (v30_vent_override_active == 0U)
    {
        AttitudeControl_ForceSafe();
    }
#endif
#endif

#if (APP_SDLOGGER_ENABLED != 0U)
    SDLogger_PushFastIMU();
#endif

    v87_imu_task_counter++;

    v87_imu_connected = IMU_IsConnected();
    v87_imu_device_id = IMU_GetDeviceID();
    v87_imu_driver_sample_valid = IMU_IsLastSampleValid();
}

/* -------------------------------------------------------------------------- */
/* 200 Hz BMP585 + needle health                                              */
/* -------------------------------------------------------------------------- */

void Task_BarometerValveHealth_200Hz(void)
{
    BarometerData_t baro;
    const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
    const LidarData_t *lidar = Lidar_GetDataPtr();
    const PreflightTriggerStatus_t preflight = PreflightTrigger_GetStatus();
    uint8_t allow_ground_tracking = 0U;

    if ((preflight.debounced_open != 0U) ||
        (preflight.flight_active != 0U))
    {
        /* One-way latch: freeze at confirmed connector separation, before any
         * possible motion can be interpreted as a new ground datum. */
        Barometer_FreezeGroundReference();
    }
    else if ((eskf != 0) &&
             (lidar != 0) &&
             (eskf->initialized != 0U) &&
             (eskf->healthy != 0U) &&
             (eskf->stationary_detected != 0U) &&
             (fabsf(eskf->velocity_z_mps) <=
              APP_BARO_GROUND_TRACK_MAX_VZ_MPS) &&
             (lidar->distance_valid != 0U) &&
             (lidar->last_sample_timestamp_us != 0UL) &&
             ((uint32_t)(micros() - lidar->last_sample_timestamp_us) <=
              APP_BARO_GROUND_TRACK_LIDAR_MAX_AGE_US))
    {
        allow_ground_tracking = 1U;
    }

    Barometer_SetGroundReferenceTrackingAllowed(allow_ground_tracking);
    SensorManager_UpdateBarometer();
    baro = Barometer_GetData();

    v87_baro_task_counter++;

    v87_baro_connected = baro.connected;
    v87_baro_data_ready = baro.data_ready;
    v87_baro_pressure_valid = baro.pressure_valid;
    v87_baro_calibrated = baro.calibrated;
    v87_baro_healthy = baro.healthy;

    v87_baro_pressure_pa = baro.pressure_pa;
    v87_baro_temperature_c = baro.temperature_c;
    v87_baro_filtered_altitude_m = baro.filtered_altitude_m;
    v87_baro_source_update_count = baro.source_update_count;

    v87_adc_timeout_count = needle_valve_hw_adc_timeout_count;
    v87_valve_fault_snapshot = needle_valve_fault;
}

/* -------------------------------------------------------------------------- */
/* 1 kHz LIDAR state-machine service                                                       */
/* -------------------------------------------------------------------------- */

void Task_Lidar_Service_1kHz(void)
{
    LidarData_t lidar;

    /*
     * This driver is a DMA-backed non-blocking state machine.
     * The normal steady-state call is short.
     *
     * At startup/reconnect, HAL_I2C_IsDeviceReady may be slower; this is
     * deliberately measured via task max/overrun counters.
     */
    Lidar_Update();
    lidar = Lidar_GetData();

    v87_lidar_task_counter++;

    v87_lidar_initialized = lidar.initialized;
    v87_lidar_connected = lidar.connected;
    v87_lidar_data_ready = lidar.data_ready;
    v87_lidar_distance_valid = lidar.distance_valid;

    v87_lidar_distance_cm = lidar.distance_cm;
    v87_lidar_distance_m = lidar.distance_m;
    v87_lidar_median_distance_m = lidar.median_distance_m;
    v87_lidar_filtered_distance_m = lidar.filtered_distance_m;

    v87_lidar_update_count = lidar.update_count;
    v87_lidar_read_count = lidar.read_count;
    v87_lidar_error_count = lidar.error_count;
    v87_lidar_timeout_count = lidar.timeout_count;

    v87_lidar_dma_tx_start_count = lidar.dma_tx_start_count;
    v87_lidar_dma_tx_complete_count = lidar.dma_tx_complete_count;
    v87_lidar_dma_rx_start_count = lidar.dma_rx_start_count;
    v87_lidar_dma_rx_complete_count = lidar.dma_rx_complete_count;
    v87_lidar_dma_busy_count = lidar.dma_busy_count;
    v87_lidar_dma_error_count = lidar.dma_error_count;

    v87_lidar_last_sample_interval_us = lidar.last_sample_interval_us;
    v87_lidar_min_sample_interval_us = lidar.min_sample_interval_us;
    v87_lidar_max_sample_interval_us = lidar.max_sample_interval_us;

    v87_lidar_calibration_complete = lidar_calibration_complete;
    v87_lidar_calibration_sample_count = lidar_calibration_sample_count;

    v87_lidar_probe_attempt_count = lidar_probe_attempt_count;
    v87_lidar_probe_success_count = lidar_probe_success_count;
    v87_lidar_probe_failure_count = lidar_probe_failure_count;
    v87_lidar_bus_recovery_count = lidar_bus_recovery_count;
    v87_lidar_last_i2c_error_code = lidar_last_i2c_error_code;
}

/* -------------------------------------------------------------------------- */
/* 200 Hz NRF24 / RemoteControl + tahliye servo service                      */
/* -------------------------------------------------------------------------- */

void Task_NRFMonitor_200Hz(void)
{
#if (APP_NRF24_ENABLED != 0U)
    NRF24_Data_t nrf;

    /* V23: RemoteControl_Update + ServoOutput_Update are serviced continuously
     * in App_Run, matching the known-working small-board project. This 200 Hz
     * task now only snapshots diagnostics. */

    nrf = NRF24_GetData();

    v89_nrf_task_counter++;

    v89_nrf_initialized = nrf.initialized;
    v89_nrf_connected = nrf.connected;
    v89_nrf_mode = (uint8_t)nrf.mode;

    v89_nrf_status_reg = nrf.status_reg;
    v89_nrf_config_reg = nrf.config_reg;
    v89_nrf_rf_ch_reg = nrf.rf_ch_reg;
    v89_nrf_rf_setup_reg = nrf.rf_setup_reg;
    v89_nrf_fifo_status_reg = nrf.fifo_status_reg;

    v89_nrf_rx_count = nrf.rx_count;
    v89_nrf_tx_count = nrf.tx_count;
    v89_nrf_tx_fail_count = nrf.tx_fail_count;
    v89_nrf_error_count = nrf.error_count;

    v89_remote_link_active = RemoteControl_IsLinkActive();
    v89_remote_command = RemoteControl_GetCommand();
    v89_remote_last_sequence = remote_rx_last_sequence;

    v89_remote_valid_packet_count = remote_rx_valid_packet_count;
    v89_remote_invalid_packet_count = remote_rx_invalid_packet_count;
    v89_remote_timeout_count = remote_rx_timeout_count;
    v89_remote_irq_count = remote_rx_irq_count;
    v89_remote_last_packet_age_ms = remote_rx_last_packet_age_ms;

    v810_servo_init_ok = vent_servo_init_ok;
    v810_servo_remote_command = vent_servo_remote_command;
    v810_servo_link_active = vent_servo_link_active;
    v810_servo_target_open = vent_servo_target_open;
    v810_servo_pulse_us = vent_servo_pulse_us;
    v810_servo_update_count = vent_servo_update_count;
    v810_servo_command_change_count = vent_servo_command_change_count;

#else
    v89_nrf_task_counter++;
#endif
}

/* -------------------------------------------------------------------------- */
/* 200 Hz Full-State ESKF correction service                                  */
/* -------------------------------------------------------------------------- */

#if ((APP_P112R10R3_GNC_MOTOR_BENCH_MODE != 0U) && \
     (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE == 0U))
static uint8_t p112r10_stimulus_started = 0U;
static uint8_t p112r10_stimulus_done = 0U;
static uint32_t p112r10_stimulus_start_ms = 0UL;
#endif

#if (APP_P112R12R8_GNC_FRESHNESS_GUARD_ENABLED != 0U)
/* Production GNC command-age guard. The state is reset whenever actuator
 * authorization/readiness is lost, so re-authorization requires a newly
 * observed GeneratedFlightControl step before a nonzero command can pass. */
static uint32_t v55_gnc_last_step_count = 0UL;
static uint32_t v55_gnc_last_step_ms = 0UL;
static uint8_t v55_gnc_step_seen = 0U;
#endif

static void V55_ApplyPhysicalNeedleOutput(void)
{
#if (APP_NEEDLE_AUTONOMOUS_ACTUATOR_MODE != 0U)
#if (APP_NEEDLE_P112R11_MECH_COMMISSION_MODE != 0U)
    /* P112R11 DO-NOT-FLY mechanical commissioning.  The compile-time PA0
     * sequencer below is the sole needle command producer in this build.
     * PE9 must remain connected. UART is still TX-only. */
    NeedleValveAutonomousControl_CommissioningUpdate();
#else
    GeneratedFlightControlStatus_t main_control =
        GeneratedFlightControl_GetStatus();
    float command = 0.0f;

    /* R8R15: a latched E-STOP revokes every normal actuator command, but it
     * owns one deliberate exception: close the needle once to the learned
     * CLOSED reference. Never let the normal authorization branch revoke that
     * safe-close while it is in progress. */
    if (App_IsStopLatched() != 0U)
    {
        NeedleValveAutonomousControl_EStopSafeCloseUpdate();
        return;
    }

    if ((App_IsActuatorAuthorized() == 0U) ||
        (NeedleValveAutonomousControl_IsReady() == 0U))
    {
#if (APP_P112R12R8_GNC_FRESHNESS_GUARD_ENABLED != 0U)
        v55_gnc_step_seen = 0U;
        v55_gnc_last_step_count = main_control.step_count;
        v55_gnc_last_step_ms = HAL_GetTick();
#endif
        NeedleValveAutonomousControl_RevokeAuthorization();
        return;
    }

#if (APP_P112R12R8_GNC_FRESHNESS_GUARD_ENABLED != 0U)
    {
        const uint32_t now_ms = HAL_GetTick();
        if ((v55_gnc_step_seen == 0U) ||
            (main_control.step_count != v55_gnc_last_step_count))
        {
            v55_gnc_last_step_count = main_control.step_count;
            v55_gnc_last_step_ms = now_ms;
            v55_gnc_step_seen = 1U;
        }
    }
#endif

    if ((main_control.main_output_valid != 0U) &&
        (main_control.vertical_sensor_source_mask != 0U) &&
        (main_control.vertical_200hz_valve_cmd ==
         main_control.vertical_200hz_valve_cmd)
#if (APP_P112R12R8_GNC_FRESHNESS_GUARD_ENABLED != 0U)
        && (v55_gnc_step_seen != 0U)
        && ((uint32_t)(HAL_GetTick() - v55_gnc_last_step_ms) <=
            APP_P112R12R8_GNC_MAX_AGE_MS)
#endif
        )
    {
        command = main_control.vertical_200hz_valve_cmd;
        if (command < 0.0f) command = 0.0f;
        if (command > 1.0f) command = 1.0f;
    }

#if (APP_P112R10R3_GNC_MOTOR_BENCH_MODE != 0U)
    if (p112r10_stimulus_started == 0U)
    {
        p112r10_stimulus_started = 1U;
        p112r10_stimulus_start_ms = HAL_GetTick();
    }

    if (p112r10_stimulus_done == 0U)
    {
        uint32_t stimulus_elapsed_ms =
            (uint32_t)(HAL_GetTick() - p112r10_stimulus_start_ms);

        if (stimulus_elapsed_ms < APP_P112R10R3_STIMULUS_HOLD_CLOSED_MS)
            command = 0.0f;
        else if (stimulus_elapsed_ms <
                 (APP_P112R10R3_STIMULUS_HOLD_CLOSED_MS +
                  APP_P112R10R3_STIMULUS_ACTIVE_MS))
        {
            if (command > APP_P112R10R3_STIMULUS_COMMAND_CAP)
                command = APP_P112R10R3_STIMULUS_COMMAND_CAP;
        }
        else
        {
            command = 0.0f;
            p112r10_stimulus_done = 1U;
        }
    }
    else command = 0.0f;
#endif

    if (NeedleValveAutonomousControl_SubmitCommand(command) == 0U)
        NeedleValveAutonomousControl_RevokeAuthorization();
#endif
#elif (APP_NEEDLE_FOUR_TURN_TEST_MODE != 0U)
    return;
#elif (APP_GENERATED_FC_PHYSICAL_OUTPUT_ENABLED != 0U)
    GeneratedFlightControlStatus_t main_control =
        GeneratedFlightControl_GetStatus();
    NeedleValveStatus_t needle = NeedleValveController_GetStatus();
    float command = 0.0f;

    if (App_IsActuatorAuthorized() == 0U)
    {
        NeedleValveController_Stop();
        return;
    }

    if ((needle.zero_valid == 0U) ||
        (needle.fault != NEEDLE_VALVE_FAULT_NONE))
    {
        NeedleValveController_Stop();
        return;
    }

    if (needle.enabled == 0U)
    {
        if (NeedleValveController_Enable() == 0U)
        {
            NeedleValveController_Stop();
            return;
        }
    }

    /* Loss of a valid main-control result actively requests CLOSED. The
     * controller remains enabled long enough to return the valve to ZERO. */
    if ((main_control.main_output_valid != 0U) &&
        (main_control.vertical_sensor_source_mask != 0U) &&
        (main_control.vertical_200hz_valve_cmd ==
         main_control.vertical_200hz_valve_cmd))
    {
        command = main_control.vertical_200hz_valve_cmd;
        if (command < 0.0f) command = 0.0f;
        if (command > 1.0f) command = 1.0f;
    }

    if (NeedleValveController_SetCommand(command) == 0U)
    {
        NeedleValveController_Stop();
    }
#else
    NeedleValveController_Stop();
#endif
}

#if (APP_P112R12R8R32_REAL_FLIGHT_LOGIC_PHYSICAL_NEEDLE_REV != 0U)
/* R16 FINAL physical main-needle handoff.
 *
 * Mission authority begins ONLY after the debounced PE9 separation latch
 * (PreflightTrigger flight_active). Before PE9 the normal flight needle path
 * has zero motor authority and the R16 mission state machine is not advanced.
 *
 * TaragayFlightLogic outputs normalized valve/Cv command u=0..1. This is
 * converted with the validated nonlinear Cv->turn map and then normalized to
 * the qualified 3-turn P112 actuator command. */
static uint32_t r8r32_fl_last_step_count = 0UL;
static uint32_t r8r32_fl_last_step_ms = 0UL;
static uint8_t r8r32_fl_step_seen = 0U;

static float R16_ValveUToTurns(float u)
{
    static const float cv_bp[10] = {0.00f,0.30f,0.55f,0.92f,1.16f,1.45f,1.66f,1.78f,1.81f,1.82f};
    static const float tr_bp[10] = {0.10f,0.25f,0.50f,0.75f,1.00f,1.25f,1.50f,2.00f,2.50f,3.00f};
    float cv; uint32_t k;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    cv = u * 1.82f;
    if (cv <= cv_bp[0]) return tr_bp[0];
    if (cv >= cv_bp[9]) return tr_bp[9];
    for (k = 0U; k < 9U; k++)
    {
        if ((cv >= cv_bp[k]) && (cv <= cv_bp[k + 1U]))
        {
            float d = cv_bp[k + 1U] - cv_bp[k];
            float a = (d > 0.0f) ? ((cv - cv_bp[k]) / d) : 0.0f;
            return tr_bp[k] + a * (tr_bp[k + 1U] - tr_bp[k]);
        }
    }
    return tr_bp[9];
}

static void R8R32_ApplyTaragayPhysicalNeedleOutput(void)
{
    const TaragayFlightLogicStatus_t fl = TaragayFlightLogic_GetStatus();
    float command = 0.0f;

    if (App_IsStopLatched() != 0U)
    { NeedleValveAutonomousControl_EStopSafeCloseUpdate(); return; }

    /* Hard mission-start gate: no normal needle motion before PE9. */
    if ((PreflightTrigger_IsFlightActive() == 0U) ||
        (PreflightTrigger_HasFault() != 0U) ||
        (App_IsActuatorAuthorized() == 0U) ||
        (NeedleValveAutonomousControl_IsReady() == 0U))
    {
        r8r32_fl_step_seen = 0U;
        r8r32_fl_last_step_count = fl.step_count;
        r8r32_fl_last_step_ms = HAL_GetTick();
        NeedleValveAutonomousControl_RevokeAuthorization();
        return;
    }

    if ((r8r32_fl_step_seen == 0U) || (fl.step_count != r8r32_fl_last_step_count))
    {
        r8r32_fl_last_step_count = fl.step_count;
        r8r32_fl_last_step_ms = HAL_GetTick();
        r8r32_fl_step_seen = 1U;
    }

    if ((fl.real_input_active != 0U) && (fl.synthetic_input_active == 0U) &&
        (fl.input_valid != 0U) && (fl.mount_cal_valid != 0U) &&
        (fl.mount_cal_fault == 0U) && (fl.valve_cmd == fl.valve_cmd) &&
        (r8r32_fl_step_seen != 0U) &&
        ((uint32_t)(HAL_GetTick() - r8r32_fl_last_step_ms) <= APP_R8R32_FL_STEP_MAX_AGE_MS))
    {
        float turns = R16_ValveUToTurns(fl.valve_cmd);
        command = turns / APP_R16_NEEDLE_MAX_TURNS;
        if (command < 0.0f) command = 0.0f;
        if (command > 1.0f) command = 1.0f;
    }

    if (NeedleValveAutonomousControl_SubmitCommand(command) == 0U)
        NeedleValveAutonomousControl_RevokeAuthorization();
}
#endif

#if (APP_P112R12R8R33_REAL_FLIGHT_LOGIC_PHYSICAL_RCS_REV != 0U)
/* R8R33 physical RCS handoff.
 *
 * Sole command authority: TaragayFlightLogic / RCS V7.13.4.
 * The legacy V49/AttitudeControl path is held safe in this revision. A
 * nonzero mask is permitted only with the PE9 flight latch, real/fresh ESKF
 * input, frozen mount calibration, healthy system, no STOP and no RCS fault.
 * Any failed gate forces all four relay outputs safe in the same service pass. */
static uint32_t r8r33_fl_last_step_count = 0UL;
static uint32_t r8r33_fl_last_step_ms = 0UL;
static uint8_t r8r33_fl_step_seen = 0U;

static void R8R33_ApplyTaragayPhysicalRCSOutput(void)
{
    /* Switch-1 ground vent is a temporary, explicit owner of the same four
     * relay GPIOs. While it is active, the flight-RCS owner must not overwrite
     * the vent mask. Releasing/aborting vent returns ownership immediately. */
    if (v30_vent_override_active != 0U)
    {
        return;
    }

    const TaragayFlightLogicStatus_t fl = TaragayFlightLogic_GetStatus();
    const SystemStatus_t sys = SystemMonitor_GetStatus();
    uint8_t authorized = 0U;

    if ((r8r33_fl_step_seen == 0U) ||
        (fl.step_count != r8r33_fl_last_step_count))
    {
        r8r33_fl_last_step_count = fl.step_count;
        r8r33_fl_last_step_ms = HAL_GetTick();
        r8r33_fl_step_seen = 1U;
    }

    if ((PreflightTrigger_IsFlightActive() != 0U) &&
        (PreflightTrigger_HasFault() == 0U) &&
        (App_IsActuatorAuthorized() != 0U) &&
        (App_IsStopLatched() == 0U) &&
        (sys.system_ok != 0U) &&
        (fl.real_input_active != 0U) &&
        (fl.synthetic_input_active == 0U) &&
        (fl.input_valid != 0U) &&
        (fl.mount_cal_valid != 0U) &&
        (fl.mount_cal_fault == 0U) &&
        (fl.rcs_fault == 0U) &&
        (r8r33_fl_step_seen != 0U) &&
        ((uint32_t)(HAL_GetTick() - r8r33_fl_last_step_ms) <=
         APP_R8R33_RCS_STEP_MAX_AGE_MS))
    {
        authorized = 1U;
    }

    if (authorized != 0U)
        SolenoidOutput_SetMask(fl.rcs_requested_mask);
    else
        SolenoidOutput_ForceSafe();
}
#endif

void Task_FullESKFCorrection_200Hz(void)
{
    FullStateESKF_CorrectMeasurements();

    /* V49 RCS reads one and only one fresh ESKF public sample at 200 Hz.
     * TIM7 independently terminates the resulting 20..60 ms valve pulse. */
#if (APP_RELAY_SEQUENCE_TEST_MODE == 0U)
#if (APP_P112R12R8R33_REAL_FLIGHT_LOGIC_PHYSICAL_RCS_REV != 0U)
    /* R8R35: retire legacy V49 state without touching the physical mask.
     * R8R34 still called AttitudeControl_ForceSafe() here, which could cut a
     * V7.13.4 40 ms pulse at the next 200 Hz correction release. */
    AttitudeControl_ForceSafeNoOutput();
#else
    if ((v30_vent_override_active == 0U) &&
        (App_IsActuatorAuthorized() != 0U))
    {
        const FullStateESKFData_t *eskf = FullStateESKF_GetDataPtr();
        const SensorData_t *sensor = SensorManager_GetDataPtr();

        if ((eskf != 0) &&
            (eskf->output_inhibited == 0U) &&
            (eskf->public_output_count !=
             v49_last_eskf_public_output_count))
        {
            v49_last_eskf_public_output_count = eskf->public_output_count;
            AttitudeControl_UpdateESKF(eskf, sensor, HAL_GetTick());
        }
        else if ((eskf != 0) && (eskf->output_inhibited != 0U))
        {
            AttitudeControl_ForceSafe();
        }
    }
    else if (v30_vent_override_active == 0U)
    {
        AttitudeControl_ForceSafe();
    }
#endif /* R8R33 single RCS authority */
#endif

    /* V50 dual-aided vertical outer loop runs at this native 200 Hz rate.
     * LIDAR acquisition continues independently at all times. The outer loop
     * remains valid with either fresh LIDAR or fresh barometer ESKF aiding.
     * The archived Simulink model remains at its generated 100 Hz base rate;
     * V55 may route the result to the needle only through the PE9 flight gate. */
#if (APP_R8R19_FLIGHT_LOGIC_COMPUTE_ONLY != 0U)
#if (APP_P112R12R8R32_REAL_FLIGHT_LOGIC_PHYSICAL_NEEDLE_REV != 0U)
    /* R16 FINAL mission epoch = PE9 separation.
     * LOW PE9: connector installed -> controller held safe, no state advance.
     * HIGH/debounced PE9 with healthy preflight: flight_active latches and the
     * R16 mission starts from INIT. RCS and main-needle physical authority use
     * the same flight_active gate. */
    {
        static uint8_t r8r32_logic_started = 0U;
        const PreflightTriggerStatus_t pf = PreflightTrigger_GetStatus();

        if ((pf.flight_active != 0U) && (pf.fault_latched == 0U))
        {
            if (r8r32_logic_started == 0U)
            {
                TaragayFlightLogic_Reset();
                r8r32_logic_started = 1U;
            }
            /* Call immediately on the PE9 service pass. The internal 200->100
             * Hz divider preserves the deterministic 10 ms flight-control rate. */
            TaragayFlightLogic_Service200Hz();
        }
        else
        {
            r8r32_logic_started = 0U;
            TaragayFlightLogic_HoldSafe();
        }
    }
    /* Retired/legacy control paths are held safe. R8R33 preserves R8R32
     * needle authority and adds physical RCS authority only from RCS V7.13.4. */
    GeneratedFlightControl_HoldSafe();
    AttitudeControl_ForceSafeNoOutput();
    R8R32_ApplyTaragayPhysicalNeedleOutput();
#if (APP_P112R12R8R33_REAL_FLIGHT_LOGIC_PHYSICAL_RCS_REV != 0U)
    R8R33_ApplyTaragayPhysicalRCSOutput();
#else
    SolenoidOutput_ForceSafe();
#endif
#else
    /* R8R19: run the latest MATLAB-derived flight logic in compute-only mode.
     * Start the synthetic mission only after preflight is healthy for 1 s so
     * the UART observer can see ARM -> ASCENT -> HOVER -> DESCENT -> TOUCHDOWN.
     * Physical RCS stays isolated and the main needle is explicitly revoked. */
    {
        static uint8_t r8r19_logic_started = 0U;
        static uint8_t r8r19_ready_timer_active = 0U;
        static uint32_t r8r19_ready_since_ms = 0UL;
        const uint32_t now_ms = HAL_GetTick();
        const PreflightTriggerStatus_t preflight = PreflightTrigger_GetStatus();

        if (r8r19_logic_started == 0U)
        {
            if (preflight.preflight_ready != 0U)
            {
                if (r8r19_ready_timer_active == 0U)
                {
                    r8r19_ready_timer_active = 1U;
                    r8r19_ready_since_ms = now_ms;
                }
                else if ((uint32_t)(now_ms - r8r19_ready_since_ms) >= 1000UL)
                {
                    TaragayFlightLogic_Reset();
                    r8r19_logic_started = 1U;
                }
            }
            else
            {
                r8r19_ready_timer_active = 0U;
                TaragayFlightLogic_HoldSafe();
            }
        }
        else
        {
            TaragayFlightLogic_Service200Hz();
        }
    }
    GeneratedFlightControl_HoldSafe();
    AttitudeControl_ForceSafe();
    NeedleValveAutonomousControl_RevokeAuthorization();
#endif
#if ((APP_P112R12R8R25_RCS_RELAY_AXIS_BENCH_REV != 0U) || \
     (APP_P112R12R8R26_IMU_ROCKET_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R27_THREE_POSE_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R28_FIVE_POSE_FRAME_CAL_REV != 0U) || \
     (APP_P112R12R8R30_FIXED_IMU_ROCKET_FRAME_REV != 0U))
    /* R8R25..R8R30: the latest MATLAB-derived logical RCS request is the sole owner
     * of the four relay outputs for this pressureless bench. PE9 is deliberately
     * not required; every other physical non-needle path remains isolated. */
    {
        const TaragayFlightLogicStatus_t fl = TaragayFlightLogic_GetStatus();
        const PreflightTriggerStatus_t pf = PreflightTrigger_GetStatus();
        const SystemStatus_t sys = SystemMonitor_GetStatus();
        const uint8_t relay_bench_authorized =
            ((pf.preflight_ready != 0U) &&
             (fl.input_valid != 0U) &&
             (fl.real_input_active != 0U) &&
             (fl.synthetic_input_active == 0U) &&
             (fl.mount_cal_valid != 0U) &&
             (sys.system_ok != 0U) &&
             (App_IsStopLatched() == 0U)) ? 1U : 0U;

#if (APP_P112R12R8R33_REAL_FLIGHT_LOGIC_PHYSICAL_RCS_REV != 0U)
        /* R8R33 already applied the flight-authorized mask above. Do not let
         * the older PE9-bypassing relay-bench owner overwrite it. */
        (void)fl;
        (void)relay_bench_authorized;
#elif (APP_P112R12R8R32_REAL_FLIGHT_LOGIC_PHYSICAL_NEEDLE_REV != 0U)
        (void)fl;
        (void)relay_bench_authorized;
        SolenoidOutput_ForceSafe();
#else
        SolenoidOutput_SetAxisBenchMask(fl.rcs_requested_mask, relay_bench_authorized);
#endif
    }
#endif
#elif (APP_NEEDLE_FOUR_TURN_TEST_MODE != 0U)
    GeneratedFlightControl_HoldSafe();
#else
    if (App_IsActuatorAuthorized() != 0U)
    {
        GeneratedFlightControl_Service200Hz();
        V55_ApplyPhysicalNeedleOutput();
    }
    else
    {
        GeneratedFlightControl_HoldSafe();
        V55_ApplyPhysicalNeedleOutput();
    }
#endif

    /* Do NOT call disabled GNCActiveControl here; its disarmed state would
     * intentionally force the independently owned V49 RCS path safe. */

#if (APP_SDLOGGER_ENABLED != 0U)
    /* P32: one coherent source snapshot per native 200 Hz state solution. */
    SDLogger_PublishSources();
#endif

    v815_eskf_correction_task_counter++;
}

/* -------------------------------------------------------------------------- */
/* 25 Hz Full-State ESKF covariance propagation                               */
/* -------------------------------------------------------------------------- */

void Task_FullESKFCovariance_25Hz(void)
{
    uint32_t start_us = micros();
    uint8_t ran = FullStateESKF_ServiceCovariance();
    uint32_t exec_us = (uint32_t)(micros() - start_us);

    if (ran != 0U)
    {
        p34_covariance_last_us = exec_us;
        if (exec_us > p34_covariance_max_us)
        {
            p34_covariance_max_us = exec_us;
        }
        p34_covariance_service_count++;
    }
}

/* -------------------------------------------------------------------------- */
/* 10 Hz coexistence monitor                                                  */
/* -------------------------------------------------------------------------- */

void Task_IntegrationMonitor_10Hz(void)
{
    Task_t *imu_task;
    Task_t *baro_task;
    Task_t *lidar_task;
    Task_t *nrf_task;
    Task_t *eskf_task;

    uint32_t now_us;
    uint32_t imu_timestamp_us;
    uint32_t valve_tick_now;

    uint32_t imu_run_now = 0UL;
    uint32_t baro_run_now = 0UL;
    uint32_t lidar_run_now = 0UL;
    uint32_t nrf_run_now = 0UL;
    uint32_t eskf_run_now = 0UL;

    v87_monitor_task_counter++;

    now_us = micros();

    /* V8.18 sensor-only qualification, raw block @ 0x1000F200. */
    SensorQualification_Update10Hz();

    /* -------------------------- exact valve 200 Hz ------------------------ */

    valve_tick_now = needle_valve_control_tick_count;

    v87_valve_tick_delta_100ms =
        valve_tick_now - last_valve_tick;

    last_valve_tick = valve_tick_now;

    v87_valve_tick_ok =
        ((v87_valve_tick_delta_100ms >= 15UL) &&
         (v87_valve_tick_delta_100ms <= 25UL)) ? 1U : 0U;

    /* --------------------------- scheduler rates -------------------------- */

    imu_task = Scheduler_GetTask(0UL);
    baro_task = Scheduler_GetTask(1UL);
    lidar_task = Scheduler_GetTask(2UL);
    nrf_task = Scheduler_GetTask(3UL);
    eskf_task = Scheduler_GetTask(4UL);

    if (imu_task != 0)
    {
        imu_run_now = imu_task->run_count;

        v87_imu_task_delta_100ms =
            imu_run_now - last_imu_task_run_count;

        last_imu_task_run_count = imu_run_now;

        v87_imu_task_last_exec_us = imu_task->last_exec_us;
        v87_imu_task_max_exec_us = imu_task->max_exec_us;
        v87_imu_task_overrun_count = imu_task->overrun_count;
        v87_imu_task_deadline_miss_count = imu_task->deadline_miss_count;
    }

    if (baro_task != 0)
    {
        baro_run_now = baro_task->run_count;

        v87_baro_task_delta_100ms =
            baro_run_now - last_baro_task_run_count;

        last_baro_task_run_count = baro_run_now;

        v87_baro_task_last_exec_us = baro_task->last_exec_us;
        v87_baro_task_max_exec_us = baro_task->max_exec_us;
        v87_baro_task_overrun_count = baro_task->overrun_count;
        v87_baro_task_deadline_miss_count = baro_task->deadline_miss_count;
    }

    if (lidar_task != 0)
    {
        lidar_run_now = lidar_task->run_count;

        v87_lidar_task_delta_100ms =
            lidar_run_now - last_lidar_task_run_count;

        last_lidar_task_run_count = lidar_run_now;

        v87_lidar_task_last_exec_us = lidar_task->last_exec_us;
        v87_lidar_task_max_exec_us = lidar_task->max_exec_us;
        v87_lidar_task_overrun_count = lidar_task->overrun_count;
        v87_lidar_task_deadline_miss_count = lidar_task->deadline_miss_count;
    }

    if (nrf_task != 0)
    {
        nrf_run_now = nrf_task->run_count;

        v89_nrf_task_delta_100ms =
            nrf_run_now - last_nrf_task_run_count;

        last_nrf_task_run_count = nrf_run_now;

        v89_nrf_task_last_exec_us = nrf_task->last_exec_us;
        v89_nrf_task_max_exec_us = nrf_task->max_exec_us;
        v89_nrf_task_overrun_count = nrf_task->overrun_count;
        v89_nrf_task_deadline_miss_count = nrf_task->deadline_miss_count;
    }

    if (eskf_task != 0)
    {
        eskf_run_now = eskf_task->run_count;

        v815_eskf_correction_task_delta_100ms =
            eskf_run_now - v815_last_eskf_task_run_count;

        v815_last_eskf_task_run_count = eskf_run_now;

        v815_eskf_correction_task_last_exec_us = eskf_task->last_exec_us;
        v815_eskf_correction_task_max_exec_us = eskf_task->max_exec_us;
        v815_eskf_correction_task_overrun_count = eskf_task->overrun_count;
        v815_eskf_correction_task_deadline_miss_count =
            eskf_task->deadline_miss_count;
    }

    v87_scheduler_ok =
        ((v87_imu_task_delta_100ms >= 80UL) &&
         (v87_imu_task_delta_100ms <= 120UL) &&
         (v87_baro_task_delta_100ms >= 15UL) &&
         (v87_baro_task_delta_100ms <= 25UL) &&
         (v87_lidar_task_delta_100ms >= 15UL) &&
         (v87_lidar_task_delta_100ms <= 25UL)) ? 1U : 0U;

    v89_scheduler_ok =
        ((v87_scheduler_ok != 0U) &&
         (v89_nrf_task_delta_100ms >= 15UL) &&
         (v89_nrf_task_delta_100ms <= 25UL)) ? 1U : 0U;

    /* -------------------------------- IMU --------------------------------- */

    v87_imu_connected = IMU_IsConnected();
    v87_imu_device_id = IMU_GetDeviceID();
    v87_imu_driver_sample_valid = IMU_IsLastSampleValid();

    imu_timestamp_us = IMU_GetLastSampleTimestampUs();

    v87_imu_progress_ok =
        ((imu_timestamp_us != 0UL) &&
         (imu_timestamp_us != last_imu_sample_timestamp_us)) ? 1U : 0U;

    last_imu_sample_timestamp_us = imu_timestamp_us;

    v87_imu_sample_age_us =
        V87_SampleAgeUs(now_us, imu_timestamp_us);

    v87_imu_ok =
        ((v87_imu_connected != 0U) &&
         (v87_imu_driver_sample_valid != 0U) &&
         (v87_imu_progress_ok != 0U) &&
         (v87_imu_sample_age_us <= 20000UL)) ? 1U : 0U;

    /* ------------------------------ barometer ----------------------------- */

    {
        BarometerData_t baro = Barometer_GetData();

        v87_baro_connected = baro.connected;
        v87_baro_data_ready = baro.data_ready;
        v87_baro_pressure_valid = baro.pressure_valid;
        v87_baro_calibrated = baro.calibrated;
        v87_baro_healthy = baro.healthy;

        v87_baro_source_delta_100ms =
            baro.source_update_count - last_baro_source_update_count;

        v87_baro_progress_ok =
            (v87_baro_source_delta_100ms > 0UL) ? 1U : 0U;

        last_baro_source_update_count = baro.source_update_count;

        v87_baro_source_update_count = baro.source_update_count;
        v87_baro_pressure_pa = baro.pressure_pa;
        v87_baro_temperature_c = baro.temperature_c;
        v87_baro_filtered_altitude_m = baro.filtered_altitude_m;

        v87_baro_ok =
            ((baro.connected != 0U) &&
             (baro.data_ready != 0U) &&
             (baro.pressure_valid != 0U) &&
             (baro.calibrated != 0U) &&
             (baro.healthy != 0U) &&
             (v87_baro_progress_ok != 0U) &&
             (baro.pressure_pa >= APP_BARO_PRESSURE_MIN_PA) &&
             (baro.pressure_pa <= APP_BARO_PRESSURE_MAX_PA)) ? 1U : 0U;
    }

    /* -------------------------------- LIDAR ------------------------------- */

    {
        LidarData_t lidar = Lidar_GetData();

        v87_lidar_initialized = lidar.initialized;
        v87_lidar_connected = lidar.connected;
        v87_lidar_data_ready = lidar.data_ready;
        v87_lidar_distance_valid = lidar.distance_valid;

        v87_lidar_distance_cm = lidar.distance_cm;
        v87_lidar_distance_m = lidar.distance_m;
        v87_lidar_median_distance_m = lidar.median_distance_m;
        v87_lidar_filtered_distance_m = lidar.filtered_distance_m;

        v87_lidar_update_delta_100ms =
            lidar.update_count - last_lidar_update_count;

        v811_lidar_sample_delta_100ms =
            v87_lidar_update_delta_100ms;

        v813_lidar_sample_delta_100ms =
            v87_lidar_update_delta_100ms;
        v813_lidar_sample_rate_hz =
            (float)v813_lidar_sample_delta_100ms * 10.0f;
        v813_lidar_profile_config_ok = lidar_profile_config_ok;

        /*
         * V8.10 logs proved that a 200 Hz service task produced only 150 Hz
         * real samples because the 5 ms conversion wait was quantized by the
         * same 5 ms task. V8.11 services the state machine at 1 kHz and now
         * explicitly requires about 20 new samples per 100 ms.
         */
        v811_lidar_sample_rate_ok =
            ((v811_lidar_sample_delta_100ms >= 18UL) &&
             (v811_lidar_sample_delta_100ms <= 22UL)) ? 1U : 0U;

        v813_lidar_200hz_ok =
            v811_lidar_sample_rate_ok;

        /* Reliability first: exact 250 Hz is a performance target, not a
         * health requirement. Weak/long-range returns may finish slower. */
        v87_lidar_progress_ok =
            ((v87_lidar_update_delta_100ms > 0UL) &&
             (v813_lidar_profile_config_ok != 0U)) ? 1U : 0U;

        last_lidar_update_count = lidar.update_count;

        v87_lidar_update_count = lidar.update_count;
        v87_lidar_read_count = lidar.read_count;
        v87_lidar_error_count = lidar.error_count;
        v87_lidar_timeout_count = lidar.timeout_count;

        v87_lidar_dma_tx_start_count = lidar.dma_tx_start_count;
        v87_lidar_dma_tx_complete_count = lidar.dma_tx_complete_count;
        v87_lidar_dma_rx_start_count = lidar.dma_rx_start_count;
        v87_lidar_dma_rx_complete_count = lidar.dma_rx_complete_count;
        v87_lidar_dma_busy_count = lidar.dma_busy_count;
        v87_lidar_dma_error_count = lidar.dma_error_count;

        v87_lidar_last_sample_interval_us = lidar.last_sample_interval_us;
        v87_lidar_min_sample_interval_us = lidar.min_sample_interval_us;
        v87_lidar_max_sample_interval_us = lidar.max_sample_interval_us;

        v87_lidar_sample_age_us =
            V87_SampleAgeUs(now_us, lidar.last_sample_timestamp_us);

        v87_lidar_calibration_complete = lidar_calibration_complete;
        v87_lidar_calibration_sample_count = lidar_calibration_sample_count;

        v87_lidar_probe_attempt_count = lidar_probe_attempt_count;
        v87_lidar_probe_success_count = lidar_probe_success_count;
        v87_lidar_probe_failure_count = lidar_probe_failure_count;
        v87_lidar_bus_recovery_count = lidar_bus_recovery_count;
        v87_lidar_last_i2c_error_code = lidar_last_i2c_error_code;

        /*
         * Startup calibration is intentionally NOT required for v87_lidar_ok.
         * The configured calibration reference is 9.00 m. On a desk/bench,
         * calibration_complete may correctly remain 0 while real distance data
         * continues to be published unchanged.
         */
        v87_lidar_ok =
            ((lidar.initialized != 0U) &&
             (lidar.connected != 0U) &&
             (lidar.data_ready != 0U) &&
             (lidar.distance_valid != 0U) &&
             (v87_lidar_progress_ok != 0U) &&
             (v87_lidar_sample_age_us <= 30000UL) &&
             (lidar.distance_m >= 0.05f) &&
             (lidar.distance_m <= 40.0f)) ? 1U : 0U;
    }

    /* ------------------------- LIDAR 1-second rate ------------------------- */

    {
        uint32_t now_ms = millis();
        uint32_t elapsed_ms =
            (uint32_t)(now_ms - v814_last_lidar_rate_ms);

        if (elapsed_ms >= 1000UL)
        {
            LidarData_t lidar_1s = Lidar_GetData();
            uint32_t delta =
                lidar_1s.update_count - v814_last_lidar_update_count_1s;
            float rate_hz =
                ((float)delta * 1000.0f) / (float)elapsed_ms;

            v814_lidar_sample_delta_1s = delta;
            v814_lidar_sample_rate_1s_hz = rate_hz;

            if ((v814_lidar_sample_rate_1s_min_hz == 0.0f) ||
                (rate_hz < v814_lidar_sample_rate_1s_min_hz))
            {
                v814_lidar_sample_rate_1s_min_hz = rate_hz;
            }

            if (rate_hz > v814_lidar_sample_rate_1s_max_hz)
            {
                v814_lidar_sample_rate_1s_max_hz = rate_hz;
            }

            /*
             * The default-sensitivity Garmin profile was observed around
             * 180-200 Hz. Use a broad 1-second acceptance band so a single
             * 100 ms scheduling phase does not create a false alarm.
             */
            v814_lidar_rate_1s_ok =
                ((rate_hz >= 160.0f) && (rate_hz <= 210.0f)) ? 1U : 0U;

            if (rate_hz < 160.0f)
            {
                v814_lidar_low_rate_window_count++;
            }

            v814_last_lidar_update_count_1s = lidar_1s.update_count;
            v814_last_lidar_rate_ms = now_ms;
        }
    }

    /* ------------------------------ valve ADC ----------------------------- */

    v87_adc_timeout_count = needle_valve_hw_adc_timeout_count;

    v87_adc_ok =
        (needle_valve_hw_adc_timeout_count < 10UL) ? 1U : 0U;

    v87_valve_fault_snapshot = needle_valve_fault;

    /* -------------------------------- SD logger ------------------------------ */

    v87_sd_frame_delta_100ms =
        sd_logger_frame_count - last_sd_frame_count;

    last_sd_frame_count = sd_logger_frame_count;

    v87_sd_bytes_delta_100ms =
        sd_logger_total_bytes_written - last_sd_total_bytes;

    last_sd_total_bytes = sd_logger_total_bytes_written;

    /*
     * Ground/preflight remains exact ~30 Hz -> nominally 3 frames / 100 ms.
     * R3R10R1 intentionally stores only 1/4 of those samples in flight RAM
     * (7.5 Hz), so actual RAM-frame progression can legitimately be 0 or 1
     * in a 100 ms window. During RAM mode, health therefore watches the TIM5
     * capture heartbeat itself (still 30 Hz) plus the RAM overflow flag.
     */
    {
        uint32_t sd_timer_delta_100ms =
            sd_logger_timer_irq_count - last_sd_timer_irq_count;
        last_sd_timer_irq_count = sd_logger_timer_irq_count;

        if (sd_logger_r10_ram_mode != 0U)
        {
            v87_sd_progress_ok =
                ((sd_timer_delta_100ms >= 2UL) &&
                 (sd_timer_delta_100ms <= 4UL) &&
                 (sd_logger_r10_ram_overflow_count == 0UL)) ? 1U : 0U;
        }
        else
        {
            v87_sd_progress_ok =
                ((v87_sd_frame_delta_100ms >= 2UL) &&
                 (v87_sd_frame_delta_100ms <= 4UL)) ? 1U : 0U;
        }
    }

    v87_sd_ok =
        ((sd_logger_initialized != 0U) &&
         (sd_logger_ready != 0U) &&
         (sd_logger_mount_ok != 0U) &&
         (sd_logger_file_open != 0U) &&
         (sd_logger_logging_active != 0U) &&
         (sd_logger_capture_timer_started != 0U) &&
         (v87_sd_progress_ok != 0U) &&
         (sd_logger_error_count == 0UL) &&
         (sd_logger_write_error_count == 0UL) &&
         (sd_logger_dropped_frame_count == 0UL) &&
         (sd_logger_buffer_overrun_count == 0UL) &&
         (sd_logger_ring_overrun_count == 0UL) &&
         (sd_logger_async_timeout_count == 0UL)) ? 1U : 0U;

    /* --------------------------- NRF24 / SPI3 ---------------------------- */

    {
        NRF24_Data_t nrf = NRF24_GetData();
        BusStatus_t spi3 = BusManager_GetStatus(BUS_ID_SPI3);

        v89_nrf_initialized = nrf.initialized;
        v89_nrf_connected = nrf.connected;
        v89_nrf_mode = (uint8_t)nrf.mode;

        v89_nrf_status_reg = nrf.status_reg;
        v89_nrf_config_reg = nrf.config_reg;
        v89_nrf_rf_ch_reg = nrf.rf_ch_reg;
        v89_nrf_rf_setup_reg = nrf.rf_setup_reg;
        v89_nrf_fifo_status_reg = nrf.fifo_status_reg;

        v89_nrf_rx_count = nrf.rx_count;
        v89_nrf_tx_count = nrf.tx_count;
        v89_nrf_tx_fail_count = nrf.tx_fail_count;
        v89_nrf_error_count = nrf.error_count;

        v89_remote_link_active = RemoteControl_IsLinkActive();
        v89_remote_command = RemoteControl_GetCommand();
        v89_remote_last_sequence = remote_rx_last_sequence;
        v89_remote_valid_packet_count = remote_rx_valid_packet_count;
        v89_remote_invalid_packet_count = remote_rx_invalid_packet_count;
        v89_remote_timeout_count = remote_rx_timeout_count;
        v89_remote_irq_count = remote_rx_irq_count;
        v89_remote_last_packet_age_ms = remote_rx_last_packet_age_ms;

        v89_spi3_ok = spi3.ok;
        v89_spi3_fault = spi3.fault;
        v89_spi3_last_error = (uint8_t)spi3.last_error;
        v89_spi3_transaction_count = spi3.transaction_count;
        v89_spi3_error_count = spi3.error_count;
        v89_spi3_timeout_count = spi3.timeout_count;
        v89_spi3_busy_count = spi3.busy_count;
        v89_spi3_slow_count = spi3.slow_count;
        v89_spi3_last_duration_us = spi3.last_duration_us;
        v89_spi3_max_duration_us = spi3.max_duration_us;

        /*
         * Hardware health does NOT require an active RF link.
         * No transmitter may be present during the first coexistence test.
         */
        v89_nrf_hw_ok =
            ((nrf.initialized != 0U) &&
             (nrf.connected != 0U) &&
             (nrf.mode == NRF24_MODE_RX) &&
             (nrf.rf_ch_reg == APP_NRF24_CHANNEL) &&
             (nrf.rf_setup_reg == 0x06U) &&
             (spi3.fault == 0U) &&
             (spi3.timeout_count == 0UL)) ? 1U : 0U;
    }

    /* ------------------------------ Servo -------------------------------- */

    v810_servo_init_ok = vent_servo_init_ok;
    v810_servo_remote_command = vent_servo_remote_command;
    v810_servo_link_active = vent_servo_link_active;
    v810_servo_target_open = vent_servo_target_open;
    v810_servo_pulse_us = vent_servo_pulse_us;
    v810_servo_update_count = vent_servo_update_count;
    v810_servo_command_change_count = vent_servo_command_change_count;

    /*
     * Servo hardware health does not require an active RF link.
     * With no link, the correct fail-safe condition is CLOSED.
     */
    v810_servo_ok =
        ((vent_servo_init_ok != 0U) &&
         (vent_servo_pulse_us >= APP_VENT_SERVO_MIN_PULSE_US) &&
         (vent_servo_pulse_us <= APP_VENT_SERVO_MAX_PULSE_US) &&
         (((vent_servo_link_active == 0U) &&
           (vent_servo_target_open == 0U)) ||
          (vent_servo_link_active != 0U))) ? 1U : 0U;

    /* ------------------------ Attitude + Full ESKF ------------------------- */

    {
        const AttitudeEstimatorData_t *attitude =
            AttitudeEstimator_GetDataPtr();
        const FullStateESKFData_t *eskf =
            FullStateESKF_GetDataPtr();

        v815_attitude_delta_100ms =
            attitude->update_count - v815_last_attitude_update_count;
        v815_last_attitude_update_count = attitude->update_count;

        v815_eskf_predict_delta_100ms =
            eskf->predict_count - v815_last_eskf_predict_count;
        v815_last_eskf_predict_count = eskf->predict_count;

        v815_eskf_public_delta_100ms =
            eskf->public_output_count - v815_last_eskf_public_count;
        v815_last_eskf_public_count = eskf->public_output_count;

        v815_eskf_covariance_delta_100ms =
            eskf->covariance_predict_count - v815_last_eskf_covariance_count;
        v815_last_eskf_covariance_count = eskf->covariance_predict_count;

        v815_eskf_correction_delta_100ms =
            v815_eskf_correction_task_counter -
            v815_last_eskf_correction_count;
        v815_last_eskf_correction_count =
            v815_eskf_correction_task_counter;

        v815_attitude_ok =
            ((attitude->enabled != 0U) &&
             (attitude->initialized != 0U) &&
             (attitude->healthy != 0U) &&
             (attitude->numerical_error_count == 0UL)) ? 1U : 0U;

        v815_eskf_initialized = eskf->initialized;
        v815_eskf_healthy = eskf->healthy;
        v815_eskf_shadow_mode = eskf->shadow_mode;
        v815_eskf_lidar_reference_ready = eskf->lidar_reference_ready;
        v815_eskf_baro_reference_ready = eskf->baro_reference_ready;
        v815_eskf_stationary_detected = eskf->stationary_detected;

        v815_eskf_predict_max_exec_us = eskf->max_predict_exec_us;
        v815_eskf_correction_max_exec_us = eskf->max_correction_exec_us;
        v815_eskf_numerical_error_count = eskf->numerical_error_count;
        v815_eskf_gap_skip_count = eskf->gap_skip_count;
        v815_eskf_reset_count = eskf->reset_count;

        v815_eskf_position_x_m = eskf->position_x_m;
        v815_eskf_position_y_m = eskf->position_y_m;
        v815_eskf_position_z_m = eskf->position_z_m;
        v815_eskf_velocity_x_mps = eskf->velocity_x_mps;
        v815_eskf_velocity_y_mps = eskf->velocity_y_mps;
        v815_eskf_velocity_z_mps = eskf->velocity_z_mps;

        v815_eskf_roll_deg = eskf->roll_deg;
        v815_eskf_pitch_deg = eskf->pitch_deg;
        v815_eskf_yaw_deg = eskf->yaw_deg;

        v815_eskf_gyro_bias_x_dps = eskf->gyro_bias_x_dps;
        v815_eskf_gyro_bias_y_dps = eskf->gyro_bias_y_dps;
        v815_eskf_gyro_bias_z_dps = eskf->gyro_bias_z_dps;

        /*
         * Target architecture:
         * nominal propagation       = 1000 Hz
         * public estimator output   =  200 Hz
         * correction task           =  200 Hz
         * covariance propagation    =   50 Hz
         */
        v815_eskf_rate_ok =
            ((v815_attitude_delta_100ms >= 80UL) &&
             (v815_attitude_delta_100ms <= 120UL) &&
             (v815_eskf_predict_delta_100ms >= 80UL) &&
             (v815_eskf_predict_delta_100ms <= 120UL) &&
             (v815_eskf_public_delta_100ms >= 15UL) &&
             (v815_eskf_public_delta_100ms <= 25UL) &&
             (v815_eskf_correction_delta_100ms >= 15UL) &&
             (v815_eskf_correction_delta_100ms <= 25UL) &&
             (v815_eskf_covariance_delta_100ms >= 4UL) &&
             (v815_eskf_covariance_delta_100ms <= 6UL)) ? 1U : 0U;

        /*
         * BARO is deliberately not required for the shadow-core PASS flag.
         * This allows bench validation while the barometer is unplugged.
         * Full v815_integration_ok still follows the complete sensor stack.
         */

        v815a_canary_ok =
            ((v815a_magic_pre == 0x815A15A1UL) &&
             (v815a_magic_post == 0x815A15A2UL)) ? 1U : 0U;

        v815a_pointer_api_ok =
            ((SensorManager_GetDataPtr() != 0) &&
             (AttitudeEstimator_GetDataPtr() != 0) &&
             (FullStateESKF_GetDataPtr() != 0)) ? 1U : 0U;

        v815a_state_finite_ok =
            ((isfinite(eskf->position_x_m) != 0) &&
             (isfinite(eskf->position_y_m) != 0) &&
             (isfinite(eskf->position_z_m) != 0) &&
             (isfinite(eskf->velocity_x_mps) != 0) &&
             (isfinite(eskf->velocity_y_mps) != 0) &&
             (isfinite(eskf->velocity_z_mps) != 0) &&
             (isfinite(eskf->roll_deg) != 0) &&
             (isfinite(eskf->pitch_deg) != 0) &&
             (isfinite(eskf->yaw_deg) != 0)) ? 1U : 0U;

        v815_eskf_shadow_core_ok =
            ((v813_core_without_baro_ok != 0U) &&
             (v815_attitude_ok != 0U) &&
             (eskf->initialized != 0U) &&
             (eskf->healthy != 0U) &&
             (eskf->shadow_mode != 0U) &&
             (eskf->lidar_reference_ready != 0U) &&
             (v815_eskf_rate_ok != 0U) &&
             (v815a_canary_ok != 0U) &&
             (v815a_pointer_api_ok != 0U) &&
             (v815a_state_finite_ok != 0U) &&
             (eskf->numerical_error_count == 0UL)) ? 1U : 0U;

        v815a_shadow_core_ok = v815_eskf_shadow_core_ok;
    }

    /* ---------------------- V8.15C RAM / firmware guard ------------------- */

    {
        uint32_t msp_now;
        const FullStateESKFData_t *eskf_guard =
            FullStateESKF_GetDataPtr();

        __asm volatile ("mrs %0, msp" : "=r" (msp_now));

        v815c_msp_address = msp_now;
        v815c_bss_end_address = (uint32_t)(uintptr_t)&_ebss;

        if (msp_now > v815c_bss_end_address)
        {
            v815c_main_sram_gap_bytes =
                msp_now - v815c_bss_end_address;

            if (v815c_main_sram_gap_bytes <
                v815c_main_sram_gap_min_bytes)
            {
                v815c_main_sram_gap_min_bytes =
                    v815c_main_sram_gap_bytes;
            }
        }
        else
        {
            v815c_main_sram_gap_bytes = 0UL;
            v815c_main_sram_gap_min_bytes = 0UL;
        }

        v815c_firmware_identity_ok =
            (v815c_flash_magic == 0x815C15C1UL) ? 1U : 0U;

        v815c_ram_magic_ok =
            (v815c_ram_magic == 0x815C15C2UL) ? 1U : 0U;

        v815c_sd_ring_in_ccm =
            ((sd_logger_ring_address >= 0x10000000UL) &&
             (sd_logger_ring_end_address <= 0x10010000UL) &&
             (sd_logger_ring_end_address >
              sd_logger_ring_address)) ? 1U : 0U;

        v815c_estimator_basic_sanity_ok =
            ((eskf_guard != 0) &&
             (eskf_guard->enabled <= 1U) &&
             (eskf_guard->initialized <= 1U) &&
             (eskf_guard->healthy <= 1U) &&
             (eskf_guard->shadow_mode <= 1U) &&
             (eskf_guard->baro_fresh <= 1U) &&
             (eskf_guard->lidar_fresh <= 1U) &&
             (eskf_guard->origin_zeroed <= 1U) &&
             (eskf_guard->horizontal_position_valid <= 1U) &&
             (eskf_guard->vertical_position_valid <= 1U)) ? 1U : 0U;
    }

    /* V8.15D raw-address snapshot. */
    V815D_UpdateFixedDiagnostic();

    /* V8.16 active-control raw-address snapshot @ 0x1000F100. */
    V816_UpdateGNCDiagnostic();

    /* V8.17 MATLAB port snapshot @ 0x1000F180. */
    V819_UpdateVerticalDiagnostic();

    /* --------------------------- final integration ------------------------ */

    v87_integration_ok =
        ((v87_scheduler_ok != 0U) &&
         (v87_imu_ok != 0U) &&
         (v87_baro_ok != 0U) &&
         (v87_lidar_ok != 0U) &&
         (v87_sd_ok != 0U) &&
         (v87_valve_tick_ok != 0U) &&
         (v87_adc_ok != 0U) &&
         (needle_valve_fault == 0U)) ? 1U : 0U;

    v89_integration_ok =
        ((v89_scheduler_ok != 0U) &&
         (v87_imu_ok != 0U) &&
         (v87_baro_ok != 0U) &&
         (v87_lidar_ok != 0U) &&
         (v87_sd_ok != 0U) &&
         (v87_valve_tick_ok != 0U) &&
         (v87_adc_ok != 0U) &&
         (v89_nrf_hw_ok != 0U) &&
         (needle_valve_fault == 0U)) ? 1U : 0U;

    v810_integration_ok =
        ((v89_integration_ok != 0U) &&
         (v810_servo_ok != 0U)) ? 1U : 0U;

    v811_integration_ok =
        ((v810_integration_ok != 0U) &&
         (v811_lidar_sample_rate_ok != 0U)) ? 1U : 0U;

    /*
     * Bench/core health intentionally ignores BARO because the user may test
     * this stack with the barometer unplugged. Full integration still requires
     * v87_baro_ok through v810_integration_ok.
     */
    v813_core_without_baro_ok =
        ((v89_scheduler_ok != 0U) &&
         (v87_imu_ok != 0U) &&
         (v87_lidar_ok != 0U) &&
         (v87_valve_tick_ok != 0U) &&
         (v87_adc_ok != 0U) &&
         (v89_nrf_hw_ok != 0U) &&
         (v810_servo_ok != 0U) &&
         (v813_lidar_profile_config_ok != 0U) &&
         (v87_lidar_progress_ok != 0U) &&
         (needle_valve_fault == 0U)) ? 1U : 0U;

    v813_integration_ok =
        ((v810_integration_ok != 0U) &&
         (v813_lidar_profile_config_ok != 0U) &&
         (v87_lidar_progress_ok != 0U)) ? 1U : 0U;

    /*
     * During active logging this should remain true. After auto-finalize,
     * compare final frame_count * frame_size against total_bytes_written.
     */
    v813_sd_accounting_ok =
        ((uint32_t)(sd_logger_frame_count * 384UL) ==
         sd_logger_total_bytes_written) ? 1U : 0U;

    v814_integration_ok =
        ((v813_integration_ok != 0U) &&
         (v814_lidar_rate_1s_ok != 0U)) ? 1U : 0U;

    v815_integration_ok =
        ((v814_integration_ok != 0U) &&
         (v815_attitude_ok != 0U) &&
         (v815_eskf_initialized != 0U) &&
         (v815_eskf_healthy != 0U) &&
         (v815_eskf_shadow_mode != 0U) &&
         (v815_eskf_rate_ok != 0U) &&
         (v815_eskf_numerical_error_count == 0UL)) ? 1U : 0U;

    SystemMonitor_Update();
}
