#include "Services/CDCTelemetry/cdc_telemetry.h"

#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "usb_device.h"

#include "Modules/Sensors/SensorManager/sensor_manager.h"
#include "Modules/Sensors/Barometer/barometer.h"
#include "Modules/Sensors/Lidar/Lidar.h"

#include "Services/SystemMonitor/system_monitor.h"

#include "main.h"

#include <stdio.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* CPU monitor değişkenleri                                                    */
/* -------------------------------------------------------------------------- */

extern volatile float cpu_load_percent;
extern volatile float cpu_idle_percent;

/* -------------------------------------------------------------------------- */
/* Debug sayaçları                                                             */
/* -------------------------------------------------------------------------- */

extern volatile uint32_t imu_dma_complete_count;
extern volatile uint32_t imu_dma_error_count;

extern volatile uint32_t ms5611_update_count;
extern volatile uint32_t lidar_update_count;
extern volatile uint32_t lidar_read_count;

/* -------------------------------------------------------------------------- */
/* IMU Butterworth filter diagnostics                                         */
/* -------------------------------------------------------------------------- */

extern volatile uint8_t sensor_imu_filter_enabled;
extern volatile uint8_t sensor_imu_filter_config_ok;
extern volatile uint8_t sensor_imu_filter_initialized;

extern volatile uint32_t sensor_imu_filter_update_count;
extern volatile uint32_t sensor_imu_filter_reset_count;
extern volatile uint32_t sensor_imu_filter_max_gap_us;

/* -------------------------------------------------------------------------- */
/* USB device                                                                  */
/* -------------------------------------------------------------------------- */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* -------------------------------------------------------------------------- */
/* Config                                                                      */
/* -------------------------------------------------------------------------- */

#define CDC_TELEMETRY_PERIOD_MS             20U
#define CDC_TELEMETRY_BUFFER_SIZE           768U

#define CDC_TELEMETRY_BUSY_RECOVER_MS       1000U

/* -------------------------------------------------------------------------- */
/* Private variables                                                           */
/* -------------------------------------------------------------------------- */

static uint32_t cdc_last_transmit_time = 0U;
static uint32_t cdc_message_counter = 0U;

static char cdc_transmit_buffer[CDC_TELEMETRY_BUFFER_SIZE];

static uint32_t cdc_busy_start_time = 0U;

/* -------------------------------------------------------------------------- */
/* Live Expressions                                                            */
/* -------------------------------------------------------------------------- */

volatile uint32_t cdc_tx_send_count = 0UL;
volatile uint32_t cdc_tx_busy_count = 0UL;
volatile uint32_t cdc_tx_error_count = 0UL;
volatile uint32_t cdc_tx_recover_count = 0UL;
volatile uint32_t cdc_tx_skip_count = 0UL;

volatile uint8_t cdc_last_status = 0U;
volatile uint16_t cdc_last_message_length = 0U;

volatile uint8_t cdc_usb_ready = 0U;
volatile uint8_t cdc_usb_tx_state = 0U;

/* -------------------------------------------------------------------------- */

static int32_t CDCTelemetry_FloatToScaledInt(float value, float scale)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * scale) + 0.5f);
    }

    return (int32_t)((value * scale) - 0.5f);
}

/* -------------------------------------------------------------------------- */

static uint8_t CDCTelemetry_IsUSBReady(void)
{
    USBD_CDC_HandleTypeDef *cdc_handle;

    if (hUsbDeviceFS.pClassData == 0)
    {
        cdc_usb_ready = 0U;
        cdc_usb_tx_state = 255U;
        return 0U;
    }

    cdc_handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    cdc_usb_tx_state = (uint8_t)cdc_handle->TxState;

    if (cdc_handle->TxState == 0U)
    {
        cdc_usb_ready = 1U;
        cdc_busy_start_time = 0U;
        return 1U;
    }

    cdc_usb_ready = 0U;

    return 0U;
}

/* -------------------------------------------------------------------------- */

static void CDCTelemetry_RecoverUSBIfStuck(void)
{
    USBD_CDC_HandleTypeDef *cdc_handle;
    uint32_t now = HAL_GetTick();

    if (hUsbDeviceFS.pClassData == 0)
    {
        return;
    }

    cdc_handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (cdc_handle->TxState == 0U)
    {
        cdc_busy_start_time = 0U;
        return;
    }

    if (cdc_busy_start_time == 0U)
    {
        cdc_busy_start_time = now;
        return;
    }

    if ((now - cdc_busy_start_time) >= CDC_TELEMETRY_BUSY_RECOVER_MS)
    {
        /*
         * USB CDC bazen host tarafı durunca TxState=1'de kalabiliyor.
         * Burada sadece transmit state'i sıfırlıyoruz.
         */
        cdc_handle->TxState = 0U;

        cdc_busy_start_time = 0U;

        cdc_tx_recover_count++;
    }
}

/* -------------------------------------------------------------------------- */

void CDCTelemetry_Init(void)
{
    cdc_last_transmit_time = HAL_GetTick();
    cdc_message_counter = 0UL;

    cdc_busy_start_time = 0U;

    cdc_tx_send_count = 0UL;
    cdc_tx_busy_count = 0UL;
    cdc_tx_error_count = 0UL;
    cdc_tx_recover_count = 0UL;
    cdc_tx_skip_count = 0UL;

    cdc_last_status = 0U;
    cdc_last_message_length = 0U;

    cdc_usb_ready = 0U;
    cdc_usb_tx_state = 0U;
}

/* -------------------------------------------------------------------------- */

void CDCTelemetry_PrintPeriodic(void)
{
    uint32_t current_time = HAL_GetTick();

    if ((current_time - cdc_last_transmit_time) < CDC_TELEMETRY_PERIOD_MS)
    {
        return;
    }

    cdc_last_transmit_time = current_time;

    /*
     * En kritik kısım:
     * USB hazır değilse buffer'a yeni veri yazmıyoruz.
     * Çünkü önceki paket hâlâ bu buffer'ı kullanıyor olabilir.
     */
    if (CDCTelemetry_IsUSBReady() == 0U)
    {
        cdc_tx_busy_count++;
        cdc_tx_skip_count++;

        CDCTelemetry_RecoverUSBIfStuck();

        return;
    }

    cdc_message_counter++;

    SensorData_t sensor = SensorManager_GetData();
    BarometerData_t baro = Barometer_GetData();
    LidarData_t lidar = Lidar_GetData();
    SystemStatus_t system = SystemMonitor_GetStatus();

    int32_t acc_mg =
        CDCTelemetry_FloatToScaledInt(sensor.accel_norm_g, 1000.0f);

    int32_t ax_mg =
        CDCTelemetry_FloatToScaledInt(sensor.accel_x_g, 1000.0f);

    int32_t ay_mg =
        CDCTelemetry_FloatToScaledInt(sensor.accel_y_g, 1000.0f);

    int32_t az_mg =
        CDCTelemetry_FloatToScaledInt(sensor.accel_z_g, 1000.0f);

    int32_t gx_mdps =
        CDCTelemetry_FloatToScaledInt(sensor.gyro_x_dps, 1000.0f);

    int32_t gy_mdps =
        CDCTelemetry_FloatToScaledInt(sensor.gyro_y_dps, 1000.0f);

    int32_t gz_mdps =
        CDCTelemetry_FloatToScaledInt(sensor.gyro_z_dps, 1000.0f);

    int32_t filtered_acc_mg =
        CDCTelemetry_FloatToScaledInt(
            sensor.accel_filtered_norm_g,
            1000.0f
        );

    int32_t filtered_ax_mg =
        CDCTelemetry_FloatToScaledInt(
            sensor.accel_x_filtered_g,
            1000.0f
        );

    int32_t filtered_ay_mg =
        CDCTelemetry_FloatToScaledInt(
            sensor.accel_y_filtered_g,
            1000.0f
        );

    int32_t filtered_az_mg =
        CDCTelemetry_FloatToScaledInt(
            sensor.accel_z_filtered_g,
            1000.0f
        );

    int32_t filtered_gx_mdps =
        CDCTelemetry_FloatToScaledInt(
            sensor.gyro_x_filtered_dps,
            1000.0f
        );

    int32_t filtered_gy_mdps =
        CDCTelemetry_FloatToScaledInt(
            sensor.gyro_y_filtered_dps,
            1000.0f
        );

    int32_t filtered_gz_mdps =
        CDCTelemetry_FloatToScaledInt(
            sensor.gyro_z_filtered_dps,
            1000.0f
        );

    int32_t temp_cx100 =
        CDCTelemetry_FloatToScaledInt(baro.temperature_c, 100.0f);

    int32_t alt_cm =
        CDCTelemetry_FloatToScaledInt(baro.altitude_m, 100.0f);

    uint32_t cpu_x100 =
        (uint32_t)CDCTelemetry_FloatToScaledInt(cpu_load_percent, 100.0f);

    uint32_t idle_x100 =
        (uint32_t)CDCTelemetry_FloatToScaledInt(cpu_idle_percent, 100.0f);

    const char *status_text =
        (system.system_ok != 0U) ? "OK" : "FAULT";

    int message_length = snprintf(
        cdc_transmit_buffer,
        sizeof(cdc_transmit_buffer),

        "STM32_DATA,"
        "COUNT=%lu,"
        "TIME_MS=%lu,"

        "IMU=%u,"
        "ACC_MG=%ld,"
        "AX_MG=%ld,"
        "AY_MG=%ld,"
        "AZ_MG=%ld,"
        "GX_MDPS=%ld,"
        "GY_MDPS=%ld,"
        "GZ_MDPS=%ld,"

        "FACC_MG=%ld,"
        "FAX_MG=%ld,"
        "FAY_MG=%ld,"
        "FAZ_MG=%ld,"
        "FGX_MDPS=%ld,"
        "FGY_MDPS=%ld,"
        "FGZ_MDPS=%ld,"

        "FILT_EN=%u,"
        "FILT_OK=%u,"
        "FILT_INIT=%u,"
        "FILT_UPD=%lu,"
        "FILT_RST=%lu,"
        "FILT_GAP_MAX_US=%lu,"

        "IMU_DMA=%lu,"
        "IMU_ERR=%lu,"

        "BARO=%u,"
        "P_PA=%ld,"
        "T_CX100=%ld,"
        "ALT_CM=%ld,"
        "BARO_UPD=%lu,"

        "LIDAR=%u,"
        "DIST_CM=%u,"
        "LIDAR_UPD=%lu,"
        "LIDAR_READ=%lu,"

        "CPU_X100=%lu,"
        "IDLE_X100=%lu,"

        "CDC_SENT=%lu,"
        "CDC_BUSY=%lu,"
        "CDC_REC=%lu,"

        "FAULT=%u,"
        "STATUS=%s\r\n",

        (unsigned long)cdc_message_counter,
        (unsigned long)current_time,

        sensor.imu_valid,
        (long)acc_mg,
        (long)ax_mg,
        (long)ay_mg,
        (long)az_mg,
        (long)gx_mdps,
        (long)gy_mdps,
        (long)gz_mdps,

        (long)filtered_acc_mg,
        (long)filtered_ax_mg,
        (long)filtered_ay_mg,
        (long)filtered_az_mg,
        (long)filtered_gx_mdps,
        (long)filtered_gy_mdps,
        (long)filtered_gz_mdps,

        (unsigned int)sensor_imu_filter_enabled,
        (unsigned int)sensor_imu_filter_config_ok,
        (unsigned int)sensor_imu_filter_initialized,
        (unsigned long)sensor_imu_filter_update_count,
        (unsigned long)sensor_imu_filter_reset_count,
        (unsigned long)sensor_imu_filter_max_gap_us,

        (unsigned long)imu_dma_complete_count,
        (unsigned long)imu_dma_error_count,

        baro.healthy,
        (long)baro.pressure_pa,
        (long)temp_cx100,
        (long)alt_cm,
        (unsigned long)ms5611_update_count,

        lidar.distance_valid,
        lidar.distance_cm,
        (unsigned long)lidar_update_count,
        (unsigned long)lidar_read_count,

        (unsigned long)cpu_x100,
        (unsigned long)idle_x100,

        (unsigned long)cdc_tx_send_count,
        (unsigned long)cdc_tx_busy_count,
        (unsigned long)cdc_tx_recover_count,

        (unsigned int)system.fault_code,
        status_text
    );

    if (message_length <= 0)
    {
        cdc_tx_error_count++;
        return;
    }

    if (message_length >= (int)sizeof(cdc_transmit_buffer))
    {
        message_length = sizeof(cdc_transmit_buffer) - 1;
    }

    cdc_last_message_length = (uint16_t)message_length;

    uint8_t status = CDC_Transmit_FS(
        (uint8_t *)cdc_transmit_buffer,
        (uint16_t)message_length
    );

    cdc_last_status = status;

    if (status == USBD_OK)
    {
        cdc_tx_send_count++;
    }
    else if (status == USBD_BUSY)
    {
        cdc_tx_busy_count++;
        cdc_tx_skip_count++;
    }
    else
    {
        cdc_tx_error_count++;
    }
}
