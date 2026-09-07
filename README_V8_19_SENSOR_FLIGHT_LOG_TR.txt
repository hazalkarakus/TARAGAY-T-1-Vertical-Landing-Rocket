TARAGAY-T1 V8.19 - SENSOR QUALIFICATION + 200 HZ FLIGHT LOG
================================================================

AMAÇ
----
MATLAB/Simulink kontrol kodu entegre edilmeden önce sensör zincirini hem
Live Expressions hem de SD kart üzerinden ölçülebilir şekilde doğrulamak.

Bu sürümde fiziksel otomatik kontrol çıkışları kapalıdır:
- APP_GNC_ACTIVE_ENABLED = 0
- RCS dry-run = 1
- Needle automatic physical output = 0
- Vent servo = 0

SENSÖR FREKANSLARI
------------------
IMU accepted task/data : nominal 1000 Hz
BMP585 real sample     : nominal 100.299 Hz
Garmin LIDAR real      : yaklaşık 175-185 Hz bench, qualification 165-205 Hz

BMP585 V8.19
------------
- NORMAL mode
- ODR code 0x0A = 100.299 Hz nominal
- pressure OSR x8
- temperature OSR x2
- sensor internal IIR bypass
- upper-layer Butterworth sample rate = 100 Hz
- startup ground-pressure reference = 200 valid sample (~2 s)
- barometer service task = 200 Hz
- a barometer sample is accepted ONLY after DRDY
- qualification baro rate window = 90..111 Hz

Bosch datasheet timing means 16x pressure OSR cannot sustain 100 Hz in NORMAL
mode; V8.19 intentionally uses 8x pressure / 2x temperature for the 100 Hz
flight profile.

SD FLIGHT LOG V14
-----------------
flight.bin format version = 14
frame size                = 384 bytes
state frame rate          = 200 Hz
data rate                 = 76,800 byte/s

3-second flight:
~600 state frames
~230,400 bytes (~225 KiB)

Every 200 Hz frame preserves:
- normal V13 sensor / ESKF / needle / NRF fields
- 5 chronological raw IMU samples from the rolling 1 kHz history
- IMU/baro/LIDAR sample age
- SensorQualification state and flags

Thus one flight.bin contains:
- 200 Hz flight/ESKF/control timeline
- approximately 1 kHz raw IMU evidence

V14 extension layout relative to frame start
---------------------------------------------
Old V13 fields remain at their previous offsets through byte 285.

+286 uint16  extension marker = 0x1419
+288..367    five raw IMU samples, 16 bytes each

Each fast IMU sample:
+0  uint32 timestamp_us
+4  int16 accel_x_raw
+6  int16 accel_y_raw
+8  int16 accel_z_raw
+10 int16 gyro_x_raw
+12 int16 gyro_y_raw
+14 int16 gyro_z_raw

+368 uint8  fast_imu_sample_count
+369 uint8  fast_imu_valid_mask
+370 uint16 imu_sample_age_us
+372 uint16 baro_sample_age_us
+374 uint16 lidar_sample_age_us
+376 uint8  sensor_qual_state
+377 uint8  sensor_qual_good_windows
+378 uint16 sensor_qual_flags_low
+380 uint16 sensor_qual_flags_high
+382 uint16 crc16

CRC covers bytes 0..381.

TIM5
----
TIM5 now captures at exact 200 Hz:
APB1 timer clock 84 MHz
prescaler 83 -> 1 MHz tick
period 4999 -> 200 Hz

The .ioc TIM5 period is also updated to 4999 so CubeMX regeneration does not
silently restore 50 Hz.

CCMRAM
------
Capture ring:
128 x 384 = 49,152 bytes

Fixed diagnostics still begin at 0x1000F000.
Linker asserts .ccmram remains below that fixed diagnostic window.

SENSOR RAW DIAGNOSTIC
---------------------
0x1000F200 remains the sensor qualification block.

V8.19 sensor magic:
0x819C19D1

Qualification targets:
IMU   950..1050 Hz
BARO   90..111 Hz
LIDAR 165..205 Hz

PASS:
sensor_qual_state = 2
sensor_qual_pass = 1
sensor_qual_good_windows >= 3

LIVE EXPRESSIONS - RECOMMENDED SET
----------------------------------
Core qualification:
sensor_qual_state
sensor_qual_pass
sensor_qual_good_windows
sensor_qual_flags
sensor_qual_imu_rate_hz
sensor_qual_baro_rate_hz
sensor_qual_lidar_rate_hz

IMU:
sensor_imu_valid
sensor_imu_calibration_complete
sensor_imu_calibration_sample_count
sensor_accel_filtered_norm_g
sensor_gyro_x_filtered_dps
sensor_gyro_y_filtered_dps
sensor_gyro_z_filtered_dps
sensor_imu_filter_initialized
sensor_imu_filter_max_gap_us
sensor_imu_valid_update_count

BMP585:
barometer_connected
barometer_healthy
barometer_calibrated
barometer_pressure_pa
barometer_filtered_altitude_m
barometer_vertical_speed_mps
barometer_last_sample_interval_us
barometer_update_count
bmp585_chip_id
bmp585_config_ok
bmp585_osr_eff
bmp585_drdy_count
ms5611_communication_error_count

LIDAR:
lidar_connected
lidar_distance_valid
lidar_distance_m
lidar_filtered_distance_m
lidar_last_sample_interval_us
lidar_update_count
lidar_error_count
lidar_timeout_count
lidar_dma_error_count
lidar_profile_config_ok
lidar_sample_pending_overrun_count

SD:
sd_logger_logging_active
sd_logger_frame_count
sd_logger_dropped_frame_count
sd_logger_ring_high_watermark
sd_logger_ring_overrun_count
sd_logger_error_count
sd_logger_write_error_count
sd_logger_async_timeout_count
sd_logger_dma_retry_exhausted_count
sd_logger_timer_last_interval_us
sd_logger_timer_max_interval_us
sd_logger_timer_max_isr_duration_us
sd_logger_frame_struct_size
sd_logger_fast_imu_push_count
sd_logger_fast_imu_duplicate_skip_count
sd_logger_fast_imu_history_count
sd_logger_fast_imu_last_valid_mask

EXPECTED BENCH VALUES
---------------------
After startup calibration and 5-10 s stable run:

sensor_qual_state              = 2
sensor_qual_pass               = 1
sensor_qual_good_windows       >= 3

sensor_qual_imu_rate_hz        ~1000
sensor_qual_baro_rate_hz       ~100
sensor_qual_lidar_rate_hz      ~175..185 typical

sensor_imu_calibration_complete = 1
sensor_accel_filtered_norm_g    ~1.0 while stationary
stationary filtered gyro values near 0 dps

barometer_connected            = 1
barometer_healthy              = 1
bmp585_chip_id                 = 0x51 for BMP585
bmp585_config_ok               = 1
bmp585_osr_eff bit7            = 1 (ODR valid)
barometer_last_sample_interval_us ~10000 us

lidar_connected                = 1
lidar_distance_valid           = 1
lidar_profile_config_ok        = 1
lidar_error/timeout/DMA error should not continuously grow

sd_logger_logging_active       = 1
sd_logger_frame_struct_size    = 384
sd_logger_timer_last_interval_us ~5000 us
sd_logger_dropped_frame_count  = 0
sd_logger_ring_overrun_count   = 0
sd_logger_error_count          = 0
sd_logger_write_error_count    = 0
sd_logger_async_timeout_count  = 0

sd_logger_fast_imu_history_count = 5 after startup
sd_logger_fast_imu_last_valid_mask = 31 (0b11111) once history is full

TEST
----
1) No pressure/gas.
2) Clean Project -> Build -> Debug.
3) Keep board stationary through IMU startup calibration.
4) Resume 10-15 s.
5) Use the Live Expressions list above for quick checking.
6) Do not judge sensor quality from one instantaneous value only.
7) Stop/finalize logging cleanly, copy flight.bin and send it for analysis.
8) flight.bin is the main source for rate/jitter/noise/drift/drop analysis.
