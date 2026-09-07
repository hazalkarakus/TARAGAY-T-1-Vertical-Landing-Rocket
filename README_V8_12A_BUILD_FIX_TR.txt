TGY V8.12A FIX
==============

EKRANDAKİ BUILD HATASI:
sd_logger.c:
- size of array 'SDLoggerFrameSizeCheck' is negative
- size of array 'SDLoggerCRCOffsetCheck' is negative

NEDEN:
V13'teki NRF/servo 16-byte extension içinde uint32_t
nrf_valid_packet_count offset 278'de tanımlanmıştı.

ARM/GCC uint32_t'yi doğal 4-byte sınıra hizaladığı için 2 byte padding ekledi.

Bu yüzden:
V8.12 tasarlanan frame = 288 byte, CRC offset = 286
gerçek C struct        = 292 byte, CRC offset = 288

ÇÖZÜM:
NRF/servo alanlarının sırası değiştirildi.

Yeni fiziksel layout:
270  nrf_link_active          u8
271  nrf_command              u8
272  nrf_valid_packet_count   u32
276  nrf_last_sequence        u8
277  servo_target_open        u8
278  servo_pulse_us           u16
280  nrf_last_packet_age_ms   u16
282  nrf_irq_count_low        u16
284  nrf_rx_count_low         u16
286  crc16                    u16
288  frame end

Böylece doğal alignment ile:
sizeof(SDLoggerFrame_t) = 288
offsetof(crc16)         = 286

V12 needle telemetry offsetleri aynen korunmuştur.

AYRICA:
app_tasks.c içinde kullanılan lidar_fast_config_ok ve ilgili Garmin
diagnostic global'leri Lidar.h içinde extern olarak tanımlandı.

BUILD:
1) Eski V8.12 -> Terminate
2) V8.12A FIX import
3) Clean
4) Build

Unused IMU function warning'leri build'i durdurmaz.
