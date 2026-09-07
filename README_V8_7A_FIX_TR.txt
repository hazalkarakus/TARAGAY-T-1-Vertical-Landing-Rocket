TGY V8.7A FIX
=============

V8.7 BUILD HATASI DÜZELTİLDİ.

Hata App/Core/Tasks/app_tasks.c satır 47-50 civarındaydı:

    v87_sd_ok = 0U;
    v87_sd_progress_ok = 0U;
    v87_sd_frame_delta_100ms = 0UL;
    v87_sd_bytes_delta_100ms = 0UL;

Bu dört atama yanlışlıkla GLOBAL scope'a yazılmıştı.
C derleyicisi bunları tip verilmemiş yeni global tanımlar gibi yorumladığı için:

- data definition has no type or storage class
- type defaults to int
- conflicting type qualifiers

hataları oluşuyordu.

V8.7A'da:
- yanlış global atamalar tamamen kaldırıldı;
- volatile global değişken tanımları korundu;
- reset atamaları doğru şekilde AppTasks_Init() içine taşındı.

Ekrandaki şu iki uyarı build'i durdurmaz:
- IMU_StartRawDMARead defined but not used
- IMU_ReadMultiBlocking defined but not used

Bunlar sadece unused-function warning'dir; şu aşamada temizlemeye gerek yok.

TEST
----
1) V8.7 eski debug session -> Terminate
2) V8.7A FIX import et
3) Clean
4) Build

Build geçerse önce motor PSU KAPALI:
- SD kart boot öncesi takılı
- Debug -> Resume
- SD + coexistence Live Expressions kontrolü

Özellikle:
sd_logger_initialized
sd_logger_ready
sd_logger_mount_ok
sd_logger_file_open
sd_logger_logging_active
sd_logger_frame_count
sd_logger_dropped_frame_count
sd_logger_ring_overrun_count
sd_logger_async_timeout_count

v87_sd_ok
v87_sd_progress_ok
v87_sd_frame_delta_100ms
v87_integration_ok

v87_imu_ok
v87_baro_ok
v87_lidar_ok
v87_valve_tick_ok

needle_valve_fault
needle_valve_hw_adc_timeout_count

Beklenen:
v87_sd_frame_delta_100ms ~= 5
v87_sd_ok = 1
v87_integration_ok = 1
needle_valve_fault = 0
