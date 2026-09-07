TGY V8.11 CLEANUP - TRUE LIDAR 200 Hz + V13 NRF/SERVO LOG + SD FINALIZE
=============================================================================

BU SÜRÜM NEDEN VAR?
-------------------
V8.10 gerçek test / FLIGHT(3).BIN analizi üç şeyi gösterdi:

1) LIDAR task 200 Hz çağrılıyordu ama gerçek yeni sample yaklaşık 150 Hz idi.
2) Profil bittikten sonra blocking FATFS finalize nedeniyle NRF task deadline
   miss sayacı artabiliyordu.
3) NRF + servo fiziksel çalışıyordu fakat V12 flight.bin içinde NRF/servo
   telemetrisi yoktu.

V8.11 bu üç konuyu temizler.

1) LIDAR GERÇEK ~200 Hz
-----------------------
Eski:
Lidar_Update task = 200 Hz
gerçek sample     ~150 Hz

Sebep:
LIDAR conversion wait = 5 ms ve state-machine de 5 ms'de bir servis edildiği
için scheduler/tick fazına göre bazı ölçümler bir sonraki 5 ms slotuna
kalabiliyordu.

Yeni:
LIDAR state-machine service = 1000 Hz
sensör conversion = 5 ms
hedef gerçek sample = ~200 Hz

Filter sample-rate ayarı 200 Hz olarak korunur.

Live:
v87_lidar_task_delta_100ms       yaklaşık 100
v87_lidar_update_delta_100ms     yaklaşık 20
v811_lidar_sample_delta_100ms    yaklaşık 20
v811_lidar_sample_rate_ok        = 1

2) SD FINALIZE DEADLINE CLEANUP
-------------------------------
SDLogger_Stop() içindeki truncate/sync/close FATFS gereği blocking olabilir.

V8.10'da stop bittikten sonra scheduler eski next_run timestamp'lerini
yakalamaya çalışıyor ve maintenance süresini runtime deadline miss gibi
sayabiliyordu.

V8.11:
SDLogger_Stop()
    ->
Scheduler_Rebase()

yapar.

Rebase:
- task run counters'ı SIFIRLAMAZ
- overrun/deadline counters'ı SIFIRLAMAZ
- yalnız next_run zamanlarını "şimdi"ye tekrar bağlar
- CPU measurement window'u yeniden başlatır

Böylece bilinçli SD finalize maintenance window'u, sonradan 10-15 adet
sahte catch-up deadline miss üretmez.

Live:
v811_sd_finalize_started
v811_sd_finalize_done
v811_sd_finalize_duration_ms
v811_scheduler_rebase_count

v811_nrf_deadline_before_finalize
v811_nrf_deadline_after_finalize
v811_lidar_deadline_before_finalize
v811_lidar_deadline_after_finalize

PASS:
before == after
scheduler_rebase_count = 1

3) V13 BINARY LOG
-----------------
V12 = 272 byte
V13 = 288 byte

V12 needle alanlarının tamamı korunur.

Yeni 16 byte NRF/servo alanı:
nrf_link_active             uint8
nrf_command                 uint8
nrf_last_sequence           uint8
servo_target_open           uint8
servo_pulse_us              uint16
nrf_last_packet_age_ms      uint16
nrf_valid_packet_count      uint32
nrf_irq_count_low           uint16
nrf_rx_count_low            uint16

CRC16 offset = 286
frame size   = 288

SD buffer:
64 x 288 = 18432 byte = 36 sector

Live:
sd_logger_nrf_link_active
sd_logger_nrf_command
sd_logger_nrf_last_sequence
sd_logger_servo_target_open
sd_logger_servo_pulse_us
sd_logger_nrf_last_packet_age_ms
sd_logger_nrf_valid_packet_count
sd_logger_nrf_irq_count_low
sd_logger_nrf_rx_count_low

4) SİSTEM
---------
IMU                1000 Hz
LIDAR service      1000 Hz
LIDAR real sample   ~200 Hz
BMP585               200 Hz
NRF + servo           200 Hz
Needle TIM7 exact     200 Hz
SD TIM5 exact          50 Hz
System monitor          10 Hz

RCS hâlâ NRF'e BAĞLI DEĞİL.
Estimator/ESKF hâlâ bu cleanup testinde aktif değil.

İLK TEST
--------
SD kart boot öncesi takılı.
Basınç/gaz hattı bağlı olmasın.
Needle motor PSU başlangıçta kapalı olabilir.

Import -> Clean -> Build -> Debug -> Resume.

Önce LIDAR:
v87_lidar_task_delta_100ms
v87_lidar_update_delta_100ms
v811_lidar_sample_delta_100ms
v811_lidar_sample_rate_ok

lidar_last_sample_interval_us
lidar_min_sample_interval_us
lidar_max_sample_interval_us

Beklenen:
task delta    ~100 / 100 ms
sample delta  ~20 / 100 ms
last interval ~5000 us
sample_rate_ok = 1

NRF/SERVO:
v89_nrf_hw_ok
v89_remote_link_active
v89_remote_command
v810_servo_ok
vent_servo_target_open
vent_servo_pulse_us

SD V13:
sd_logger_initialized
sd_logger_ready
sd_logger_mount_ok
sd_logger_logging_active
sd_logger_frame_count
sd_logger_dropped_frame_count
sd_logger_ring_overrun_count
sd_logger_async_timeout_count

sd_logger_nrf_link_active
sd_logger_nrf_command
sd_logger_servo_target_open
sd_logger_servo_pulse_us

SYSTEM:
v811_integration_ok
v89_nrf_task_deadline_miss_count
v87_lidar_task_deadline_miss_count
cpu_load_percent

DİNAMİK TEST
------------
1 USER -> needle ZERO
2 USER -> needle ENABLE
motor PSU düşük akım limiti
3 USER -> 30/50/80/100/0

Aynı test boyunca NRF transmitter:
- command 0
- command 1
- command 0
şeklinde servo testi de yapabilirsin.

Basınçsız / güvenli bench koşulunda yap.

Profil PASS sonrası SD otomatik finalize olur.

Finalde:
v83_test_result = 2
v83_test_profile_index = 5

v811_sd_finalize_done = 1
v811_scheduler_rebase_count = 1

nrf deadline before == after
lidar deadline before == after

sd_logger_file_open = 0
sd_logger_logging_active = 0

NOT:
SD kapandıktan sonra v87_sd_ok / integration flag'lerinin 0 olması normaldir.
Aktif logging sırasında v811_integration_ok = 1 olmalıdır.

BIN TEST
--------
SD'den FLIGHT.BIN'i al.

Projede:
decode_flight_v13.py

Kullanım:
python decode_flight_v13.py FLIGHT.BIN

Beklenen:
Format        V13
Frame size    288
CRC           tüm frameler doğru
Sequence gaps 0
Average SD    50 Hz
Needle faults 0

CSV içinde aynı zaman ekseninde:
- Valve_Cmd
- Pot RAW / target
- NRF link
- NRF command
- servo target
- servo pulse

görülebilir.

V8.11 PASS'TEN SONRA
--------------------
Sensör + actuator + NRF + servo + SD altyapısını dondurup estimator/ESKF
entegrasyonuna geçebiliriz.
