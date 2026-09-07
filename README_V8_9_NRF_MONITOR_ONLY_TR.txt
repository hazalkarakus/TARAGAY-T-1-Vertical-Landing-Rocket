TGY V8.9 - NRF24 / SPI3 MONITOR-ONLY INTEGRATION
=================================================

TABAN
-----
V8.8 V12 testinde doğrulanan sistem korunur:
- IMU 1 kHz
- BMP585 200 Hz
- LIDAR 200 Hz
- Needle exact TIM7 200 Hz
- SD exact TIM5 50 Hz
- V12 272-byte flight.bin + needle telemetry

V8.9 EKLENTİSİ
--------------
NRF24L01+ / SPI3 geri açıldı.

DONANIM / MEVCUT PROJE PINLERİ
-------------------------------
SPI3:
- NRF CSN = PD0
- NRF CE  = PD1
- NRF IRQ = PD3 / EXTI3
- channel = 76
- payload = 4 byte
- address = "TGY01"

NRF IRQ priority = 6.
Needle TIM7 priority = 5.
Bu nedenle needle 200 Hz kontrol IRQ'su radio IRQ'dan daha yüksek öncelikte.

ÇOK ÖNEMLİ - MONITOR ONLY
--------------------------
NRF paketleri okunur ve mevcut RemoteControl protokolü doğrulanır.

AMA:
- RCS sürülmez
- solenoid sürülmez
- servo sürülmez
- needle Valve_Cmd değiştirilmez

Remote command yalnız Live Expressions + status LED için izlenir.

Mevcut 4-byte packet:
byte0 = 0xA5
byte1 = command (0 veya 1)
byte2 = sequence
byte3 = 0xA5 ^ command ^ sequence ^ 0x5A

TASK
----
Task0 IMU        1000 Hz
Task1 BMP585      200 Hz
Task2 LIDAR       200 Hz
Task3 NRF monitor 200 Hz
Task4 monitor      10 Hz

SD logger main-loop async service + TIM5 50 Hz capture.
Needle TIM7 exact 200 Hz IRQ.

İLK TEST - TRANSMITTER GEREKMİYOR
---------------------------------
SD kart boot öncesi takılı.
Motor PSU kapalı.

Import -> Clean -> Build -> Debug -> Resume.

Live Expressions:

NRF HW:
nrf24_initialized
nrf24_connected
nrf24_mode
nrf24_status_reg
nrf24_config_reg
nrf24_rf_ch_reg
nrf24_rf_setup_reg
nrf24_fifo_status_reg
nrf24_error_count

V8.9:
v89_nrf_initialized
v89_nrf_connected
v89_nrf_mode
v89_nrf_rf_ch_reg
v89_nrf_rf_setup_reg
v89_nrf_hw_ok

v89_nrf_task_delta_100ms
v89_nrf_task_max_exec_us
v89_nrf_task_overrun_count
v89_nrf_task_deadline_miss_count

SPI3:
v89_spi3_ok
v89_spi3_fault
v89_spi3_transaction_count
v89_spi3_error_count
v89_spi3_timeout_count
v89_spi3_slow_count
v89_spi3_last_duration_us
v89_spi3_max_duration_us

SYSTEM:
v89_scheduler_ok
v89_integration_ok

v87_imu_ok
v87_baro_ok
v87_lidar_ok
v87_sd_ok
v87_valve_tick_ok

cpu_load_percent
cpu_idle_percent

BEKLENEN - NRF DONANIM
----------------------
v89_nrf_initialized = 1
v89_nrf_connected   = 1
v89_nrf_mode        = 2  (RX)
v89_nrf_rf_ch_reg   = 76
v89_nrf_rf_setup_reg= 6
v89_nrf_hw_ok       = 1

v89_nrf_task_delta_100ms ~20
overrun = 0
deadline miss = 0

SPI3:
fault = 0
timeout = 0
error = 0
transaction_count sürekli artar

TRANSMITTER YOKSA
-----------------
Bu NORMAL:
v89_remote_link_active = 0
v89_remote_valid_packet_count = 0

Bunlar v89_nrf_hw_ok veya v89_integration_ok'u düşürmez.

RF PAKET TESTİ
--------------
Transmitter açıldığında Live Expressions:

v89_remote_link_active
v89_remote_command
v89_remote_last_sequence
v89_remote_valid_packet_count
v89_remote_invalid_packet_count
v89_remote_timeout_count
v89_remote_irq_count
v89_remote_last_packet_age_ms
v89_nrf_rx_count

Beklenen:
- valid_packet_count sürekli artar
- last_sequence değişir
- irq_count artar
- nrf_rx_count artar
- link_active = 1
- last_packet_age_ms düşük kalır
- invalid_packet_count idealde 0
- nrf_error_count = 0

Transmitter kapatılırsa mevcut timeout:
500 ms

yaklaşık yarım saniye sonra:
link_active = 0
remote_command = 0
timeout_count +1

Bu FAIL değildir; failsafe link timeout davranışıdır.

ACTUATOR COEXISTENCE
--------------------
NRF hardware statik test PASS olduktan sonra gerçek actuator profilini tekrar
çalıştırabilirsin:

1 USER -> ZERO
2 USER -> ENABLE
motor PSU düşük akım limiti
3 USER -> 30/50/80/100/0

Finalde:
v83_test_result = 2
v83_test_profile_index = 5
needle_valve_fault = 0

Profil sırasında:
v89_nrf_hw_ok = 1
v87_imu_ok = 1
v87_baro_ok = 1
v87_lidar_ok = 1
v87_valve_tick_ok = 1

SD logger profil sonunda otomatik finalize olur; bundan sonra sd_ok ve
integration_ok'ın 0'a düşmesi normaldir.

SONRAKİ
-------
V8.9 monitor-only PASS olduktan sonra radio'yu doğrudan RCS'e bağlamayacağız.

Önce:
- uzun süre packet-loss testi
- link timeout/failsafe
- CPU + SPI3 marjı
- SD log altında RF coexistence

doğrulanır.

Daha sonra kontrollü bir sürümde RemoteControl command -> RCS state-machine
bağlantısı yapılır.
