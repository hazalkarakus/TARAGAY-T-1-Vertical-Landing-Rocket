TGY V8.10 - NRF24 + TAHLIYE SERVO ACTIVE
===========================================

TABAN
-----
V8.9A'da doğrulanan:
- IMU 1 kHz
- BMP585 200 Hz
- LIDAR 200 Hz
- NRF24 / SPI3 200 Hz
- Needle exact TIM7 200 Hz
- SD exact TIM5 50 Hz
- V12 flight.bin + needle telemetry

V8.10'DA NE DEĞİŞTİ?
--------------------
Önceki full projedeki NRF -> tahliye servo bağlantısı tekrar aktif edildi.

NRF KOMUT MANTIĞI
-----------------
remote command = 0
    -> tahliye servo CLOSED

remote command = 1
    -> tahliye servo OPEN

NRF link kaybı > 500 ms
    -> RemoteControl command = 0
    -> servo CLOSED

ServoOutput ayrıca link_active kontrolü yaptığı için ikinci bir fail-safe vardır.

SERVO DONANIMI
--------------
Signal: PB6
Timer : TIM4_CH1
AF    : AF2
PWM   : 50 Hz / 20 ms

Mevcut önceki full-project ayarı AYNEN korunur:
CLOSED = 1500 us
OPEN   = 2500 us

app_config:
APP_VENT_SERVO_ENABLED         = 1
APP_VENT_SERVO_CLOSED_PULSE_US= 1500
APP_VENT_SERVO_OPEN_PULSE_US  = 2500

NRF
---
CSN = PD0
CE  = PD1
IRQ = PD3 / EXTI3
SPI3
channel = 76
payload = 4
address = TGY01

GÜVENLİK SINIRI
---------------
NRF artık SADECE tahliye servosunu sürer.

NRF hâlâ şunları sürmez:
- RCS solenoidleri
- needle Valve_Cmd
- diğer servo/aktüatörler

İLK TEST
--------
Basınç/gaz hattı bağlı olmadan veya sistem basınçsızken test et.
Servo mekanizmasının mekanik stopa bastırmadığından emin ol.

SD kart boot öncesi takılı olabilir.
Needle motor PSU kapalı kalabilir.

Import -> Clean -> Build -> Debug -> Resume.

Live Expressions:

NRF:
v89_nrf_initialized
v89_nrf_connected
v89_nrf_mode
v89_nrf_hw_ok
v89_remote_link_active
v89_remote_command
v89_remote_valid_packet_count
v89_remote_timeout_count

SERVO:
vent_servo_init_ok
vent_servo_remote_command
vent_servo_link_active
vent_servo_target_open
vent_servo_pulse_us
vent_servo_update_count
vent_servo_command_change_count

V8.10:
v810_servo_ok
v810_integration_ok
v810_servo_init_ok
v810_servo_remote_command
v810_servo_link_active
v810_servo_target_open
v810_servo_pulse_us
v810_servo_update_count
v810_servo_command_change_count

TRANSMITTER KAPALIYKEN
----------------------
Beklenen:
v89_nrf_hw_ok       = 1
vent_servo_init_ok  = 1
link_active         = 0
target_open         = 0
pulse_us            = 1500
v810_servo_ok       = 1

TRANSMITTER KOMUT 0
-------------------
Beklenen:
link_active         = 1
remote_command      = 0
target_open         = 0
pulse_us            = 1500

TRANSMITTER KOMUT 1
-------------------
Beklenen:
link_active         = 1
remote_command      = 1
target_open         = 1
pulse_us            = 2500

TRANSMITTER KAPAT / LINK KES
----------------------------
500 ms civarında:
link_active         -> 0
remote_command      -> 0
target_open         -> 0
pulse_us            -> 1500
timeout_count       +1

SCHEDULER
---------
NRF + servo service = 200 Hz

v89_nrf_task_delta_100ms ~20
overrun = 0
deadline miss = 0

Needle TIM7 200 Hz, IMU, BMP585, LIDAR ve SD timing bozulmamalı.

NOT
---
2500 us önceki full-project servo ayarıdır.
Mekanik bağlantı değiştiyse 90 dereceyi yeniden fiziksel olarak doğrula;
servo ile mekanik stopa zorla bastırma.

SONRAKİ
-------
Bu sürüm geçince:
- NRF + servo uzun link/failsafe testi
- gerekirse servo durumunu SD V13 frame'e ekleme
- sonra estimator / ESKF entegrasyonu
- RCS komut bağlantısı daha sonra ayrı güvenli aşamada yapılır
