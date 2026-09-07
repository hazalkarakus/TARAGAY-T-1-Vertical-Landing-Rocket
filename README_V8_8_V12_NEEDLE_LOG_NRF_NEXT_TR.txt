TGY V8.8 - SD V12 NEEDLE TELEMETRY
====================================

V8.7A ile doğrulanan sensör + SD + gerçek actuator mimarisi korunur.
Bu sürümün tek ana değişikliği flight.bin içine needle-valve telemetrisi
eklenmesidir.

NRF UNUTULMADI
--------------
NRF24 bu V8.8'de bilinçli olarak hâlâ kapalıdır.

SIRADAKİ:
V8.9 = NRF24 / SPI3 monitor-only entegrasyonu.

Önce radio:
- register connection
- RX polling / IRQ
- packet counters
- link timeout
- CPU/scheduler coexistence

doğrulanacak. NRF komutunu RCS/solenoidlere hemen bağlamayacağız.

V12 FORMAT
----------
V11 = 256 byte
V12 = 272 byte

Yeni 16 byte:
requested Valve_Cmd x10000  uint16
limited Valve_Cmd x10000    uint16
pot RAW                      uint16
ZERO ADC                     uint16
target ADC                   uint16
error ADC                    int16
RPWM                         uint8
LPWM                         uint8
state flags                  uint8
needle fault                 uint8

state flags:
bit0 enabled
bit1 zero_valid
bit2 lock

CRC16 offset = 270
frame size = 272

SD BUFFER
---------
64 frame x 272 = 17408 byte = 34 sector
double buffer = 34816 byte

capture ring:
256 x 272 = 69632 byte
50 Hz'de 5.12 s kapasite

LIVE EXPRESSIONS
----------------
sd_logger_needle_requested_cmd_x10000
sd_logger_needle_limited_cmd_x10000
sd_logger_needle_raw_adc
sd_logger_needle_zero_adc
sd_logger_needle_target_adc
sd_logger_needle_error_adc
sd_logger_needle_rpwm
sd_logger_needle_lpwm
sd_logger_needle_state_flags
sd_logger_needle_fault

ZAMANLAMA
---------
Logger needle status için NeedleValveController_GetStatus() çağırmaz.
Bu fonksiyon kısa süre interrupt kapattığı için TIM7 200 Hz loop'a jitter
eklememek adına mevcut volatile controller publish değerleri kopyalanır.

TEST
----
SD kart boot öncesi takılı.
Motor PSU başlangıçta kapalı.

Import -> Clean -> Build -> Debug -> Resume.

Statik:
SD health aynı V8.7A gibi PASS olmalı.
sd_logger_frame_struct_size = 272 olmalı.
frame drop / ring overrun / buffer overrun / async timeout = 0.

Dinamik:
1 USER -> ZERO
2 USER -> ENABLE
motor PSU düşük akım limiti
3 USER -> 30 -> 50 -> 80 -> 100 -> 0

Profil boyunca:
sd_logger_needle_requested_cmd_x10000
sd_logger_needle_limited_cmd_x10000
sd_logger_needle_raw_adc
sd_logger_needle_target_adc

değişmeli.

Profil PASS sonrası logger otomatik finalize olur.
flight.bin'i PC'ye al.

DECODER
-------
decode_flight_v12_needle.py

Kullanım:
python decode_flight_v12_needle.py flight.bin

Çıktı:
flight_V12_NEEDLE.csv

PASS
----
format_version = 12
frame_size = 272
CRC = tüm frameler doğru
sequence gap = 0
50 Hz kayıt
needle fault = 0
Valve_Cmd ve gerçek pot hareketi logda görünür.

V8.8 PASS -> V8.9 NRF24 SPI3 monitor-only.
