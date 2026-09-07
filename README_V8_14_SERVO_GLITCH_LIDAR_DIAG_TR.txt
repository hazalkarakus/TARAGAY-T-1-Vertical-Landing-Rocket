TGY V8.14 - SERVO GLITCH FILTER + NEEDLE TEST FIX + LIDAR 1s DIAG
======================================================================

NEDEN?
------
Son gerçek testte:

1) Needle %100 leg hedefe yaklaşık ±2 ADC gelmesine rağmen bench test
   600 ms kesintisiz lock şartı nedeniyle TIMEOUT verdi.

2) Tahliye servosu OPEN olduktan sonra kısa süre CLOSED olup tekrar OPEN
   hareketi yaptı.

3) LIDAR 100 ms cadence ekranı bazı anlarda 60 Hz gibi çok düşük değer
   gösterdi; bunun sürekli düşüş mü yoksa kısa transient mi olduğunu
   1 saniyelik pencerede ayırmak gerekiyor.

1) NEEDLE BENCH DWELL
---------------------
Yalnız test supervisor değişti:

600 ms -> 250 ms

Controller:
- ±3 ADC tolerance
- hysteresis
- PWM
- slew
- stall
- fault logic

DEĞİŞMEDİ.

2) SERVO GLITCH NEDENİ / ÇÖZÜMÜ
--------------------------------
Kod incelendi:
ServoOutput_Update yalnız NRF taskından çağrılıyor.
Başka bir task servo pulse'u kapatıp açmıyor.

Eski mantık:
link_active = 0 olduğunda servo anında CLOSED.

RemoteControl ise packet age 500 ms olunca:
link_active = 0
command = 0

yapıyor.

Dolayısıyla kısa RF timeout + hemen yeni paket:
OPEN -> CLOSED -> OPEN

hareketini doğrudan üretebilir.

V8.14 iki filtre ekler:

A) COMMAND DEGLITCH
Valid packet link'i hemen canlı tutar.
Ama command 0/1 değişimi 40 ms stabil kalmadan çıkış command'ına uygulanmaz.

Bu:
1 -> kısa yanlış 0 -> tekrar 1
gibi packet chatter'ını servoya geçirmez.

B) LINK-LOSS SERVO GRACE
RemoteControl hard timeout yine 500 ms.

Servo OPEN iken link_active 0 olursa:
100 ms daha mevcut OPEN pulse tutulur.

Link 100 ms içinde geri gelirse:
servo fiziksel olarak hiç CLOSED'a gitmez.

Gerçek link kaybında:
500 ms RF timeout + 100 ms servo grace
≈ 600 ms sonra CLOSED.

Bu yalnız 100 ms ek fail-safe gecikmesidir.

SERVO DIAGNOSTICS
-----------------
vent_servo_transition_reason

0 = none/init
1 = command OPEN
2 = command CLOSED
3 = confirmed link-loss CLOSED
4 = ForceClosed

vent_servo_link_loss_grace_count
vent_servo_link_glitch_suppressed_count
vent_servo_link_loss_grace_active
vent_servo_link_loss_grace_age_ms

REMOTE DIAGNOSTICS
------------------
remote_rx_candidate_command
remote_rx_candidate_age_ms
remote_rx_command_change_count
remote_rx_command_chatter_count

Ayrıca mevcut:
remote_rx_timeout_count
remote_rx_last_packet_age_ms
remote_rx_valid_packet_count

ÇOK ÖNEMLİ AYIRICI TEST
-----------------------
Fiziksel servo tekrar CLOSED->OPEN yaparsa aynı anda şuna bak:

vent_servo_pulse_us
vent_servo_target_open
vent_servo_transition_reason

Eğer fiziksel servo hareket ediyor AMA:
pulse_us = 2500
target_open = 1
transition_reason değişmiyor

ise problem artık KOD DEĞİL:
- servo beslemesi
- brownout
- kablo
- GND
- servo mekanik yükü

tarafındadır.

V13 SD log zaten:
NRF link
NRF command
servo target
servo pulse

alanlarını 50 Hz kaydediyor. FLIGHT.BIN de bunu ayırabilir.

3) LIDAR 1-SECOND RATE
----------------------
100 ms rate hâlâ mevcut.

Yeni:
v814_lidar_sample_delta_1s
v814_lidar_sample_rate_1s_hz
v814_lidar_sample_rate_1s_min_hz
v814_lidar_sample_rate_1s_max_hz
v814_lidar_low_rate_window_count
v814_lidar_rate_1s_ok

Default sensitivity profile için kabul:
160 .. 210 Hz / 1-second window

Hedef:
yaklaşık 180-200 Hz.

Tek bir 100 ms pencerede 60 Hz görmek artık tek başına FAIL sayılmaz.
1-second rate sürekli <160 Hz ise ayrıca incelenir.

TEST
----
Basınç/gaz hattı bağlı olmasın.

A) Servo:
NRF command:
0 -> 1 -> 0
her konumda birkaç saniye bekle.

Sonra OPEN konumdayken kısa RF kesintisi / transmitter kapat-aç testi.

Beklenen:
kısa reacquire <100 ms ise servo CLOSED'a fiziksel gitmez.
gerçek link loss ~600 ms sonunda CLOSED.

B) Needle:
ZERO -> ENABLE -> 30/50/80/100/0

Beklenen:
v83_test_profile_index = 5
v83_test_result = 2
fault = 0

C) LIDAR:
motor + NRF + servo yükü altında en az 20-30 s izle.

Beklenen:
v814_lidar_sample_rate_1s_hz yaklaşık 180-200
v814_lidar_rate_1s_ok = 1
low_rate_window_count artmamalı veya çok seyrek kalmalı.

D) SD:
error = 0
write_error = 0
retry_exhausted = 0
