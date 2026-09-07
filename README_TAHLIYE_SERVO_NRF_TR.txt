TGY STM32F407 - TAHLIYE SERVOSU + nRF 0/1 ENTEGRASYONU
=====================================================

AMAÇ
----
Mevcut STM32F103 -> nRF24L01 -> STM32F407 ON/OFF komutunu tahliye servosuna
baglamak.

KOMUT MANTIGI
-------------
nRF = 0 -> Tahliye KAPALI
nRF = 1 -> Tahliye ACIK
nRF link kaybi (>500 ms) -> mevcut RemoteControl modulu 0'a doner; servo da
KAPALI konuma gider.

SERVODAN STM32'YE SINYAL
------------------------
PWM pini: PB6
Timer: TIM4 Channel 1
Alternate Function: AF2
PWM periyodu: 20 ms (50 Hz)

SERVONUN 3 KABLOSU
------------------
Sinyal (genelde sari/turuncu) -> STM32 PB6
+V (genelde kirmizi)          -> servo icin AYRI 10-12.6 V guc hatti
GND (genelde kahverengi/siyah)-> servo guc GND
STM32 GND                     -> servo guc GND ile ORTAK

ONEMLI GUC NOTU
---------------
Bu 150 kg sinifi servo STM32'nin 5 V/3.3 V pininden BESLENMEZ.
Mevcut karttaki +12 V regulatorunun servo tepe akimini tasiyabildigi kesin
olmadan servoyu o hatta baglama. Servo icin uygun akim kapasiteli ayri bir
10-12.6 V regule hat ve sigorta kullan.

YAZILIMDA EKLENEN DOSYALAR
--------------------------
App/Services/ServoOutput/servo_output.c
App/Services/ServoOutput/servo_output.h

DEGISTIRILEN DOSYALAR
---------------------
App/app.c
- ServoOutput_Init() eklendi.
- Her App_Run dongusunde nRF komutu ServoOutput_Update() fonksiyonuna veriliyor.

App/Common/app_config.h
- Servo PWM ve OPEN/CLOSED pulse ayarlari eklendi.

ILK TEST AYARLARI
-----------------
APP_VENT_SERVO_CLOSED_PULSE_US = 1500 us
APP_VENT_SERVO_OPEN_PULSE_US   = 1600 us

Bu degerler ILK GUVENLI MASA TESTI icindir; 1600 us tam 90 derece acilma
degeri olarak kabul edilmemistir.

90 DERECE KALIBRASYON
---------------------
1. Basinc hattini ve vanayi servodan ayir veya sistemi tamamen basincsiz yap.
2. nRF komutunu 0 yap. Servo 1500 us konumuna gelir.
3. Vanayi mekanik olarak tam KAPALI konuma getir.
4. Servo hornunu bu konumda vanaya hizala.
5. nRF komutunu 1 yap. Servo 1600 us konumuna gider.
6. Hareket yonu dogruysa APP_VENT_SERVO_OPEN_PULSE_US degerini 50 us artir:
   1650, 1700, 1750 ...
7. Vana tam 90 derece ACIK oldugu anda o pulse degerini kaydet.
8. Mekanik stopa servo ile bastirma; stopa gelmeden hemen once kalacak sekilde
   degeri ayarla.

LIVE EXPRESSIONS
----------------
vent_servo_init_ok
vent_servo_remote_command
vent_servo_link_active
vent_servo_target_open
vent_servo_pulse_us
vent_servo_update_count
vent_servo_command_change_count

BEKLENEN
--------
nRF 0: vent_servo_target_open = 0, pulse = CLOSED degeri
nRF 1: vent_servo_target_open = 1, pulse = OPEN degeri
link kesilirse: target_open = 0

PB6 SECIM NEDENI
----------------
- Mevcut projede PB6 baska bir periferikte kullanilmiyor.
- STM32F407'de PB6, TIM4_CH1 PWM alternate function'ina sahiptir.
- IMU, barometre, nRF, SDIO, LIDAR ve mevcut RCS role pinleriyle cakisma yoktur.
