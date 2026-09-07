TGY V8.19I - COMBINED GNC DRY-RUN
=================================

AMAÇ
----
RCS yön/röle testi tamamlandıktan sonra zaman kaybetmeden yatay ve dikey
kontrol zincirlerini aynı yazılımda çalıştırmak.

AKTİF ZİNCİR
------------
- IMU / BMP585 / LIDAR
- Classic AttitudeEstimator: roll, pitch, yaw
- Full-State ESKF: Z konumu ve Z hızı
- AttitudeControl -> SolenoidOutput -> fiziksel RCS röleleri
- VerticalLandingControl V10.6 -> compute-only Valve_Cmd

FİZİKSEL ÇIKIŞLAR
-----------------
- RCS röle GPIO: aktif
- İğne vana PWM/enable: kapalı
- Tahliye servosu: kapalı

Bu sürüm uçuş yazılımı değildir. Basınç hattı, selenoid bobinleri ve iğne vana
motor beslemesi bağlı olmadan birleşik kontrol doğrulaması içindir.

PA0
---
- İlk basış: estimatorlar sağlıklıysa birleşik GNC ARM
- İkinci basış: DISARM ve bütün RCS röleleri OFF
- Fault varken ilk basış: fault latch temizlenir ve sistem DISARM kalır

RCS EŞLEŞMESİ
-------------
- X+ -> IN4
- X- -> IN3
- Y+ -> IN2
- Y- -> IN1

SABİT BELLEK
------------
0x1000F100: GNC bloğu, 128 byte
  magic   = 0x819C1909
  version = 0x00081909
  size    = 0x00000080

Flags içinde bit18 = V8.19I combined dry-run aktif.

0x1000F180: dikey kontrol bloğu, 64 byte
  magic   = 0x819C19D1
  version = 0x00081909
  size    = 0x00000040

Dikey blok alanları:
+0x00 magic
+0x04 version
+0x08 size
+0x0C flags
+0x10 z [mm]
+0x14 v [mm/s]
+0x18 mass [g]
+0x1C pressure [mbar]
+0x20 Valve_Cmd x10000
+0x24 z source
+0x28 v source
+0x2C mass source
+0x30 pressure source
+0x34 sequence begin
+0x38 sequence end
+0x3C checksum

HIZLI TEST
----------
1. Basınç hattını, selenoid bobinlerini ve iğne vana motor beslemesini ayır.
2. Clean -> Build -> Debug -> Resume.
3. Sistemi düz ve sabit şekilde 15 saniye çalıştır.
4. PA0'a bir kez bas; X/Y yön testini birer kez yap.
5. PA0'a tekrar bas ve bütün rölelerin kapandığını gör.
6. Yalnız DISARM durumunda Suspend yap.
7. 0x1000F100-0x1000F1BF Memory görüntüsünü al.

NOT
---
F_RATED=1120 N, mass=27.5 kg ve pressure=300 bar bu sürümde model
değerleridir. Canlı basınç, güncel kütle ve kesin vana-itki haritası bağlı
değildir. İğne vananın fiziksel otomatik çıkışı bu nedenle kapalıdır.
