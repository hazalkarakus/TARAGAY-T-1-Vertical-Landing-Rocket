V42 - IMU FIZIKSEL SPI1 HAT TESTI
=================================

Taban: V41 IMU SPI auto-probe + V40 UART driver diagnostics.

Degisen kaynak dosyalari:
- App/app.c
- App/Modules/Sensors/IMU/imu.c
- App/Modules/Sensors/IMU/imu.h

Diger sensor, SD, nRF, kontrol ve scheduler kaynaklarina dokunulmadi.

V42 bir kez, scheduler baslamadan once sadece IMU hatlarini test eder:
- PA4 = CS
- PA5 = SPI1 SCK
- PA6 = SPI1 MISO
- PA7 = SPI1 MOSI

Testler:
1) CS/SCK/MOSI GPIO high-low readback
2) CS HIGH iken MISO internal pull-up/pull-down takibi
3) Cok yavas GPIO bit-bang WHO_AM_I okuma, SPI Mode 0
4) Cok yavas GPIO bit-bang WHO_AM_I okuma, SPI Mode 3
5) SPI1 pinleri AF5'e ve onceki SPI profil ayarlarina geri getirilir

UART yeni satirlari:
IMUPHY DONE=1 CS=1/0 SCK=1/0 MOSI=1/0 MISO_PU_PD=1/0 GPIOOK=1 MFREE=1
IMUBB MODE0=0x.. MODE3=0x.. RESP=0/1 AUTOPROBE=... MODE=... DIV=... TRY=... RD=... PE=...

Beklenen saglikli WHO_AM_I: 0x6A (projedeki ALT kabul 0x6B).

Yorum:
- GPIOOK=1: STM32 CS/SCK/MOSI pinlerini high/low suruyor.
- MFREE=1: CS high iken MISO pull-up/pull-down'u takip ediyor; hat hard-stuck gorunmuyor.
- RESP=1 ve MODE0/3=0x6A/0x6B: fiziksel hat/sensor cevabi var, HAL SPI yapisi incelenmeli.
- RESP=0 ve iki bit-bang WHO da 0x00: sensor besleme/CS/MISO/MOSI/SCK surekliligi daha kuvvetli supheli.
- MISO_PU_PD=0/0: MISO low'a tutuluyor/kisa devre/yanlis hat olabilir.
- MISO_PU_PD=1/1: MISO high'a tutuluyor/kisa devre/yanlis hat olabilir.
