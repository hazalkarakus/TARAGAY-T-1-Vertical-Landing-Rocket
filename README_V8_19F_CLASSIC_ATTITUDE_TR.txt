TARAGAY-T1 V8.19F - ILK KLASORDEKI ROLL/PITCH/YAW KAYNAGI
======================================================================

AMAC
----
TGY_FULL_SENSOR_NRF_SERVO_90 klasorunde dogru calistigi gozlenen
AttitudeEstimator roll, pitch ve yaw ciktilarini aktif GNC/RCS girisine
yeniden baglamak.

DEGISTIRILENLER
---------------
- Roll, pitch ve yaw: AttitudeEstimator quaternion/Euler hatti.
- Roll hizi: ilk klasordeki gibi filtreli gyro X.
- Pitch hizi: ilk klasordeki gibi filtreli gyro Y.
- RCS esik mantigi yalniz yeni Euler orneginde calisir (yaklasik 100 Hz).
- Estimator resetinden uretilen ilk Euler ornegi RCS tarafinda kullanilmaz.

DEGISTIRILMEYENLER
------------------
- IMU surucusu, kalibrasyonu ve filtreleri.
- AttitudeEstimator denklemleri ve tum kazanc/agirlik ayarlari.
- FullStateESKF 1000/200/50 Hz mimarisi.
- ESKF konum, hiz, barometre ve LIDAR birlestirmesi.
- BMP585 DRDY'siz gercek 200 Hz yolu.
- Sensor qualification, SD logging, MATLAB vana kontrolu.
- Fiziksel cikis guvenlikleri: GNC kapali, RCS dry-run acik.

NOT
---
Ilk klasordeki eksen isimleri, isaretler ve donus yonleri aynen korunmustur;
ilave eksen remap veya isaret degisikligi yapilmamistir.

TEST
----
1) Motor ve solenoid gucunu kapali tut.
2) Projeyi Clean + Build yap ve karta yukle.
3) Kart duz ve sabitken 10 saniye bekle.
4) CubeIDE Memory ekraninda 0x1000F100 adresini acip Refresh yap.
5) 0x1000F100-0x1000F17F goruntusunu kaydet.

GNC DIAGNOSTIC ALANLARI
-----------------------
0x1000F110 : flags; bit16=1 classic AttitudeEstimator kaynagi
0x1000F140 : roll  [mdeg]
0x1000F144 : pitch [mdeg]
0x1000F148 : yaw   [mdeg]
0x1000F14C : roll rate  [mdeg/s]
0x1000F150 : pitch rate [mdeg/s]

0x1000F000 blogu FullStateESKF konum/hiz ve ham ESKF teshisi olarak
korunmustur. Kontrole giden klasik acilar 0x1000F100 blogundadir.
