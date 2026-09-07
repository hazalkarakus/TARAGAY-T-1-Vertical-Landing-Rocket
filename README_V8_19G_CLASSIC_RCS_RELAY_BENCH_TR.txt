TARAGAY-T1 V8.19G - KLASIK ATTITUDE ILE FIZIKSEL RCS ROLE TEZGAH TESTI
======================================================================

AMAC
----
TGY_FULL_SENSOR_NRF_SERVO_90 klasorunde calisan RCS yolunu V8.19F'nin
dogrulanmis sensor/ESKF/barometre altyapisinda fiziksel role cikisina baglamak.

KONTROL ZINCIRI
---------------
AttitudeEstimator roll/pitch + filtreli gyro X/Y
    -> AttitudeControl
    -> SolenoidOutput
    -> aktif-LOW dort kanalli role karti

ROLE GPIO ESLESMESI
-------------------
IN1 = PE7  = pozitif roll hatasi kanali = maske 0x01
IN2 = PE11 = negatif roll hatasi kanali = maske 0x02
IN3 = PE15 = pozitif pitch hatasi kanali = maske 0x04
IN4 = PB15 = negatif pitch hatasi kanali = maske 0x08

Cikislar aktif-LOW'dur:
- GPIO HIGH = role OFF
- GPIO LOW  = role ON

V8.19G GUVENLIK POLITIKASI
--------------------------
- Acilista butun RCS roleleri OFF.
- RCS, PA0 USER dugmesine basilmadan fiziksel cikis vermez.
- Ilk basma RCS role kontrolunu ARM eder.
- Ikinci basma sistemi DISARM eder ve tum roleleri hemen OFF yapar.
- Needle-valve fiziksel hareketi kapali kalir.
- Ayni eksendeki karsi iki role ayni anda acilamaz.
- Sensor/estimator hatasi tum roleleri OFF yapar ve fault latch olusturur.
- Yon degisiminde 30 ms break-before-make deadtime korunur.
- |aci| < 5 derece iken butun RCS roleleri OFF tutulur.
- |aci| >= 10 derece iken eski klasordeki duzeltme olayi baslar.

ZORUNLU ILK TEST KOSULU
-----------------------
Ilk testte basinc hatti, selenoidler ve motor gucu kesinlikle bagli olmayacak.
Yalniz STM32, sensorler ve role karti baglanacak. Once role LED'leri ve multimetre
ile GPIO HIGH=OFF / LOW=ON davranisi dogrulanacak.

TEZGAH TESTI
------------
1) Basinc hattini, selenoidleri ve motor beslemesini ayir.
2) F407 GND ile role karti GND'sini ortakla.
3) PE7, PE11, PE15 ve PB15 pinlerini sirasiyla IN1-IN4'e bagla.
4) Projeyi CubeIDE'ye import et; Project > Clean ve Build Project yap.
5) Koda girip Resume/F8 yap; karti duz ve sabit tutarak en az 10 saniye bekle.
6) Live Expressions'a LIVE_EXPRESSIONS_V8_19G.txt listesini ekle.
7) gnc_v816_attitude_state_ok=1 ve rcs/attitude fault degerleri 0 olmalidir.
8) PA0 USER dugmesine bir kez bas. gnc_v816_armed=1 ve button_stage=1 olmalidir.
9) Kart duzken rcs_requested/applied_mask=0 ve tum roleler OFF kalmalidir.
10) Karti tek eksende yavasca 10 dereceden fazla yatir; ilgili role LED'ini izle.
11) Karti tekrar 5 derecenin altina getir; tum roleler OFF olmalidir.
12) PA0'a tekrar bas; gnc_v816_armed=0 ve applied_mask=0 olmalidir.

KRITIK DEBUG NOTU
-----------------
RCS armed veya herhangi bir role aktifken CubeIDE Suspend/Pause yapma. Islemci
durursa yazilim zaman asimi ilerleyemez ve GPIO son seviyesinde kalabilir.
Memory ekranini yalniz sistem DISARM ve applied_mask=0 iken ac.

FAULT DAVRANISI
---------------
Fault latch olusursa butun roleler OFF olur. PA0'a ilk basma fault'u onaylar ve
sistemi disarmed durumda tutar. Yeniden arm etmek icin kosullar saglikliyken PA0'a
bir kez daha basmak gerekir.

DEGISTIRILMEYENLER
------------------
- BMP585 DRDY'siz gercek 200 Hz yolu
- IMU ve klasik AttitudeEstimator denklemleri
- FullStateESKF 1000/200/50 Hz mimarisi
- Sensor qualification ve SD logging
- NRF monitor yolu
- MATLAB/needle fiziksel cikisi kapali

SABIT BELLEK TESHISI
--------------------
0x1000F100 ilk uc word:
- magic   = 0x819C1907
- version = 0x00081907
- boyut   = 0x00000080

0x1000F100 flags:
- bit1  = RCS armed
- bit2  = fault latched
- bit4  = klasik attitude state OK
- bit8  = RCS dry-run; V8.19G'de 0 olmalidir
- bit15 = AttitudeControl fault yok
- bit16 = klasik AttitudeEstimator kaynagi
- bit17 = V8.19G fiziksel role-only bench modu

0x1000F120 = requested mask
0x1000F124 = applied mask
0x1000F140 = roll, mdeg
0x1000F144 = pitch, mdeg
0x1000F148 = yaw, mdeg
