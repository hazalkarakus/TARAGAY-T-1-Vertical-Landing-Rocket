TGY STM32F407 - SENSORLER + nRF24 + 4 KANAL RCS ROLE/SELENOID SURUMU
====================================================================

BU PAKETTE KORUNAN SISTEMLER
----------------------------
- ISM330DLC IMU ve filtreleri
- Barometre
- LIDAR
- AttitudeEstimator, VerticalEKF ve FullStateESKF
- SD kart kaydi
- USART2 telemetri
- STM32F103 + nRF24 ON/OFF haberlesmesi
- F407 uzerindeki yesil LED'in nRF komutuna gore yanip sonmesi

EKLENEN SISTEMLER
-----------------
- IMU roll/pitch acisina gore 4 kanalli RCS kontrolu
- 4 kanalli role karti icin aktif-LOW GPIO cikislari
- Karsi yon valflerinin ayni anda acilmasini engelleyen yazilim interlock'u
- 30 ms break-before-make yon degistirme guvenligi
- Sensor/estimator hatasinda tum cikislari OFF yapan fail-safe
- Live Expressions icin rcs_* ve relay_* global degiskenleri

ROLE BAGLANTILARI
-----------------
F407 PE7  -> Role IN1 -> pozitif roll hatasini duzeltme kanali
F407 PE11 -> Role IN2 -> negatif roll hatasini duzeltme kanali
F407 PE15 -> Role IN3 -> pozitif pitch hatasini duzeltme kanali
F407 PB15 -> Role IN4 -> negatif pitch hatasini duzeltme kanali
F407 GND  -> Role GND
Role VCC  -> Role kartinin gerektirdigi besleme (kullanilan kart 5 V ise 5 V)

ONEMLI: Cikis mantigi aktif-LOW'dur.
- GPIO HIGH = role OFF
- GPIO LOW  = role ON

ILK TEST - SELENOID BAGLAMADAN
------------------------------
1. Ilk testte gercek selenoidleri ve basinc hattini BAGLAMA.
2. Yalnizca F407, IMU, nRF ve role kartini bagla.
3. Projeyi STM32CubeIDE'ye import et.
4. Project > Clean yap.
5. Project > Build Project yap.
6. Kodu F407'ye yukle ve Resume/F8 yap.
7. Kart acilirken IMU kalibrasyonu icin karti sabit tut.
8. Baslangictan sonra en az 3 saniye bekle.
9. Karti roll veya pitch yonunde 10 dereceden fazla yatir.
10. Uygun role kanalinin LED'i yanmalidir.
11. Acinin mutlak degeri 5 derecenin altina geldiginde roleler OFF olur.

BASIT TEZGAH DAVRANISI
-----------------------
- |aci| < 5 derece: tum roleler OFF
- 5-10 derece: LIDAR/VerticalEKF tehlikeli inis ongorusu yoksa OFF
- |aci| >= 10 derece: ilgili duzeltme rolesi devreye girer
- Karsi iki role ayni eksende ayni anda acilamaz
- Sensor/estimator hatasinda tum roleler OFF

nRF DAVRANISI
-------------
Bu surumde nRF haberlesmesi onceki gibi korunmustur:
- F103 anahtar ON ve link aktif: F407 yesil LED yanip soner
- F103 anahtar OFF veya link kaybi: yesil LED soner

nRF anahtari bu surumde RCS rolelerini arm/disarm ETMEZ.
RCS roleleri dogrudan IMU/attitude kontrolune gore calisir.

LIVE EXPRESSIONS - ILK EKLENECEKLER
----------------------------------
main_loop_heartbeat
imu_ok
rcs_state
rcs_fault
rcs_config_valid
rcs_estimator_healthy
rcs_attitude_fresh
rcs_roll_deg
rcs_pitch_deg
rcs_roll_rate_dps
rcs_pitch_rate_dps
rcs_valve_demand_mask
rcs_valve_applied_mask
relay_in1_pe7_active
relay_in2_pe11_active
relay_in3_pe15_active
relay_in4_pb15_active
rcs_output_interlock_fault
rcs_control_update_count
rcs_safety_service_count
rcs_command_change_count
rcs_reversal_count
rcs_event_count
remote_rx_link_active
remote_rx_command
remote_rx_valid_packet_count

MASKE DEGERLERI
---------------
0x00 = tum kanallar OFF
0x01 = IN1 / PE7
0x02 = IN2 / PE11
0x04 = IN3 / PE15
0x08 = IN4 / PB15

NORMAL CALISMA BEKLENTISI
-------------------------
rcs_config_valid = 1
rcs_fault = 0
rcs_estimator_healthy = 1
rcs_control_update_count surekli artar
rcs_safety_service_count surekli artar
rcs_output_interlock_fault = 0

DERLEME NOTU
------------
Paket icinde hazir HEX yoktur. Kendi CubeIDE surumunde Clean + Build yaparak
Debug/tgy.hex dosyasini olustur. CubeMX .ioc dosyasindan yeniden kod uretmeden
once bu kaynak kodlarini yedekle; role GPIO'lari uygulama servisinde dinamik
olarak yapilandirilmistir.

GUVENLIK
--------
- Ilk test sadece role LED'leriyle, basincsiz ve selenoidsiz yapilmalidir.
- Role cikislarini multimetre ile dogrulamadan selenoid baglama.
- Role kontaklari uzerinden selenoid beslemesi verilirken ayri sigorta kullan.
- STM32 GPIO pininden selenoidi dogrudan besleme.
