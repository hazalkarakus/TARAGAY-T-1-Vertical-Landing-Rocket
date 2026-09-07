TGY V8.19M - IGNE VANA 10 CEVRIM DAYANIKLILIK TESTI
=====================================================

AMAC
----
V8.19L ile manuel 0.10 -> 0.25 -> 0.50 -> 0.75 -> 1.00 -> 0.00 profili
basariyla tamamlandi. V8.19M ayni dogrulanmis vana kontrolcusu, 195 ADC/tur
kalibrasyonu, otomatik homing ve PA0 guvenlik filtresini korur. Yalniz bench
test yoneticisi, gecici motor/surucu hareket kesilmelerini yakalamak icin
otomatik 10 tam acma-kapama cevrimine donusturulmustur.

Bu paket BASINCSIZ bench testi icindir; ucus yazilimi degildir.

KORUNAN GUVENLIK SINIRLARI
--------------------------
- Kapali bolge          : 1015..1023 ADC
- 1 tam vana turu       : 195 ADC
- Maksimum hareket      : 4,5 tur / 878 ADC
- Acma yonu             : ADC azalir
- Kapama yonu           : ADC artar
- Konum toleransi       : +/-3 ADC
- Homing PWM            : RPWM 65
- Hareket stall korumasi, ADC kopma ve yon kontrolleri aktiftir.
- GNC Valve_Cmd fiziksel motora bagli DEGILDIR.
- RCS cikislari dry-run/guvenli durumdadir.

TEST AKISI
----------
1. Sistem acildiktan sonra 3 saniye bekler.
2. Vana dusuk PWM ile otomatik kapanir ve ZERO alinir.
3. Homing bitince BTS7960 tamamen kapatilir.
4. PA0 en az 1 saniye LOW kaldiginda test baslatmaya hazir olur.
5. PA0'a bir kez en az 0,5 saniye basmak 10 cevrim testini baslatir.
6. Her cevrim:
   - 1.00 komutu: yaklasik 4,5 tur tam acma
   - acik ucta 2 saniye konum dogrulama
   - 0.00 komutu: tam kapama
   - kapali ucta 2 saniye konum dogrulama
   - BTS7960 kapali halde 5 saniye soguma
7. Onuncu kapanis dogrulaninca surucu kapanir ve TEST_PASS olur.

Her hareket bacagi icin 20 saniye, testin tamami icin 10 dakika ust sinir
vardir. Kontrolcunun kendi 800/1000 ms ilerlememe korumalari daha once devreye
girerek motoru guvenli sekilde durdurabilir.

PA0 GUVENLIGI
-------------
- Gecerli test baslatma basisi: en az 500 ms kesintisiz HIGH.
- Yeni basma kabul edilmeden once: en az 1000 ms kesintisiz LOW.
- Kisa darbeler komut sayilmaz.
- Test aktifken yeniden kurulan PA0'daki yeni HIGH kenari, hareket/dwell/
  cooldown ayrimi olmadan acil ABORT yapar.
- ABORT, FAULT ve PASS durumlarinda LPWM/RPWM ile L_EN/R_EN kapatilir.

AUTO STATE
----------
0 : BOOT_WAIT
1 : HOMING
2 : READY_DISABLED
3 : ENDURANCE_ACTIVE
4 : TEST_PASS
5 : ABORTED
6 : FAULT

ENDURANCE PHASE
---------------
0 : IDLE
1 : OPENING
2 : OPEN_DWELL
3 : CLOSING
4 : CLOSE_DWELL
5 : COOLDOWN
6 : PASS
7 : ABORTED
8 : FAULT

FAILURE REASON
--------------
0 : yok
1 : vana kontrolcusu fault verdi
2 : hareket bacagi 20 saniyeyi asti
3 : toplam test 10 dakikayi asti
4 : enable veya komut reddedildi
5 : operator PA0 ile ABORT etti
6 : ZERO gecerliligi kayboldu
7 : beklenmeyen test durumu

ILK CALISTIRMA
--------------
1. Basincli/gaz hattini ve RCS selenoidlerini ayir.
2. Vana ve aktarim mekanizmasinin serbest oldugunu kontrol et.
3. Motor beslemesini daha once dogrulanan dusuk akim limitiyle ac.
4. Reset/Restart -> Resume yap ve PA0'a dokunma.
5. Homing bittikten sonra beklenen:

   v819m_auto_state                = 2
   v819m_auto_ready_for_test       = 1
   v819m_auto_home_start_count     = 1
   v819m_auto_home_pass_count      = 1
   needle_valve_zero_valid         = 1
   needle_valve_fault              = 0
   needle_valve_enabled            = 0
   needle_valve_lpwm/rpwm          = 0

6. PA0'a yaklasik 1 saniye basip birak. Bundan sonra yeniden PA0'a basma.
7. Debugger'i hareket sirasinda Suspend etme.
8. Anormal ses, kablo isinmasi veya guc kaynagi akim siniri gorulurse motor
   beslemesini kes.

BASARILI BITIS
--------------
v819m_auto_state                    = 4
v819m_endurance_phase               = 6
v819m_endurance_cycles_completed    = 10
v819m_failure_reason                = 0
needle_valve_raw_adc                = 1015..1023
needle_valve_zero_valid             = 1
needle_valve_lock                   = 1
needle_valve_fault                  = 0
needle_valve_enabled                = 0
needle_valve_lpwm/rpwm              = 0

Her cevrimin acik/kapali ADC ve hareket sureleri 10 elemanli dizilerde
saklanir. Bir fault olursa reset atmadan FAILURE alanlari okunmalidir.

