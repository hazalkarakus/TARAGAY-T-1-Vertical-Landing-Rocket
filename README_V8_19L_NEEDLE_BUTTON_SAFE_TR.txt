TGY V8.19L - IGNE VANA OTOMATIK REFERANS + PA0 GUVENLIK DUZELTMESI
=================================================================

AMAC
----
V8.19K otomatik homing testinde PA0'a dokunulmadigi halde tek bir kisa HIGH
darbesi buton basisi sayilmis ve surucuyu MANUAL_ACTIVE durumuna gecirmisti.
V8.19L bu davranisi duzeltir ve V8.19J ile dogrulanan fiziksel vana surucusunu
guvenli otomatik kapanma/ZERO + filtrelenmis PA0 komut testiyle tekrar dogrular.
Bu paket basincsiz bench testi icindir; ucus yazilimi degildir.

KORUNAN SISTEMLER
-----------------
- IMU, BMP585, LIDAR, attitude estimator ve ESKF aynen aktiftir.
- SD logger aynen aktiftir.
- NRF -> tahliye servo yolu aynen korunmustur.
- RCS fiziksel cikislari dry-run/guvenli durumdadir.
- GNC Valve_Cmd fiziksel igne vana motoruna bagli DEGILDIR.

KALIBRASYON
-----------
- Kapali bolge          : 1015..1023 ADC
- 1 tam vana turu       : 195 ADC
- Acma yonu             : ADC azalir
- Kapama yonu           : ADC artar
- Maksimum hareket      : 4,5 tur / 878 ADC
- Konum kilidi          : +/-3 ADC, birakma esigi 4 ADC

OTOMATIK REFERANS
-----------------
App_Init tamamlandiktan sonra sistem 3 saniye bekler. Ardindan:

1. ADC kopuk/0 bolgesinde degilse BTS7960 enable edilir.
2. Vana RPWM=65 ile dusuk hizda kapama yonunde surulur.
3. RAW ADC'nin artmasi zorunludur.
4. RAW ADC 1015 veya ustunde 100 ms kararlı kalinca motor durur.
5. O anki RAW ADC, ZERO_ADC olarak kaydedilir.
6. LPWM/RPWM ve L_EN/R_EN sifirlanir.

Guvenlik kesmeleri:
- ADC <= 3, ardisik 3 kontrol ticki : fault 4
- 1 saniye boyunca en az 2 ADC ilerleme yok : fault 2
- Toplam homing suresi 15 saniyeyi asar : fault 5
- Kapama komutunda ADC baslangictan 8 sayim azalir : fault 6
- Homing sirasinda PA0'a basilir : ABORT, tum motor cikislari kapanir

PA0 GUVENLIK FILTRESI
---------------------
- Dahili GPIO_PULLDOWN aktiftir.
- Komut girisi, PA0 en az 1000 ms kesintisiz LOW kalmadan kurulmaz.
- Gecerli basma icin PA0 en az 500 ms kesintisiz HIGH kalmalidir.
- 500 ms'den kisa HIGH darbeleri komut sayilmaz ve glitch sayacina yazilir.
- Her komuttan sonra yeni basma kabul edilmeden once PA0 yeniden en az
  1000 ms LOW kalmalidir.
- Hareket sirasinda, onceki basma birakilip giris tekrar kurulduktan sonraki
  yeni HIGH kenari motoru hizla ABORT durumuna alir.
- PA0 acilista HIGH takili kalirsa surucu komut moduna gecemez.

ILK OTOMATIK REFERANS TESTI
---------------------------
Basinc/gaz hatti ve RCS selenoidleri bagli OLMAMALIDIR.

1. Vana tam acik uca yakin olmasin; tercihen 0,5..2 tur acik birak.
2. BTS7960 ve STM32 GND ortak olsun.
3. Motor beslemesini dusuk akim limitiyle ac.
4. Clean -> Build -> Debug -> Resume yap.
5. Ilk 3 saniye PWM'ler sifir kalmalidir.
6. Sonra RPWM yaklasik 65 olur ve RAW ADC artar.
7. Referans bitince beklenen:

   v819l_auto_state                  = 2
   needle_valve_homing_active        = 0
   needle_valve_homing_complete      = 1
   needle_valve_zero_valid           = 1
   needle_valve_zero_adc             = 1015..1023
   needle_valve_enabled              = 0
   needle_valve_fault                = 0
   needle_valve_lpwm/rpwm            = 0
   v819l_auto_ready_for_command      = 1  (PA0 1 saniye LOW kaldiktan sonra)
   v819l_auto_button_press_count     = 0
   v819l_button_raw                  = 0
   v819l_button_stable               = 0
   v819l_button_armed                = 1

Motor hareket ederken debugger'da Suspend yapma.

PA0 KOMUT TESTI
---------------
Otomatik referans basariyla bittikten ve v819l_button_armed=1 olduktan sonra:

1. Her PA0 basmasinda butonu yaklasik 0,7 saniye basili tut ve birak.
2. Bir sonraki basmadan once PA0'i en az 1 saniye birakilmis durumda tut.
3. PA0 birinci basma: BTS7960 etkinlesir, hedef hala 0,00; hareket yoktur.
4. Sonraki her PA0 basisi, yalniz needle_valve_lock=1 iken siradaki hedefi
   uygular:

   0,10 -> 0,25 -> 0,50 -> 0,75 -> 1,00 -> 0,00

5. Bir sonraki basistan once hedefe ulasilmasini ve lock=1 olmasini bekle.
6. Hareket sirasinda (lock=0) PA0'a basmak acil durdurmadir.
7. Son 0,00 hedefi tamamlaninca surucu kapanir ve v819l_auto_state=4 olur.

Yaklasik hedefler, ZERO=1023 icin:
- 0,10 : 935 ADC
- 0,25 : 803 ADC
- 0,50 : 584 ADC
- 0,75 : 364 ADC
- 1,00 : 145 ADC
- 0,00 : 1023 ADC

V819L AUTO STATE
----------------
- 0 : acilis beklemesi
- 1 : otomatik homing aktif
- 2 : ZERO basarili, surucu kapali, komuta hazir
- 3 : PA0 komut testi aktif
- 4 : komut testi PASS, surucu kapali
- 5 : operator ABORT, surucu kapali
- 6 : FAULT, surucu kapali

NOT
---
NeedleValveAutoControl_SubmitCommand(float) gelecekte dogrulanmis bir komut
kaynagina baglanmak icin hazirdir. Bu V8.19L paketinde GNC fiziksel motora
baglanmamistir; canli ana basinc, guncel kutle ve olculmus vana-acikligi ->
itki haritasi tamamlanmadan bu baglanti etkinlestirilmemelidir.
