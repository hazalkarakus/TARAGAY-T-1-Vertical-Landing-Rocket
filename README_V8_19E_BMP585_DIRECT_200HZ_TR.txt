TARAGAY-T1 V8.19E - BMP585 DRDY'SIZ GERCEK 200 HZ DIRECT READ
================================================================

AMAC
----
BMP585 barometre cikisini, INT_STATUS/DRDY davranisina bagli kalmadan,
ESKF ve filtre katmanina saniyede yaklasik 200 taze fiziksel ornek verecek
sekilde calistirmak.

NEDEN SENSOR 218.5 HZ, HOST 200 HZ?
-----------------------------------
STM32 barometre task'i her 5000 us'de (200 Hz) calisir. BMP585 sensoru
218.5 Hz'de (yaklasik her 4577 us'de) yeni olcum tamamlar. Sensor hosttan
daha hizli oldugu icin normal kosulda her host okumasindan once en az bir
yeni fiziksel olcum hazir olur.

Bosch NORMAL-mode zamanlama tablosunda P x4 / T x1 kombinasyonu en fazla
220 Hz destekler. Bu nedenle 218.5 Hz secimi gecerlidir ve 200 Hz icin
P x4 basinc oversampling'ini koruyan en yuksek-cozunurluklu marjli secimdir.

KONFIGURASYON
-------------
- Sensor ODR       : 218.5 Hz (code 0x01)
- Pressure OSR     : x4
- Temperature OSR  : x1
- Host read rate   : 200 Hz
- INT_SOURCE       : 0x00 (DRDY kaynagi kapali)
- Dahili IIR       : bypass (ust katman Butterworth kullanir)
- ODR_CONFIG       : 0x85
- OSR_CONFIG       : 0x50
- OSR_EFF          : 0x90

TAZE ORNEK KORUMASI
-------------------
Her 200 Hz task cagrisi 0x1D adresinden 6 byte sicaklik+basinc burst okur.
Yeni ham P/T cifti onceki kabul edilen cift ile ayniysa:
- yeni ornek sayilmaz,
- filtre ve ESKF'ye ikinci kez verilmez,
- bmp585_duplicate_raw_count artar.

Farkli ve gecerli bir P/T cifti geldiyse:
- bmp585_fresh_sample_count artar,
- ms5611_update_count artar,
- Barometer katmani filtre/irtifa hesabini bir kez calistirir.

FILTRE / KALIBRASYON
-------------------
- Butterworth katsayilari gercek 200 Hz ornekleme frekansina gore kurulur.
- Ground pressure calibration 400 taze ornek kullanir; onceki yaklasik
  2 saniyelik kalibrasyon suresi korunur.
- Qualification kabul araligi 190..205 Hz'dir.
- Barometre freshness siniri 20 ms'dir.

SABIT BELLEK BLOKLARI
---------------------
0x1000F200: genel sensor qualification (64 x uint32_t)
0x1000F300: BMP585 direct-read diagnostic (64 x uint32_t)

0x1000F300 alan haritasi:
[0]  magic                         0xB58519E1
[1]  version                       0x0008190E
[2]  byte count                    0x00000100
[3]  snapshot sequence begin
[4]  flags
[5]  BARO task total run count
[6]  driver service total count
[7]  direct 6-byte read attempts
[8]  direct read errors
[9]  duplicate raw P/T count
[10] fresh raw P/T count
[11] low-level accepted update count
[12] upper barometer update count
[13] upper source update count
[14] upper duplicate skip count
[15] legacy DRDY count (V8.19E'de 0)
[16] task rate x10                 hedef 1950..2050
[17] service rate x10              hedef 1950..2050
[18] direct-read rate x10          hedef 1950..2050
[19] duplicate rate x10            ideal 0
[20] fresh-sample rate x10         hedef 1900..2050
[21] upper accepted rate x10        hedef 1900..2050
[22] task last execution us
[23] task max execution us
[24] task overrun count             hedef 0
[25] task deadline miss count       hedef 0
[26] last service interval us       hedef ~5000
[27] min service interval us
[28] max service interval us
[29] last fresh interval us         hedef ~5000
[30] min fresh interval us
[31] max fresh interval us
[32] CHIP_ID                        hedef 0x51
[33] INT_SOURCE                     hedef 0x00
[34] ODR_CONFIG                     hedef 0x85
[35] OSR_CONFIG                     hedef 0x50
[36] OSR_EFF                        hedef 0x90
[37] STATUS
[38] low byte INT_STATUS, next byte standby readback
[39] periodic register snapshots
[40] register snapshot errors       hedef 0
[41] register mismatches            hedef 0
[42] last register snapshot time us
[43] communication errors           hedef 0
[44] invalid ADC count              hedef 0
[45] fresh ADC read count
[46] upper source update count
[47] upper accepted update count
[48] abs(task-service)
[49] abs(fresh-low-level update)
[50] qualification baro rate x10
[51] snapshot time us
[52] last upper sample interval us
[53] min upper sample interval us
[54] max upper sample interval us
[55] config_ok                      hedef 1
[56] direct_read_mode_ok            hedef 1
[57] register_readback_ok           hedef 1
[58] init/readback packed flags
[59] snapshot sequence end
[60] checksum
[61] inverse checksum
[62] 0x4241524F ('BARO')
[63] 0x56313945 ('V19E')

ILK BENCH TESTI
---------------
1) Motor guc kaynagi KAPALI kalsin.
2) CubeIDE: Project -> Clean, sonra Build Project.
3) V8.19E ELF'i karta Debug ile yukle.
4) Resume ile en az 15 saniye calistir.
5) Suspend yap.
6) Memory ekraninda 0x1000F300 adresini acip Refresh yap.
7) 0x1000F300..0x1000F3FF ekran goruntusunu gonder.

CubeIDE ilk satiri little-endian gorunumde soyle baslamalidir:

    E11985B5 0E190800 00010000

Gercek karar icin task/service/direct-read/fresh rate, duplicate sayisi,
register readback, overrun ve checksum birlikte degerlendirilmelidir.
