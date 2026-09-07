TARAGAY-T1 V8.19D - BMP585 STANDBY + RUNTIME TESHIS DUZELTMESI
================================================================

V8.19C TEST SONUCU
------------------

0x1000F300 blogu gecerli bir snapshot verdi; ancak BMP585 runtime servisi
baslamadi:

- BARO task 200 Hz calisti.
- Driver service / INT_STATUS poll / DRDY / olcum sayaclari 0 kaldi.
- CHIP_ID = 0x51, STATUS = 0x02.
- INT_SOURCE = 0x01, ODR_CONFIG = 0xA9, OSR_CONFIG = 0x59.
- INT_CONFIG readback = 0x00.
- INT_CONFIG dogrulamasi BMP585_Configure() fonksiyonunu basarisiz dondurdu.

Bu nedenle V8.19C kaydindan 100 Hz ile 50 Hz arasinda karar verilemez.


V8.19D YAKLASIMI
----------------

1) Kesin STANDBY gecisi

   V8.19C, STANDBY icin ODR_CONFIG registerina 0x00 yaziyordu. V8.19D,
   deep standby'i acikca devre disi birakan degeri kullanir:

       ODR_CONFIG = 0x80
       deep_dis    = 1
       pwr_mode    = 0 (STANDBY)

   3 ms sonra ODR_CONFIG tekrar okunur. deep_dis=1 ve pwr_mode=STANDBY
   dogrulanmadan OSR/IIR/interrupt ayarlarina gecilmez.

2) INT_CONFIG artik polled veri yolunu durdurmaz

   Bu firmware fiziksel BMP585 INT pinini kullanmiyor. 200 Hz task,
   INT_SOURCE ile acilan DRDY durumunu INT_STATUS registerindan poll ediyor.
   INT_CONFIG ise fiziksel pinin pulse/latched, polarity, drive ve enable
   ayarlarini belirliyor.

   V8.19D yine Bosch sirasi ile INT_CONFIG yazmayi dener:

       INT_SOURCE = 0x00
       INT_STATUS oku / temizle
       INT_CONFIG yaz
       INT_SOURCE.DRDY = 1

   Fakat INT_CONFIG readback alt nibble'i 0x0A olmazsa bu durum yalnizca
   teshis olarak tutulur; barometre runtime servisi engellenmez. Zorunlu
   veri-yolu kontrolleri (ODR, OSR, DRDY kaynagi, NVM durumu) aynen korunur.

3) Ayri init kayitlari

   INT_CONFIG icin su uc deger ayri saklanir:

       bmp585_int_config_before_reg
       bmp585_int_config_written_reg
       bmp585_int_config_reg          (readback)

   Boylece registerin yazmadan onceki, yazilmak istenen ve gercek okunan
   degerleri tek testte karsilastirilabilir.


SABIT BELLEK BLOGU
------------------

Adres:

    0x1000F300..0x1000F3FF

Ilk uc 32-bit word:

    [0] = 0xB58519D1
    [1] = 0x0008190D
    [2] = 0x00000100

CubeIDE byte gorunumunde ilk satir:

    C11985B5 0D190800 00010000

Word [38]:

    bits 7:0   = son INT_STATUS
    bits 15:8  = STANDBY ODR_CONFIG readback (beklenen 0x80)

Word [58]:

    bits 7:0   = INT_CONFIG yazma oncesi
    bits 15:8  = INT_CONFIG yazilmak istenen
    bits 23:16 = INT_CONFIG anlik readback
    bits 27:24 = konfigurasyon asamasi (8 = runtime hazir)
    bit 28     = STANDBY 0x80 dogrulandi
    bit 29     = INT_CONFIG alt nibble 0x0A dogrulandi (yalniz teshis)
    bit 30     = DRDY kaynagi dogrulandi
    bit 31     = zorunlu runtime konfigurasyonu dogrulandi

INT_CONFIG tamamen dogrulanirsa tipik [58]:

    0xF83A3A35

INT_CONFIG readback yine 0x00 kalir fakat zorunlu runtime yolu acilirsa
ust durum byte'i 0xD8 olur. Bu durumda da service/poll/DRDY sayaclari
ilerlemelidir.

Son word:

    [63] = 0x56313944 = 'V19D'


ILK TEST
--------

1) Project -> Clean
2) Project -> Build Project
3) V8.19D .elf dosyasini karta Debug ile yukle
4) Resume ile en az 15 saniye calistir
5) Suspend
6) Memory view -> 0x1000F300
7) Bir kez Refresh
8) 0x1000F300..0x1000F3FF ekran goruntusunu al

Gecerli snapshot kosullari:

    [3] == [59]
    [3] cift sayi
    [61] == bitwise NOT [60]
    [60] checksum dogru

Beklenen temel alanlar:

    [16] BARO task rate x10         = yaklasik 2000
    [17] driver service rate x10    = yaklasik 2000
    [18] INT_STATUS poll rate x10   = yaklasik 2000
    [20] DRDY rate x10              = hedef 900..1110
    [21] measurement rate x10       = hedef 900..1110
    [29] son DRDY araligi           = hedef yaklasik 10000 us
    [32] CHIP_ID                    = 0x51
    [33] INT_SOURCE                 = 0x01
    [34] ODR_CONFIG                 = 0xA9
    [35] OSR_CONFIG                 = 0x59
    [36] OSR_EFF                    = 0x99
    [40] register read error        = 0
    [50] qualification rate x10     = hedef 900..1110
    [55] zorunlu runtime config_ok  = 1
    [56] drdy_source_ok             = 1
    [57] zorunlu register readback  = 1

Barometre kabul araligi degistirilmedi: 90..111 Hz.
MATLAB vana kontrolu, ESKF, RCS ve fiziksel cikislar degistirilmedi.
