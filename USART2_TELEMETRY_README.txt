UYARI: BU DOSYANIN DEVAMI V54 ARŞİV BİLGİSİDİR VE V55 İÇİN GEÇERLİ DEĞİLDİR.
V55 pin/polarite/alan bilgisi için USART2_TELEMETRY_V55_README.txt dosyasını
ve monitor_uart_v55.py aracını kullanın. V55 çerçevesi $TGY55 ile başlar.

TGY V54 USART2 UÇUŞ TANI TELEMETRİSİ (ARŞİV)
====================================

1. DONANIM BAĞLANTISI

USB-TTL dönüştürücü 3.3 V lojik seviyeli olmalıdır.

  STM32 PA2 / USART2_TX  --->  USB-TTL RXD
  STM32 GND              --->  USB-TTL GND

İsteğe bağlı, yalnızca gelecekteki RX geliştirmesi için:

  STM32 PA3 / USART2_RX  <---  USB-TTL TXD

Uçuş bilgisayarı kendi beslemesinden çalışırken USB-TTL kartının 5 V veya
3.3 V besleme pinini bağlamayın. Ortak GND zorunludur. RS-232 seviyesini
doğrudan bağlamayın; bu pinler 3.3 V TTL'dir.

2. SERİ PORT AYARI

  USART       : USART2
  TX          : PA2 / AF7
  RX          : PA3 / AF7
  Hız         : 115200 baud
  Çerçeve     : 8 veri biti, parity yok, 1 stop biti (8N1)
  Akış kontrol: yok
  TX yöntemi  : DMA1 Stream6 / Channel4, normal mode
  Yayın hızı  : 10 Hz

DMA gönderimi non-blocking'dir. DMA meşgulse yeni satır bekletilmez; atlanır
ve uart_busy_skips sayacı artırılır. Böylece 1 kHz zamanlayıcı, 200 Hz ESKF,
RCS ve sensör görevleri UART yüzünden durmaz.

3. GÜVENLİK SINIRI

PA3 donanımsal olarak RX kipindedir fakat V54 içinde UART komut çözücüsü
yoktur. UART üzerinden RCS, ana motor, iğne vana, NRF STOP kilidi veya PE9
durumu değiştirilemez. Bu sürümde UART yalnızca tanı verisi çıkarır.

4. VERİ AKIŞI

Açılışta sırasıyla:

  1) '# TGY UART FLIGHT DIAGNOSTICS V54 ...' sürüm satırı
  2) '# frame,seq,...,crc16_ccitt' alan adları
  3) Her 100 ms'de bir '$TGY54,...*ABCD' veri satırı

Her başarılı veri satırında seq bir artar. Atlanan veya kaybolan çerçeveler
seq sıçramasından görülür. Yıldızdan sonraki dört hex karakter CRC-16/CCITT-
FALSE değeridir: başlangıç 0xFFFF, polinom 0x1021. CRC, satırın '$' işaretinden
yıldızdan hemen önceki son karaktere kadar olan kısmı üzerinde hesaplanır.

5. ÖNEMLİ ALANLAR

  vertical_source_mask:
    0 = geçerli düşey yardımcı kaynak yok
    1 = yalnız Lidar
    2 = yalnız barometre
    3 = Lidar + barometre

  vertical_degraded:
    0 = iki kaynak birlikte
    1 = tek kaynakla yedekli/degraded çalışma veya kaynak yok

  pe9_raw_open / pe9_debounced_open:
    0 = ayrılma konnektörü bağlı (PE9 LOW)
    1 = ayrılmış/açık (PE9 HIGH)

  rcs_req_mask / rcs_applied_mask bitleri:
    bit0 = PE7  roll pozitif hata valfi
    bit1 = PE11 roll negatif hata valfi
    bit2 = PE15 pitch pozitif hata valfi
    bit3 = PB15 pitch negatif hata valfi

  rcs_roll_pd_us / rcs_pitch_pd_us:
    İşaretli PD zaman çıktısının mikrosaniye ölçekli gösterimidir.
    Örnek: 43000 = +43.000 ms, -25000 = -25.000 ms.

  rcs_autodamp_mask:
    bit0 = roll oto-frenleme, bit1 = pitch oto-frenleme

  main_cmd_x10000:
    200 Hz ana-motor dış döngüsünün 0..1 komut hesabı x10000.
    main_physical_enabled bu V54 kaynak ağacında 0'dır; hesap UART'ta görünür
    fakat otomatik ana vana fiziksel olarak sürülmez.

  *_mm      = metre x1000
  *_mmps    = metre/saniye x1000
  *_cdeg    = derece x100
  *_mdps    = derece/saniye x1000
  *_cN      = Newton x100
  *_x10000  = normalize değer x10000
  cpu_x100  = CPU yüzde değeri x100

Lidar sürekli servis edilmeye devam eder. lidar_reads artmaya devam etmeli;
lidar_valid veya lidar_fresh geçici olarak 0 olduğunda, baro_fresh=1 ve
vertical_source_mask=2 ise düşey kestirim barometre yardımıyla devam eder.
ESKF'nin bağımsız kabul/red sayaçları baro_updates, lidar_updates,
baro_rejects ve lidar_rejects alanlarında izlenir.

6. CUBEIDE İLE DERLEME

  1) Klasörü STM32CubeIDE workspace'ine import edin.
  2) Project -> Clean Project seçin.
  3) Build Project ile ELF/HEX üretin.
  4) Kartı programlayın.

Bu teslimatta yeni V54 kaynakları için hazır HEX yoktur. Eski V53/V52/V51/V50/V49 HEX'i
bu klasörün koduymuş gibi kullanmayın. UART taşıması Core/Src/usart.c içinde
doğrudan register + DMA ile uygulanmıştır. İlk testten önce tgy.ioc üzerinden
otomatik kod üretimi yapmayın; mevcut kaynakları doğrudan derleyin.
