# V54 — 4 Turluk İğne Vana + UART Uçuş Tanısı

V54, V53'ün kesintisiz Lidar + barometre yedekli düşey füzyonunu, V49'un
fiziksel Full-State ESKF işaretli-PD RCS kontrolünü, PE9 ayrılma kapısını ve
NRF alıcısını korur. İğne vana yazılım hareket sınırını 2.2 turdan 4.0 tura
çıkarır ve sistemi USART2 üzerinden gözlenebilir hâlde tutar.

## V54 iğne vana hareket değişikliği

- Kontrol döngüsü değişmedi: 1000 Hz / 1 ms.
- Maksimum hareket: 4.0 tur.
- Kalibrasyon: 195 ADC/tur.
- Toplam hedef strok: 780 ADC.
- Güvenli en düşük ZERO değeri 1012 ADC olduğunda tam açık alt hedefi
  232 ADC'dir.
- V53'ün fiziksel tur/s hedefi korundu:
  - açma normalize rampası: +0.9625/s,
  - kapatma normalize rampası: -0.8250/s.
- 4.0 turluk yazılım hareketi için nominal hedef süresi:
  - açma: yaklaşık 1.039 s,
  - kapatma: yaklaşık 1.212 s.
- Açma hedef hızı yaklaşık 3.85 tur/s (231 RPM), kapatma hedef hızı yaklaşık
  3.30 tur/s (198 RPM)'dir.
- 1000 Hz PD, PWM sınırları, +/-15 ADC aktif fren, 5 ms yön değiştirme ölü
  zamanı ve stall korumaları değiştirilmedi.

Bu süreler yazılım hedef rampasıdır. Gerçek mekanik süre; besleme gerilimi,
vana torku, redüktör boşluğu ve sürtünmeye bağlıdır. İlk doğrulama basınçsız ve
gaz hattı enerjisizken yapılmalıdır. Vana mekanik olarak 4 tur açılabiliyor ve
potansiyometre tam strokta yaklaşık 780 ADC değişebiliyor olmalıdır; mekanik
sona vurmadan önce bu durum elle doğrulanmalıdır.

## UART donanımı

- USART2 TX: PA2 / AF7
- USART2 RX: PA3 / AF7
- 115200 baud, 8N1, akış kontrolü yok
- TX: DMA1 Stream6 / Channel4
- Yayın: 10 Hz
- Lojik seviye: 3.3 V TTL

Ana bağlantı PA2 -> USB-TTL RXD ve ortak GND'dir. PA3 yalnızca gelecekteki RX
geliştirmesi için ayrılmıştır. V54'te UART komutları kapalıdır; seri porttan
hiçbir aktüatör sürülemez ve STOP kilidi temizlenemez.

## Non-blocking çalışma

Telemetri `App_Run()` içinden servis edilir fakat kendi 100 ms periyodunu
uygular. Metin yalnız DMA boşken hazırlanıp gönderilir. DMA meşgulse bekleme
yapılmaz; satır atlanır ve sayaç artırılır. `printf`, `delay`, UART polling ve
dinamik bellek kullanılmaz.

Bu nedenle UART yolu şunları bloke etmez:

- IMU ve RCS güvenlik servisi: 1 kHz
- Full-State ESKF ve RCS kontrol güncellemesi: 200 Hz
- Lidar DMA durum makinesi: sürekli 1 kHz servis
- Barometre ölçüm/füzyon yolu
- SDIO asenkron logger
- NRF alıcı ve STOP kilidi

## Çerçeve

Açılış sürüm satırı ve alan başlığından sonra veri şu biçimdedir:

```text
$TGY54,<seq>,<time_ms>,...,<uart_busy_skips>*<CRC16>\r\n
```

CRC-16/CCITT-FALSE; başlangıç `0xFFFF`, polinom `0x1021` kullanır. CRC, `$`
karakterinden `*` karakterinin hemen öncesine kadar hesaplanır. `seq` yalnız
başarılı DMA başlangıcından sonra artar; böylece kayıp/atlanan satırlar bulunur.

Alanların tam sırası ve ölçekleri `USART2_TELEMETRY_README.txt` dosyasındadır.

## Kontrol sırasında bakılacak temel alanlar

`vertical_source_mask` düşey yardımcı kaynağı gösterir:

- `0`: yardımcı kaynak yok; ana motor hesap yolu güvenli sıfıra geçer
- `1`: yalnız Lidar
- `2`: yalnız barometre
- `3`: Lidar + barometre

İtki sırasında Lidar geçici olarak okuyamazsa beklenen geçiş `3 -> 2`'dir.
Lidar görevi kapanmaz; `lidar_reads` tekrar artmaya ve yeni ölçüm aramaya devam
eder. Ölçüm yeniden güvenilir olduğunda kaynak maskesi tekrar `3` olabilir.

RCS için `roll_cdeg`, `pitch_cdeg`, açısal hızlar, işaretli PD zamanları,
istenen/uygulanan röle maskeleri, kalan atım süresi, cooldown, oto-fren maskesi,
durum, fault ve olay sayacı aynı satırdadır.

PE9 için ham ve 10 ms debounce edilmiş durum, preflight hazır/fault bilgisi ve
uçuş saati gönderilir. NRF bağlantısı, komut/flag, paket yaşı, RX sayacı ve
tek yönlü STOP kilidi ayrıca izlenir.

Ana motor tarafında 200 Hz düşey komut, hedef kuvvet, ESKF'den tahmin edilen
itki ve yanma kararı görünür. Ancak `main_physical_enabled=0` kalır: bu teslimat
ana motor otomatik fiziksel çıkışını açmaz. V49 fiziksel RCS yolu ve NRF alıcısı
ise açık kalır.

## Hızlı masa testi

1. Basınç/gaz ve aktüatör güçlerini güvenli biçimde ayırın.
2. PA2'yi 3.3 V USB-TTL RXD'ye, GND'yi GND'ye bağlayın.
3. Terminali 115200-8N1 açın.
4. V54 banner, tek alan başlığı ve `$TGY54` satırlarını doğrulayın.
5. `seq` artarken CRC, `uart_dma_errors=0` ve `uart_busy_skips` durumunu izleyin.
6. Lidar görüşünü kontrollü biçimde bozup `lidar_reads` sayacının durmadığını,
   maskenin barometre sağlıklıysa `2` olduğunu doğrulayın.
7. PE9 testini yalnız inert/enerjisiz aktüatörlerle yapın; debounce ve uçuş
   durumunu UART'tan izleyin.

Hazır izleme aracı:

```text
python monitor_uart_v54.py --port COM5
python monitor_uart_v54.py --port /dev/ttyUSB0
python monitor_uart_v54.py uart_capture.txt
```

Seri port kipinde `pyserial` gerekir. Araç CRC ve alan sayısını doğrular; bozuk
satırları reddeder ve kaynak maskesi/RCS/ana motor/NRF özetini gösterir.

## Derleme

Bu ortamda ARM hedef derleyicisi bulunmadığı için V54'e ait yeni HEX üretilmiş
değildir. Projeyi STM32CubeIDE'ye import edip Clean Project ve Build Project
yapın. Eski V53/V52/V51/V50/V49 HEX dosyasını V54 kaynaklarının çıktısı olarak kullanmayın.
