# TGY V8.19M V55 — PE9 uçuş kilitli sürüm

Bu sürümde konnektör takılıyken ana motor iğne vanası ve dört RCS çıkışı
fiziksel olarak kapalı tutulur. PA0 dayanım/çevrim testi uçuş yazılımından
çıkarılmıştır. USART2 yalnızca telemetri gönderir; UART üzerinden aktüatör
komutu kabul edilmez.

> Bu kaynak kod statik/host kontrollerinden geçirilmiştir; basınçlı veya ateşli
> sistem üzerinde doğrulanmış, uçuşa sertifikalı bir sürüm değildir. İlk testte
> gaz/basınç hattı, ana motor ve solenoid güçleri fiziksel olarak ayrılmalıdır.

## PE9 bağlantısı ve kilit mantığı

- Konnektör takılı: PE9, konnektör üzerinden GND'ye bağlı ve LOW olmalıdır.
- Konnektör ayrılmış: PE9 açık kalır; dahili pull-up pini HIGH yapar.
- Yazılımdaki `pe9_raw_open` fiziksel seviyeyi değil, **ayrılmış/açık** anlamını
  gösterir. Takılıyken `0`, ayrılınca `1` beklenir.
- Sistem önce en az 100 ms boyunca takılı durumu görmeden ayrılmayı kabul etmez.
- Ayrılma 10 ms debounce sonrasında ve yalnızca preflight hazırsa uçuşu başlatır.
- Kart PE9 açıkken açılırsa aktüatörler devreye girmez.
- Hazırlık tamamlanmadan PE9 ayrılırsa durum `4` olur, hata kilitlenir ve bütün
  aktüatörler kapalı kalır. Gücü kesip sorunu düzeltmeden yeniden denenmemelidir.

Preflight durumları:

| Değer | Anlam |
|---:|---|
| 0 | Kalibrasyon / sensör ve iğne sıfırı bekleniyor |
| 1 | Takılı konnektör henüz doğrulanmadı |
| 2 | Sistem hazır, ayrılma bekleniyor |
| 3 | Geçerli ayrılma görüldü, uçuş aktif |
| 4 | Erken ayrılma hatası; aktüatörler kilitli |

## İğne vana / ana motor

- Potansiyometre: PC1 / ADC1_IN11.
- BTS7960: PB0 TIM3_CH3 LPWM (açma), PB1 TIM3_CH4 RPWM (kapama), PC4/PC5 enable.
- Hareket sınırı dört turdur: yaklaşık 780 ADC sayımı.
- Güç verilmeden önce vana mekanik olarak tamamen kapalı konuma getirilmelidir.
  Kapalı okuma 1012–1023 aralığında 250 ms kararlıysa ZERO, motor sürülmeden
  yakalanır.
- Konnektör takılıyken `needle_enabled=0`, `needle_rpwm=0` ve `needle_lpwm=0`
  kalmak zorundadır.
- Ayrılma sonrasında ESKF dikey durumu ve ana kontrol çıktısı geçerliyse sürücü
  etkinleşebilir. Normal masa koşulunda iniş/yanma koşulu oluşmadığından komutun
  yine sıfır kalması normaldir.
- Yazılımsal dört tur hedef rampası yaklaşık 1.04 s açma ve 1.21 s kapama
  süresine karşılık gelir. Gerçek süre yük, besleme ve mekanik sürtünmeye göre
  daha uzun olabilir.
- ADC geri beslemesi yoksa, yön tersse, 800 ms ilerleme görülmezse veya dört tur
  sınırı aşılırsa motor durur ve hata üretir.

Önceki takılma problemini azaltmak için ADC girişinde üç örnek medyan + EMA,
yakın hedefte darbeli sürme, yön değiştirmede 5 ms ölü zaman ve işaretli PD
sönümleme kullanılır. D terimi artık hedefe yaklaşırken motoru hızlandırmaz.

## RCS solenoidleri

Röle girişleri active-low’dur: HIGH kapalı/güvenli, LOW açık. Açılışta pinler
çıkış moduna alınmadan önce HIGH yazılır.

| Eksen | Röle girişi | STM32 pini | UART maskesi |
|---|---|---|---:|
| X+ / roll pozitif hata düzeltme | IN1 | PB15 | 1 |
| X- / roll negatif hata düzeltme | IN2 | PE15 | 2 |
| Y+ / pitch pozitif hata düzeltme | IN3 | PE11 | 4 |
| Y- / pitch negatif hata düzeltme | IN4 | PE7 | 8 |

RCS, 200 Hz ESKF açısı ve cayro hızını kullanır:

`atım_ms = 2.50 × açı_hatası_deg + 3.55 × açısal_hız_hatası_deg_s`

Mutlak sonuç 20 ms altındaysa vana hiç açılmaz; 20–60 ms arası uygulanır,
60 ms üzerinde 60 ms’ye kırpılır. Her atımdan sonra o eksen 100 ms soğumaya
girer. Bu nedenle yalnızca 3 derece sabit eğmek atım üretmez; hız yaklaşık
sıfırken eşik yaklaşık 8 derecedir. `rcs_req_mask` kontrol talebini,
`rcs_applied_mask` gerçekten pine uygulanan maskeyi gösterir.

## LIDAR ve barometre

LIDAR servis döngüsü hiç durmaz ve yeniden okumayı sürdürür. ESKF LIDAR ve
barometreyi ayrı tazelik/innovation kontrollerinden geçirir. Uçuşta kaynaklardan
biri geçici olarak bozulursa diğeriyle dikey kestirim devam edebilir. Başlangıçta
zemin referansı için hem LIDAR hem barometre referansının hazır olması gerekir.

## UART bağlantısı

- USART2 TX: PA2
- USART2 RX: PA3, fakat komut alımı kapalıdır
- 115200 baud, 8N1, 10 Hz, DMA TX
- Çerçeve: `$TGY55,...*CRC16`

Bilgisayarda:

```text
py -m pip install pyserial
py monitor_uart_v55.py --port COM18
```

Linux örneği:

```text
python3 monitor_uart_v55.py --port /dev/ttyUSB0
```

## Zorunlu basınçsız ilk test

1. Solenoid beslemesini, ana motor ateşleme hattını ve gaz/basınç hattını ayırın.
2. İğne vanayı mekanik olarak kapatın; PE9’u GND'ye bağlayan konnektörü takın.
3. Kartı açın ve UART’ta `pe9_raw_open=0`, `connector_seen=1` görünmesini bekleyin.
4. Takılı durumda motor PWM’lerinin ve `rcs_applied_mask` değerinin sürekli sıfır
   olduğunu doğrulayın. Herhangi biri sıfır değilse besleme vermeyin.
5. `preflight_state=2`, `preflight_ready=1`, `needle_zero_valid=1` olmadan PE9’u
   ayırmayın.
6. PE9’u ayırınca `pe9_raw_open=1`, ardından `flight_active=1` ve
   `actuator_authorized=1` beklenir.
7. Solenoid güçleri hâlâ ayrıyken kartı statik olarak yaklaşık 9–12 derece
   yatırın. UART’ta `rcs_req_mask` ve kısa süreli `rcs_applied_mask` değişimini;
   multimetre/lojik analizörde ilgili active-low pinin 20–60 ms LOW olmasını
   kontrol edin.
8. Pin yönleri doğru, karşılıklı iki vana hiçbir zaman birlikte açılmıyor ve
   UART hata sayaçları artmıyorsa ancak bundan sonra ayrı bir düşük enerjili
   aktüatör testi planlayın.

Kritik beklenen takılı durum satırı: `flight=0`, `PE9open=0`, `seen=1`,
`auth=0`, `RCS=0`, `PWM=0/0`. PE9 elektrik seviyesi bu tanıma uymuyorsa yazılımı
değiştirmek yerine önce kablo/polariteyi doğrulayın.

## Derleme

Projeyi STM32CubeIDE’ye mevcut proje olarak içe aktarın ve `Build Project`
çalıştırın. Beklenen çıktı:

`Debug/TGY_V8_19M_V55_FLIGHT_INTERLOCK.hex`

Bu teslim ortamında `arm-none-eabi-gcc` bulunmadığı için HEX üretilmemiştir.
Kaynak sözdizimi, V55 güvenlik kuralları, UART alan/CRC eşleşmesi ve RCS host
testleri doğrulanmıştır. Gerçek karta yüklemeden önce CubeIDE derlemesinin sıfır
hata ile tamamlanması zorunludur.
