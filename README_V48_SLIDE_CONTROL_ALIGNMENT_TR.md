# V48 - Görsel Kontrol Algoritması Uyarlaması

Bu revizyon, PE9 ayrılma tetiğine ek olarak paylaşılan yedi görseldeki kontrol
mimarisini kaynak koda taşır.

## Uygulanan davranışlar

- PE9 ayrılma konnektörü: dahili pull-up, bağlıyken LOW, ayrılınca HIGH,
  10 ms debounce, en az 3 s sensör/ESKF/LIDAR/iğne hazırlığı ve tek seferlik T0.
- NRF24 alıcısı açık ve önceki çalışan küçük-kart yolu korunmuştur: SPI3,
  kanal 76, 4 bayt paket, sürekli RemoteControl servisi ve 200 Hz tanılama.
- ESKF: 1 kHz nominal yayılım, 200 Hz ölçüm düzeltmesi. İvme normundaki hızlı
  değişim titreşim/şok metriği olarak EMA ile izlenir; yerçekimi ölçümünün R
  değeri dinamik olarak 1x..25x büyütülür.
- RCS: açı çıktısı 200 Hz. Darbe süresi
  `Kp*|açı hatası| + Kd*|açısal hız hatası|`; 20 ms altı reddedilir,
  60 ms üstü kırpılır. Karşı valf frenlemesi 20 ms, tekrar ateşleme kilidi
  200 ms'dir. Yalnız ilk dört fiziksel RCS valfi kullanılır.
- Düşey dış döngü: 200 Hz, `H = V^2/(2*a_max)` ateşleme yüksekliği,
  335 N feedforward ve ESKF'den türetilen gerçek itki ile PI kayıp telafisi.
  Basınç sensörü kullanılmaz; başlangıç model basıncı 300 bar olarak kalır.
- İğne vana: TIM7 ile 1000 Hz, ADC EMA filtresi, PD PWM takibi, 2,2 tur
  mekanik/yazılımsal sınır ve +/-15 ADC aktif kısa-devre fren bölgesi.

## Güvenlik ve devreye alma notu

RCS'nin mevcut PE9-gated fiziksel yolu korunmuştur. Yeni 200 Hz düşey komut
`generated_fc_status.vertical_200hz_valve_cmd` alanında üretilir; otomatik ana
vana fiziksel yönlendirmesi commissioning güvenliği nedeniyle varsayılan olarak
kapalıdır. HIL testinde motor yönü, potansiyometre işareti, 2,2 tur son noktası,
335 N/460 N itki kalibrasyonu ve röle gecikmeleri doğrulanmadan bu bayrak
açılmamalıdır.

335 N, görselde istenen feedforward değeridir. Projedeki 27,5 kg model kütlesi
ile yalnızca ağırlığı dengeleyen teorik değer yaklaşık 269,7 N'dir; bu nedenle
335 N değeri uçuş öncesi statik itki testiyle doğrulanmalıdır.
