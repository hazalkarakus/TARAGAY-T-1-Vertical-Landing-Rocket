# V48 PE9 Ayrılma Konnektörü ve Pre-Flight Kilidi

## Elektrik bağlantısı

- STM32 pini: `PE9`
- Giriş tipi: dijital giriş, dahili `PULL-UP`
- Konnektör takılı: PE9, konnektör üzerinden `GND`'ye kısa devredir → `LOW`
- Konnektör ayrılmış/açık: dahili pull-up nedeniyle `HIGH`
- PE9'a 5 V veya 24 V kesinlikle uygulanmaz.

Bu bağlantıda ayrılma olayı `LOW -> HIGH` geçişidir. Yazılım önce PE9'un
en az 100 ms boyunca gerçekten LOW olduğunu görmek zorundadır. Dolayısıyla
kart PE9 açıkken açılırsa uçuş algoritmaları kendiliğinden başlamaz.

## Durum makinesi

1. `BOOT_CALIBRATION`
   - En az 3 saniye beklenir.
   - IMU kalibrasyonu, attitude estimator, ESKF origin/vertical state,
     LIDAR referansı ve iğne vana zero/homing sonucu doğrulanır.
   - RCS röleleri kapalı tutulur; üretilen Simulink komutları sıfırdır.
2. `WAIT_CONNECTOR`
   - Sistem PE9'un takılı/LOW durumunu henüz doğrulamadıysa burada bekler.
3. `WAIT_SEPARATION`
   - Bütün pre-flight koşulları hazırdır ve PE9 LOW'dur.
4. `FLIGHT_ACTIVE`
   - PE9 HIGH durumu 10 ms kesintisiz doğrulanınca tek yönlü olarak kilitlenir.
   - `separation_timestamp_ms` T=0 olur ve `flight_time_ms` sıfırdan sayar.
   - Fiziksel eski RCS yolu ve compute-only Simulink modeli ancak bundan sonra
     çalışabilir.
5. `EARLY_SEPARATION_FAULT`
   - Konnektör daha önce görülmüş, fakat kalibrasyon tamamlanmadan ayrılmışsa
     hata kilitlenir ve RCS kapalı kalır. Yeniden başlatma gerekir.

## Live Expressions

Tek yapı olarak aşağıdaki sembol eklenebilir:

`preflight_trigger_status`

Önemli alanlar:

- `state`
- `raw_open`
- `debounced_open`
- `connector_seen`
- `preflight_ready`
- `flight_active`
- `fault_latched`
- `separation_timestamp_ms`
- `flight_time_ms`
- `debounce_reject_count`

## Gönderilen görsellerle gerçek kodun karşılaştırması

Görseller genel kontrol mimarisini doğru anlatıyor, fakat aşağıdaki değerler
mevcut V48 koduyla aynı değildir:

- ESKF nominal yayılımı 1000 Hz, düzeltme ve çıktı yolu 200 Hz'dir.
- Entegre `Ucus_Bilgisayari` Simulink modeli 100 Hz'dir; 200 Hz değildir.
- Fiziksel iğne vana konum döngüsü 200 Hz'dir; 1000 Hz değildir.
- İğne valf toleransı ±3 ADC, yeniden bırakma eşiği 4 ADC'dir; ±15 değildir.
- Hareket sınırı 878 ADC / 195 ADC-tur ≈ 4.5 turdur; 2.2 tur değildir.
- Motor hedefteyken iki PWM sıfırlanır. Mevcut BTS7960 sürüşünde yazılımla
  aktif kısa-devre frenleme yapıldığı iddia edilemez.
- 27.5 kg kütlede hover feedforward yaklaşık `27.5 × 9.80665 = 269.7 N`'dur;
  335 N değildir.
- Uçuşta basınç ölçülmez. 300 bar model başlangıç değeri ve ESKF ivmesinden
  itki kestirimi kullanılır; gerçek adyabatik tank basınç modeli/choked-flow
  boğaz modeli bu firmware içinde uygulanmış değildir.
- RCS süreleri tek bir `20..60 ms` kırpması değildir. Mevcut legacy koruma
  değerleri minimum OFF 20 ms, reversal dead-time 30 ms, minimum correction
  30 ms ve maksimum brake 200 ms'dir; Simulink modelinin kendi 10 ms tabanlı
  durum/pulse sayaçları ayrıca bulunur.
- ESKF'de adaptif gravity correction/gating vardır; fakat görselde anlatılan
  ayrı bir “manifold şok dedektörü R matrisini otomatik değiştirir” bloğu
  birebir uygulanmış değildir.

Bu nedenle sunumda “uygulanmıştır” denilecekse yukarıdaki gerçek değerler
kullanılmalıdır.
