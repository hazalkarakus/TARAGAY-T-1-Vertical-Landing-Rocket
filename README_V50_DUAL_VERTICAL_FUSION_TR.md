# V50 — Kesintisiz Lidar + Barometre Yedekli Düşey Füzyon

Bu sürüm, ana itki sırasında Garmin LIDAR-Lite ölçümünün zaman zaman
geçersizleşebileceği kabul edilerek hazırlanmıştır. Lidar hiçbir uçuş fazında
durdurulmaz. Ana motorun 200 Hz düşey kontrol yolu ise artık yalnız canlı Lidar
ölçümüne bağlı değildir.

## Değişmeyen sensör frekansları

- IMU okuma ve ESKF nominal ilerletme: 1000 Hz
- Lidar DMA durum makinesi servisi: 1000 Hz
- Lidar hedef ölçüm hızı: yaklaşık 200 Hz
- BMP585 barometre: 200 Hz
- Full-State ESKF ölçüm düzeltmesi: 200 Hz
- ESKF kovaryans ilerletmesi: 50 Hz
- Ana motor dış kontrol döngüsü: 200 Hz

Lidar servisi ana motor komutundan, `suicide_burn_active` durumundan ve sensör
geçerlilik kararından bağımsızdır. Hatalı örnek yalnız füzyonda reddedilir;
sonraki I2C/DMA ölçümü yine başlatılır.

## Başlangıç referansları

PE9 ayrılma olayından önce:

1. Lidar yere olan başlangıç mesafesini toplar.
2. BMP585 başlangıç atmosfer basıncını/bağıl irtifa referansını toplar.
3. IMU durağan bias başlangıcını tamamlar.
4. Full-State ESKF koordinat başlangıcını sıfırlar.

V50 preflight kapısı hem güncel Lidar referansını hem güncel barometre
referansını zorunlu tutar. Böylece uçuşa tek bir düşey ölçüm kaynağıyla
başlanmaz.

## Uçuşta sensör politikası

Kontrolcü ham sensörleri doğrudan ortalamaz; Full-State ESKF durumunu tüketir.
ESKF her Lidar ve barometre örneği için ayrı tazelik, fiziksel aralık ve
innovation kapıları uygular.

| Kaynak maskesi | Durum | Ana motor hesabı |
|---:|---|---|
| `3` | Lidar + barometre güncel | Devam eder, normal |
| `1` | Yalnız Lidar güncel | Devam eder, degraded |
| `2` | Yalnız barometre güncel | Devam eder, degraded |
| `0` | İkisi de kullanılamıyor | Komut sıfır, integral temizlenir |

İtki Lidar ölçümünü bozduğunda beklenen geçiş `3 -> 2` olur. ESKF'nin
barometre düzeltmesi ve IMU ilerletmesi devam ettiği için düşey kontrol
hesabının yalnız `lidar_fresh=0` nedeniyle kapanmasına artık izin verilmez.

## Yüksekliğin devam ettirilmesi

AGL yüksekliği:

```text
height_agl = startup_lidar_reference + ESKF position_z
```

ile üretilir. Başlangıç Lidar referansı sabit yükseklik datumudur. Uçuşta Lidar
geçici olarak kaybolursa barometre, kendi başlangıç referansına göre ESKF
`position_z` durumunu düzeltmeye devam eder; IMU ise iki düzeltme arasında hızlı
hareketi ilerletir.

## Lidar geri geldiğinde

Lidar sürücüsü her zaman çalıştığı için veri tekrar geldiğinde:

- sensör aralığı kontrol edilir,
- son kabul edilen örneğe göre sıçrama kontrol edilir,
- dört tutarlı yeniden-kazanım örneği aranır,
- ESKF tahminiyle innovation kapısı uygulanır,
- konum ve hız düzeltme adımları sınırlandırılır.

Bu kontroller zaten Full-State ESKF içindedir. Tek bir hatalı Lidar örneğiyle
yükseklik aniden değiştirilmez.

## Ana motor kontrolü

200 Hz düşey kontrolcü aynı kalmıştır:

```text
H_burn = V_down^2 / (2 * 6.0)
F_target = 335 N + 55 * speed_error + integral_support
Valve_Cmd = clamp(F_target, 0, 460 N) / 460 N
```

Tank basıncı ölçülmez. Teslim edilen itki ESKF dünya-Z ivmesinden tahmin edilir
ve integral terimi eksik yavaşlamayı en fazla 90 N destekle telafi eder.

## Live Expressions

`generated_fc_status` içinde yeni alanlar:

- `vertical_sensor_source_mask`
- `vertical_sensor_degraded`
- `lidar_aiding_active`
- `baro_aiding_active`
- `vertical_sensor_transition_count`
- `vertical_dual_source_count`
- `vertical_lidar_only_count`
- `vertical_baro_only_count`
- `vertical_no_aiding_count`

Lidar hattının fiziksel nedenini ayırmak için ayrıca:

- `lidar_timeout_count`
- `lidar_dma_error_count`
- `lidar_last_i2c_error_code`
- `lidar_distance_valid`
- `lidar_raw_distance_cm`
- `full_eskf_lidar_fresh`
- `full_eskf_baro_fresh`

izlenmelidir.

## Fiziksel çıkış durumu

Ana motor/iğne vana uçuş çıkışı hâlâ compute-only'dir:

```c
APP_GENERATED_FC_PHYSICAL_OUTPUT_ENABLED = 0
APP_GNC_NEEDLE_PHYSICAL_ENABLED_PORT_READY = 0
```

Bu V50 değişikliği sensör yedekliliğini ve ana motor hesap yolunu düzeltir;
iğne vanayı otomatik uçuş komutuna bağlamaz. V49'daki Full-State ESKF tabanlı
dört valfli fiziksel RCS, PE9 ve NRF STOP güvenliği korunmuştur.

Basınçlı test veya fiziksel ana motor etkinleştirmesinden önce Lidar kaybı,
barometre kaybı ve iki sensörün birlikte kaybı HIL kayıtlarıyla doğrulanmalıdır.
