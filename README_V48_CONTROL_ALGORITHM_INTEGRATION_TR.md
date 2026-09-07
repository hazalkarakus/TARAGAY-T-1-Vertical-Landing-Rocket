# V48 — Uçuş Bilgisayarı Kontrol Algoritması Entegrasyonu

Bu sürüm, `Ucus_Bilgisayari` Simulink modelinin R2025b ile üretilmiş
`grt.tlc` C kodunu küçük kart STM32F407 projesine ekler. Model sürümü 1.214,
üretilme zamanı 21 Ağustos 2026 ve doğal örnekleme süresi 10 ms'dir (100 Hz).

## Bu sürümde etkin olan davranış

- Model 100 Hz'de gerçek sensör/ESKF durumlarıyla çalışır.
- `Ana itki`, `V1`, `V3`, `V5`, `V7` sonuçları
  `generated_fc_status` içinde izlenebilir.
- Yeni algoritmanın bütün çıkışları **compute-only** durumundadır.
- Yeni model ana iğne motoruna veya RCS rölelerine bağlanmamıştır.
- Önceki V26/V47 davranışları korunur: iğne valfi 10 çevrim dayanım modu ve
  eski attitude-to-RCS yolu bu entegrasyondan bağımsızdır.

`APP_GENERATED_FC_PHYSICAL_OUTPUT_ENABLED` değeri sıfırdır. Değer sıfırdan
farklı yapılırsa bilinçli olarak derleme hatası üretilir; fiziksel bağlantı,
V1/V3/V5/V7 eksen-yön eşlemesi ve ana valf kabul testi tamamlanmadan açılamaz.

## Giriş eşlemesi

| Model girişi | STM32 kaynağı | Birim/not |
|---|---|---|
| `Pitch` | klasik attitude estimator pitch | derece |
| `Yaw` | klasik attitude estimator yaw | derece |
| `XPOS`, `yPOS` | Full-State ESKF X/Y | metre |
| `vx`, `Vy` | Full-State ESKF X/Y hız | m/s |
| `zpos` | `0.4013 + LIDAR AGL` | CG yüksekliği, metre |
| `zvel` | Full-State ESKF Z hızı | m/s, yukarı pozitif |
| `ZposIMU` | `0.4013 + ESKF position_z` | metre; üretilen C'de şu an kullanılmıyor |
| `m_guncel` | model sabiti | 27.5 kg |
| `P_main_bar` | başlangıç/model sabiti | 300 bar; uçuşta basınç ölçümü yok |
| `Gercek_Itki_N` | `m * (a_z + g)` ESKF kestirimi | 0..600 N sınırlandırılmış |
| `pitch_rate` | filtreli gyro Y | rad/s |
| `yaw_rate` | filtreli gyro Z | rad/s |

Basınç sensörü bulunmadığı için model 300 bar başlangıç değeriyle çalışır.
İtki geri beslemesi sabit sıfır yapılmamıştır; aksi hâlde model yaklaşık 0.5 s
içinde yanlış “itki yetersizliği” kilidi oluşturur. Bunun yerine düşey ESKF
ivmesinden itki kestirimi verilerek valf komutunun gerçek araç tepkisine göre
uyarlanabilmesi sağlanmıştır. Bu kestirim uçuş dışında zemin tepki kuvvetini de
itki gibi görebileceğinden fiziksel aktivasyondan önce HIL/uçuş-benzeri test
zorunludur.

## Zamanlama ve geçerlilik

`Task_FullESKFCorrection_200Hz()` her çalıştığında adaptör çağrılır; dahili
bölücü model adımını tam iki çağrıda bir gerçekleştirir. Her adım sırası:

1. sensör ve estimator durumunu doğrula,
2. `Ucus_Bilgisayari_output()`,
3. `Ucus_Bilgisayari_update()`.

Düşey ESKF/LIDAR veya attitude geçersizse bütün yeni model çıkışları sıfırlanır.
Veri yeniden geçerli olduğunda model kalıcı durumları sıfırlanarak temiz başlar.
Yatay mutlak konum yardımı mevcut projede bulunmadığından
`horizontal_position_valid=0` ve `rcs_output_valid=0` kalır.

## GRT koduna yapılan gömülü hedef uyarlamaları

- Masaüstü `rt_logging`, solver ve MAT-file kayıt altyapısı kaldırıldı.
- Eksik MATLAB `tmwtypes.h` yerine sabit genişlikli, yerel `rtwtypes.h` eklendi.
- Masaüstü simülasyonunun 7 saniyelik `tFinal` sınırı `-1` yapıldı.
- Algoritmanın blok matematiği, parametre tablosu ve durum makineleri korunur.
- Debug make dosyalarına yeni kaynaklar eklendi; model/adaptör kaynakları
  çift duyarlıklı matematik yükü nedeniyle `-O2` ile derlenir.

## CubeIDE doğrulama sırası

1. Projeyi `TGY_V8_19M_V48_CONTROL_INTEGRATED` adıyla içe aktarın.
2. Clean + Build yapın. Teslim ZIP'inde eski sürüme ait ELF/HEX özellikle yoktur.
3. Basınç/gaz hattı ve yeni algoritma fiziksel çıkışı kapalıyken çalıştırın.
4. Live Expressions'a `generated_fc_status` ekleyin.
5. `step_count` artışını 100 Hz, `invalid_input_count` değerini ve çıkışların
   `[0,1]` aralığını doğrulayın.
6. Özellikle model yürütme süresini DWT ile ölçün; 1 kHz IMU görevi ve 200 Hz
   ESKF düzeltmesinde deadline kaçırılmadığını doğrulayın.
7. V1/V3/V5/V7 mekanik yön eşlemesi, iğne komutu ve HIL sonuçları incelenmeden
   fiziksel bağlantı eklemeyin.

## İlgili dosyalar

- `App/Modules/Control/GeneratedFlightControl/generated_flight_control.[ch]`
- `App/Modules/Control/GeneratedFlightControl/Generated/`
- `App/Core/Tasks/app_tasks.c`
- `App/Common/app_config.h`
- `VALIDATION_V48_CONTROL_ALGORITHM.txt`

PE9 ayrılma konnektörü ve pre-flight durum makinesi için ayrıca
`README_V48_PE9_PREFLIGHT_TRIGGER_TR.md` dosyasına bakın.
