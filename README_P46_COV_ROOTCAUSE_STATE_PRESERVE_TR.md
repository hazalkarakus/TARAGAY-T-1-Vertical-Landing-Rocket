# P46 — Covariance root-cause + state-preserving recovery

P46, P45 donanım soak kaydında görülen tekrarlı `COV_DIAGONAL` olayını kök
sebebe kadar izlemek ve covariance arızasında nominal navigasyon durumunu
sıfırlamamak için hazırlanmıştır. P45'teki public `z/vz` guard, IMU ikinci-bozuk
örnek full non-blocking recovery, P44 scheduler/LiDAR zamanlaması ve V14 SD
binary formatı korunmuştur.

P45 kaydında covariance simetrisi bozulmadan, yaklaşık periyodik biçimde
`COV_DIAGONAL` oluştuğu için P46 limit büyütmek yerine bozulmanın **hangi
işlemde, hangi state diagonalında ve clamp öncesi hangi gerçek değerde**
oluştuğunu kaydeder.

## 1. Covariance fault kök-sebep kaydı

Bir yapısal covariance problemi clamp edilmeden önce aşağıdaki bilgiler tutulur:

- `covariance_fault_stage`
- `covariance_fault_state_index`
- `covariance_fault_other_index`
- `covariance_fault_raw_value`
- `covariance_fault_aux_value`
- `covariance_fault_timestamp_us`
- rollback sırasında son sağlam P'nin kullanılıp kullanılmadığı

Stage kodları:

| Kod | Stage | Açıklama |
|---:|---|---|
| 0 | NONE | Henüz kayıt yok |
| 1 | PROPAGATE | 25 Hz covariance propagation |
| 2 | GRAVITY | gravity scalar correction |
| 3 | ZUPT | stationary velocity update |
| 4 | GYRO_BIAS | stationary gyro-bias update |
| 5 | ACCEL_BIAS | stationary accel-bias update |
| 6 | BARO | barometer correction |
| 7 | LIDAR | LiDAR vertical correction |
| 8 | INTEGRITY | periyodik full-matrix integrity scan |
| 9 | GAP_INFLATE | IMU gap sonrası covariance inflation |

State indexleri:

| Index | State |
|---:|---|
| 0 | PX |
| 1 | PY |
| 2 | PZ |
| 3 | VX |
| 4 | VY |
| 5 | VZ |
| 6 | THX |
| 7 | THY |
| 8 | THZ |
| 9 | BAX |
| 10 | BAY |
| 11 | BAZ |
| 12 | BGX |
| 13 | BGY |
| 14 | BGZ |
| 255 | N/A |

Örnek yorum: `stage=GRAVITY`, `state=THX`, negatif raw diagonal görülürse
bozulma gravity correction sonrası roll error covariance hattında oluşmuştur.
`stage=PROPAGATE` ise measurement update değil propagation tarafı incelenmelidir.

## 2. Float roundoff ile gerçek covariance arızası ayrıldı

P46'da `APP_FULL_ESKF_COV_NEGATIVE_ROUNDOFF_TOL = 1e-7` kullanılır.

- `0 ... -1e-7` aralığındaki çok küçük negatif diagonal, tek-hassasiyetli
  kayan-nokta iptal hatası kabul edilir; minimum variance'a floor edilir ve
  **yapısal covariance fault/recovery başlatmaz**.
- `-1e-7` değerinden daha negatif diagonal gerçek `COV_DIAGONAL` olarak
  kaydedilir ve covariance-only recovery başlatır.
- non-finite ve tolerans dışı asimetri yine gerçek fault'tur.

Tiny-negative olaylarında ayrıca en son roundoff kaydı tutulur:

- `covariance_roundoff_clamp_count`
- `covariance_roundoff_last_stage`
- `covariance_roundoff_last_state_index`
- `covariance_roundoff_last_raw_value`
- `covariance_roundoff_last_timestamp_us`

Böylece P45'teki olay aslında yalnız sayısal roundoff ise fault sayacı sıfır
kalırken hangi update'in küçük negatifi oluşturduğu yine görülebilir.

## 3. State-preserving covariance recovery

P45'te covariance fault, `FullStateESKF_Init()` üzerinden tüm estimator
reacquire akışını başlatıyordu. P46 covariance sebeplerinde bunu yapmaz.

Her tam covariance integrity PASS sonrasında 15x15 `P` matrisi
`covariance_last_good` checkpoint'ine alınır. Gerçek covariance fault oluşursa:

1. Bozuk stage/state/raw değerleri önce kaydedilir.
2. Bozuk measurement update'in error-state'i nominal duruma inject edilmez.
3. `P`, son integrity-PASS checkpoint'inden geri yüklenir.
4. Covariance propagation accumulator temizlenir.
5. Nominal position, velocity, quaternion, accel/gyro bias ve public generation
   **korunur**.
6. ESKF `healthy=1` olarak normal correction akışına devam eder.

Sayaçlar:

- `eskf_cov_faults`: yapısal covariance integrity fault sayısı
- `eskf_cov_reinits`: covariance recovery sayısı; P46'da eski isim protokol
  uyumluluğu için korunmuştur
- `eskf_cov_rollbacks`: state-preserving covariance rollback sayısı
- `eskf_cov_rollback_last_good`: son recovery'nin last-good P kullandığını gösterir

`STATE_NUMERICAL` veya public `z/vz` guard hataları covariance-only recovery'ye
çevrilmemiştir; bu gerçek nominal-state arızalarında P45 full safe
reinitialize/reacquire yolu aynen korunur.

## 4. UART P46 / TGY57

UART frame `$TGY57` ve **241 data field** oldu. Header'da CRC alanıyla birlikte
242 kolon vardır. Yeni P46 alanları:

- `eskf_cov_fault_stage`
- `eskf_cov_fault_state`
- `eskf_cov_fault_other`
- `eskf_cov_fault_raw_n1e9`
- `eskf_cov_fault_aux_n1e9`
- `eskf_cov_fault_time_ms`
- `eskf_cov_rollback_last_good`
- `eskf_cov_rollbacks`
- `eskf_cov_roundoff_clamps`
- `eskf_cov_roundoff_stage`
- `eskf_cov_roundoff_state`
- `eskf_cov_roundoff_raw_n1e9`
- `eskf_cov_roundoff_time_ms`

`monitor_uart_p46.py`, stage ve state kodlarını isimle gösterir. `raw_n1e9`
değerleri Python tarafında tekrar float'a çevrilir. Fiziksel UART değişmedi:
USART2 PA2-TX / PA3-RX, 115200 8N1, DMA TX.

## 5. IMU / SD / scheduler kapsamı

Bu üç bölüm P46'da bilerek değiştirilmemiştir:

- P45 IMU: ilk bozuk örnek soft resync, ikinci bozuk örnek full non-blocking
  recovery.
- P44 scheduler: covariance slot reservation ve LiDAR 1 kHz best-effort davranışı.
- SD: V14 / 384-byte binary frame, 200 Hz; 128 MiB primary + 96 MiB fallback.

`decode_flight_v14_p46.py`, P44/P45 V14 decoder ile byte-byte aynı decoder
mantığını kullanır.

## İlk P46 bench testi

1. Solenoid/motor/basınç gücü kapalı olsun; yalnız aviyonik.
2. CubeIDE'de **Project -> Clean**, ardından **Build Project**; zero-error build
   görmeden karta yükleme yapma.
3. Boot banner'da `FW 8.19M-P46-COV-ROOTCAUSE-STATE-PRESERVE` ve `$TGY57` doğrula.
4. UART:

```text
python monitor_uart_p46.py --port COM8
```

5. Önce 3–5 dakika kısa bench testi yap.
6. Özellikle şu alanları izle:
   - `eskf_cov_faults`
   - `eskf_cov_rollbacks`
   - `eskf_cov_roundoff_clamps`
   - fault stage/state/raw
   - roundoff stage/state/raw
   - `eskf_public_rejects`
   - `system_ok/system_fault`
7. Kısa test stabilse 20–25+ dakika soak testine geç.
8. Test sonunda `uart_p46.txt`, `flight_v14_state.csv` ve
   `flight_v14_fast_imu.csv` birlikte analiz edilmelidir.

## P46 test yorumu

### İdeal PASS

- `eskf_cov_faults = 0`
- `eskf_cov_rollbacks = 0`
- `eskf_public_rejects = 0`
- SD drop/write/overrun/timeout = 0
- IMU recovery failure = 0
- sistem başlangıçtan sonra `system_ok=1` durumunda kalır

`eskf_cov_roundoff_clamps` birkaç kez artabilir. Bu durumda stage/state/raw
kaydına bakılır. Sayı sürekli hızlı artıyorsa gerçek fault olmasa bile ilgili
update Joseph/PSD açısından iyileştirilmelidir.

### Fault tekrar ederse

P46'nın amacı bu durumda estimator'u tümden sıfırlamadan aşağıdaki kök-sebep
üçlüsünü üretmektir:

```text
STAGE + STATE + RAW DIAGONAL
```

Aynı stage/state tekrar ediyorsa P47'de yalnız o covariance update matematiği
Joseph-form/PSD-safe biçimde hedeflenebilir. Böylece bilinmeyen probleme karşı
covariance limitini körlemesine büyütmekten kaçınılır.

## Build notu

Teslim ortamında `arm-none-eabi-gcc` bulunmadığından gerçek Cortex-M4 target
link/build burada koşturulamamıştır. P46 kaynak validatorü, host regression
suite, Python syntax ve host-GCC C syntax kontrolleri PASS'tir. STM32'ye
flashlamadan önce CubeIDE Clean + Build zorunludur.
