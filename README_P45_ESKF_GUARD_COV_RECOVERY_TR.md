# P45 — ESKF public guard, covariance recovery, IMU escalation ve 128 MiB SD

P45, P44 final soak testinde kalan iki arızaya dar kapsamlı müdahale eder:
ESKF sayısal çökmesinin bozuk `z/vz` durumunu public çıkışa taşıması ve ilk
soft SPI resync sonrasında tekrar bozulan IMU örneğinin tam recovery'ye yeterince
açık biçimde yükseltilmemesi. P44 scheduler, LiDAR best-effort zamanlaması,
CPU bütçesi, PE9/aktüatör güvenliği ve V14 SD frame yapısı değiştirilmemiştir.

## 1. ESKF anormal z/vz public-output guard

200 Hz correction görevinin public commit sınırına bir son kontrol eklendi.
Aday nominal durum önce sayısal sağlık ve covariance bütünlüğünden geçer; sonra
son kabul edilmiş public `z/vz` örneğine göre süre-bağımlı sıçrama kontrolü yapılır.

P45 varsayılan eşikleri:

- maksimum guard aralığı: `50 ms`
- z taban sıçrama payı: `0.50 m`
- z hız payı: `2.0 * max(|vz_old|, |vz_new|) * dt`
- vz taban sıçrama payı: `4.0 m/s`
- vz ivme payı: `120 m/s² * dt`

Guard yalnız origin zero tamamlandıktan ve vertical reacquire/inhibit aktif
değilken devreye girer. Anormal aday `FullESKF_UpdatePublicState()` çağrısından
**önce** reddedilir. Ardından ESKF güvenli reinitialize/reacquire yoluna alınır.

Reset reason kodları:

| Kod | Anlam |
|---:|---|
| 0 | NONE |
| 1 | MANUAL |
| 2 | STATE_NUMERICAL |
| 3 | COV_NONFINITE |
| 4 | COV_DIAGONAL |
| 5 | COV_ASYMMETRY |
| 6 | PUBLIC_Z_JUMP |
| 7 | PUBLIC_VZ_JUMP |
| 8 | PUBLIC_Z_VZ_JUMP |

## 2. Covariance bütünlük kontrolü ve safe reacquire

15x15 covariance matrisi için aşağıdaki bütünlük kontrolleri eklendi:

- bütün diagonal ve off-diagonal elemanlarda finite/reasonable kontrolü,
- diagonal elemanlarda `> 0` ve maksimum covariance sınırı,
- `P[i][j]` / `P[j][i]` simetri farkı kontrolü,
- symmetrize/clamp sırasında bulunan bozukluğun sonraki integrity check'e
  taşınması; yani clamp işlemi artık gerçek arızayı gizlemiyor.

Kontrol hem 25 Hz covariance service sonrasında hem 200 Hz public correction
sınırında yapılır. Hata görülürse public commit yapılmaz, `numerical_error_count`
ve ilgili covariance sayaçları güncellenir, ESKF sıfırlanıp mevcut attitude-backed
reacquire akışına geri döner.

## 3. UART P45 / TGY56 tanıları

UART data frame `$TGY56` oldu ve 228 alana çıktı. Yeni tanılar:

- `imu_pattern_recovery_escalations`
- `eskf_reset_reason`
- `eskf_reset_count`
- `eskf_num_errors`
- `eskf_public_rejects`
- `eskf_z_jump_rejects`
- `eskf_vz_jump_rejects`
- `eskf_cov_ok`
- `eskf_cov_checks`
- `eskf_cov_faults`
- `eskf_cov_reinits`
- `eskf_cov_diag_min_u1e6`
- `eskf_cov_diag_max_m1e3`
- `eskf_cov_sym_max_u1e6`

`monitor_uart_p45.py` / `monitor_uart_v56.py` reset reason kodunu isimle de
gösterir. UART fiziksel ayarı aynı kalır: USART2 PA2-TX / PA3-RX, 115200 8N1.

## 4. IMU: ikinci bozuk örnekte full non-blocking recovery

P44 davranışının ilk basamağı aynen korunur:

1. İlk repeated-word bozuk triplette yazılım-SPI transaction boundary soft resync.
2. Retry aynı görev içinde yapılmaz; sonraki 1 ms IMU task'a bırakılır.
3. Bu ikinci örnek de bozuksa ikinci soft-resync denenmez.
4. `IMU_StartRecovery()` ile mevcut tam recovery state machine hemen arm edilir.
5. Gerçek BUS_RESET / register configure adımları sonraki 1 kHz task çağrılarında
   ilerler; recovery akışına `HAL_Delay()` eklenmemiştir.

Yeni `imu_pattern_recovery_escalation_count` sayacı UART'ta görünür.

## 5. SD uzun soak alanı

V14 binary frame ve 200 Hz logging formatı değişmedi.

- birincil preallocation: **128 MiB** ≈ **29.1 dk**
- fallback: **96 MiB** ≈ **21.8 dk**
- veri hızı: 384 byte x 200 Hz = 76,800 B/s

## İlk P45 gazsız soak testi

1. Solenoid/motor/basınç gücü kapalı; yalnız aviyonik test.
2. CubeIDE: **Clean Project -> Build Project -> karta yükle**.
3. Boot banner'da `FW 8.19M-P45-ESKF-GUARD-COV-RECOVERY` ve `$TGY56` gör.
4. `monitor_uart_p45.py --port COMx` ile UART kaydını izle/kaydet.
5. En az 20 dakika, tercihen 25+ dakika çalıştır.
6. Test boyunca kartı kontrollü hareket ettirerek IMU/LiDAR gerçek veri değişimi
   oluştur; ancak aktüatör gücü kapalı tut.
7. Test sonunda UART kaydı ve `flight.bin` dosyasını birlikte incele.

## PASS ölçütleri

- `miss_imu=0`; scheduler/fast fault yeni hata üretmemeli.
- `sd_logging=1`, `sd_last_result=0`, `sd_dropped=0` beklenir.
- `eskf_cov_ok=1` normal çalışma boyunca korunmalı.
- `eskf_cov_faults`, `eskf_cov_reinits`, `eskf_num_errors`,
  `eskf_public_rejects` normal soak boyunca tercihen 0 kalmalı.
- Bir ESKF arızası gerçekten oluşursa bozuk aday public'e commit edilmeden
  reset reason görünmeli ve estimator reacquire yoluna girmeli.
- IMU repeated-word tekrarında ilk hata soft retry; ikinci bozuk örnekte
  `imu_pattern_recovery_escalations` ve full recovery sayacı artmalı.
- Herhangi bir estimator/sistem arızasında fiziksel RCS ve ana motor çıkışları
  güvenli durumda kalmalı.

## Not

Bu pakette kaynak seviyesinde P45 doğrulaması ve host-GCC `-fsyntax-only`
kontrolü yapılmıştır. Container'da `arm-none-eabi-gcc` bulunmadığından gerçek
STM32 cross-build burada koşturulmamıştır; karta yüklemeden önce CubeIDE build
zorunludur.
