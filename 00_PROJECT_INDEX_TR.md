# TARAGAY-T1 — Proje Endeksi (00_PROJECT_INDEX_TR)

> Bu dosya, kök dizindeki 479 adet `.md`/`.txt` revizyon notunu ve 84 adet kök-dizin
> Python yer-istasyonu script'ini tek bir haritada toplamak için hazırlanmıştır.
> **Hiçbir mevcut dosya taşınmamış, yeniden adlandırılmamış veya silinmemiştir** — bu
> dosya salt-ekleme (additive) bir endekstir.

---

## 1. Proje Özeti

TARAGAY-T1, STM32F407 (Cortex-M4) üzerinde çalışan, dikey iniş/hover yapan bir yarışma
roketinin uçuş bilgisayarı yazılımıdır. Mimari `App/` altında katmanlanmıştır:

- **App/Core** — scheduler (200 Hz/1 kHz görev döngüleri) ve `app_tasks.c` (görev
  entegrasyonu, UART/telemetri alan tanımları).
- **App/Modules** — `Estimation/FullStateESKF` (tam durum hata-durum Kalman filtresi),
  `Control/TaragayFlightLogic` (görev state machine'i: INIT→SELF_CHECK→PREPOSITION→
  READY→ASCENT→CAPTURE→HOVER→DEGRADED/SAFE), `Control/NeedleValve`, `Sensors`, `NRF24`,
  `RemoteControl`.
- **App/Services** — `PreflightTrigger` (PE9 ayrılma/emniyet mantığı), `SDLogger`,
  `SolenoidOutput` (RCS), `NeedleValveAutonomousControl`/`NeedleValveHardware` (ana iğne
  vana motoru), `UARTTelemetry`, `NRFTelemetry`, `SystemMonitor`.
- **Kök dizin** — CubeIDE proje dosyaları, onlarca yıl(!) süren iterasyonun revizyon
  notları (`00_START_HERE_*`, `README_*`, `VALIDATION_*`, `*_DO_NOT_FLY.txt`,
  `*_SHA256.txt` gibi) ve yer istasyonu tarafı Python script'leri (`monitor_uart_*.py`,
  `decode_flight_*.py`, `sd_flight_reader_motor_v16.py`).

**Mevcut nihai durum: R16 / PE9.** Uçuş otoritesi zinciri şu şekildedir: PE9 ayrılma
konektörü → `PreflightTrigger` (flight_active tek yönlü latch) → `App_IsActuatorAuthorized()`
→ gerçek/taze Full-State ESKF verisi → `TaragayFlightLogic_Service200Hz()` (100 Hz kontrol
adımı) → ana iğne vana (`R8R32` fiziksel handoff → P111/P110/P112 adaptif aktüatör → motor)
ve RCS (`R8R33` fiziksel handoff → `SolenoidOutput_SetMask()` → röle/selenoid). Aktif uçuş
geometrisi: PE9-sonrası ilk geçerli CG yüksekliği `z_sep`; hedef `z_sep+3.00 m`; ana-itki
emniyet tavanı `z_sep+4.00 m` (0.20 m histerezisli); needle 3 tur / 585 ADC; Hover V19.6 +
RCS V7.13.4 kontrol çekirdekleri korunur. Uçuş sırasında SD'ye doğrudan yazılmaz; kayıtlar
RAM'de tutulur ve STOP/E-STOP sonrası 3 s "quiet" koşulu sağlanınca SD'ye replay edilir
(V16 SD çerçeve formatı, 192 bayt/frame).

---

## 2. Revizyon Zaman Çizelgesi / Soy Ağacı

Proje isimlendirmesi iki iç içe geçmiş eksen kullanır: **P-serisi** (kabaca kronolojik
büyük adımlar, P30…P112) ve her P adımının içinde dallanan **R-serisi alt revizyonlar**
(`R<n>` bir öncekinin küçük bir türevidir; `R<n>R<m>` onun türevidir, vs. — örn.
`P112R12R8R35R3R10R4R2`, `…R4R1`'in, o da `…R4`'ün, o da `…R3R10`'un türevidir).
Ayrıca **V-serisi** (V8.x, V2x-V5x) daha eski bir donanım/entegrasyon aşamasına aittir ve
P-serisinin atasıdır. Son olarak **R16**, STM32 koduna Simulink/MATLAB tarafındaki ayrı bir
model revizyon numarasıdır (`Ucus_Bilgisayari_3p5mm_HOVER_R16e5/R16e6`) ve **SD log format
sürümü olan "V16" ile karıştırılmamalıdır** — ikisi bağımsız numaralandırmalardır.

### 2.1 Erken/orta dönem — V-serisi ve P-serisi ana hatlar (tarihsel)

- **V8.x - V2x/V5x (README_V8_3…V8_19M, README_V23/V30/V38-V50/V54/V55, vb.):** Sensör
  entegrasyonu (IMU/BMP585/LiDAR), SD logger, nRF24 haberleşme, needle valve donanımı,
  ESKF gölge/aktif entegrasyonu, PE9 kilidi ve scheduler'ın kademe kademe kurulduğu dönem.
  Sonuncusu `README_V55_FLIGHT_INTERLOCK_TR.md` (PE9 uçuş kilitli sürüm).
- **P30-P57 (README_P30…P57):** 200 Hz ESKF, SD/nRF/IMU/LiDAR sağlamlaştırma, kovaryans
  kök-neden düzeltmeleri, ilk "Flight Candidate" adayları (P48/P49).
- **P59-P71:** nRF explicit-TDD telemetri protokolünün kurulması (P63/P64/P67/P68),
  E-STOP yetki sertleştirme (P60), karşılıklı RCS tahliye (P71).
- **P74-P96 (çoğu `START_HERE_P*.txt` + `*_DO_NOT_FLY.txt`):** Needle valve donanım
  karakterizasyonu — breakaway eşiği/zamanlaması, PA0 güvenli adım, dual-ADC/robust
  feedback (P80-P83), ADC sweep/bypass/brake bench testleri (P87-P96). Tümü **sadece
  bench/DO-NOT-FLY** damgalıdır.
- **P108-P112 (temel):** P110 adaptif needle konum kontrolörü, P111 5-çevrim adaptif
  öğrenme, P112 UART-bağımsız deterministik adaptif needle kontrolcüsü — bunlar
  `P112R12R8...` alt-ağacının temelini oluşturur.

### 2.2 P112 alt-revizyon ağacı → final dal

```
P112
 └─ R12 (Production Autonomous / 950+ / Inert)
     └─ R8 (Clean Production Candidate / INERT)
         ├─ R2, R7, R8, R9, R11, R12, R13, R16 … (sensör/scheduler/ESKF determinizmi,
         │                                          nRF coexistence, E-STOP hardening)
         ├─ R19 (Flight Logic dry-run, MATLAB otoritesi STM32'ye taşınır)
         ├─ R20 (3 tur needle limiti + dry-run)
         ├─ R21 (Single Flight Authority, 3 turn, dry-run inert)
         ├─ R22 (gerçek ESKF girişi, compute-only)
         ├─ R23-R29 (IMU→rocket-frame kalibrasyonu, 3/5 poz + paired-axis derived-Z)
         ├─ R32 (REAL FLIGHT LOGIC → fiziksel ana needle handoff)
         ├─ R33 (REAL FLIGHT LOGIC → fiziksel RCS handoff)
         ├─ R34 (RCS quiet upright / anti-chatter)
         └─ R35 (Final Full-System Pressureless Dry-Run) — "atış firmware'i değildir"
             ├─ R35R1 (diagnostic closure)
             ├─ R35R2 (closure fixes)
             └─ R35R3 (SD service closure)
                 ├─ R35R3R2 (UART phase retry hotfix)
                 ├─ R35R3R3 (SD phase retry hotfix)
                 ├─ R35R3R4 (SD host-busy defer)
                 ├─ R35R3R5 (P71 SD known-good restore provenance)
                 ├─ R35R3R6 (motor-EMI actions / SD transient start retry)
                 ├─ R35R3R7 → R35R3R7R1 (SDIO motor-EMI diagnostic → no runtime HAL_Delay)
                 ├─ R35R3R8 (motor-aware SD 30 Hz)
                 ├─ R35R3R9 (SDIO 750 kHz + actuator quiet window)
                 └─ R35R3R10 (RAM-FIRST FLIGHT LOGGER — SD, uçuş sırasında yazılmaz)
                     ├─ R35R3R10R1 (60 s endurance bench)
                     ├─ R35R3R10R2 (E-STOP + feedback recovery)
                     ├─ R35R3R10R3 (Final Candidate)
                     └─ R35R3R10R4 (nRF Stable TDD)
                         └─ R35R3R10R4R1 (postflight SD safe replay) — kısaltılmış adı
                            kök dizinde **"R4R1"** olarak geçer.
                             ├─ R4R2 (PE9-sonrası göreli +3 m hedef / +4 m ana-itki tavanı)
                             │        — bu, `00_FLIGHT_CANDIDATE_AUDIT_TR.md`'nin konusudur.
                             └─ R4R3 (needle 3 tur → 2 tur denemesi) — **DEAD-END**: mevcut
                                      kaynakta (`app_config.h`,
                                      `APP_R16_NEEDLE_MAX_TURNS=3.00f`,
                                      `APP_R8R32_NEEDLE_MAX_TRAVEL_ADC=585U`) hâlâ 3 tur/585
                                      ADC kullanılıyor — yani R4R3 değişikliği sonradan
                                      **uygulanmamış/geri alınmış** görünüyor. Sadece
                                      tarihsel referans olarak okunmalı.
```

### 2.3 R16 / PE9 katmanı (nihai)

`R4R2` (+3 m / +4 m göreli görev geometrisi) tabanı üzerine, **Simulink model revizyonu
R16** (`R16e5`/`R16e6`, `b_hat` DOWN=0.050/UP=0.015, fast authority, 3-turn nonlinear
Cv↔turns eşleme, ±30° RCS zarfı) ve **PE9 görev-başlangıç otoritesi** (PE9 LOW = bağlı/GND
→ state machine ilerlemez; PE9 HIGH debounced + preflight_ready → `flight_active` latch
olur ve INIT ile görev başlar) `App/Modules/Control/TaragayFlightLogic/taragay_flight_logic.c`,
`App/Core/Tasks/app_tasks.c` ve `App/Services/PreflightTrigger/preflight_trigger.c`
dosyalarına entegre edilmiştir. Bu üç dosya `R16_CRITICAL_SOURCE_SHA256.txt` ile SHA-256
imzalanmıştır ve `R16_HOST_SYNTAX_CHECK.txt` (clang -fsyntax-only PASS) ile
`R16_PE9_STATIC_AUDIT.txt` (16/16 PASS statik denetim) tarafından doğrulanmıştır.

Not: `App/Common/app_version.h` içindeki `APP_VERSION_STRING` hâlâ
`"8.19M-P112R12R8R35R3R10R4R2-REL3M-CEIL4M-FLIGHT"` metnini taşıyor — yani firmware
banner'ı R16/PE9 katmanı eklendikten sonra güncellenmemiş. Sayısal sabitler (3 m/4 m/3 tur)
zaten R4R2 ile aynı olduğu için davranışsal bir çelişki yoktur, ancak banner string'i R16'yı
yansıtmıyor; ileride banner güncellenirse bu not da güncellenmelidir.

### 2.4 Yan dallar (ana uçuş hattı DIŞINDA — tarihsel/özel amaçlı)

- **`000_START_HERE_NRF_RX_ONLY_100HZ_TR.md`** ve **`_250KBPS_TR.md`** +
  **`CHANGE_AUDIT_NRF_RX_ONLY_100HZ.txt`** + **`250KBPS_CHANGE_AUDIT.txt`**: `R4R3`
  tabanından (`TARAGAY_T1_R8R35R3R10R4R3_REL3M_CEIL4M_2TURN_FLIGHT`) türetilmiş, roketin
  nRF downlink'ini tamamen kapatıp yalnızca yer→roket tek yönlü 100 Hz / 250 kbps RF
  yaptığı **ayrı bir yan varyant**. Ana R16/PE9 hattına dahil edilmemiştir; sadece o özel
  ground-link testi için referanstır.
- **`REFERENCE_MATLAB_*.txt`**: Hover/RCS kontrol kanununun MATLAB kaynağı (Simulink
  taşınma öncesi doğruluk referansı). Kod değil, türetme kaynağı olarak arşivde tutulur.

### 2.5 Güncel/nihai kabul edilmesi gereken belgeler

Aşağıdaki dosyalar **şu an güncel/nihai** kabul edilmelidir (yeni katılan biri veya
ileride başka bir revizyon eklerken önce bunlar okunmalı):

1. `00_R16_FINAL_PE9_MISSION_START_TR.md` — PE9 görev-başlangıç otoritesi (en üst düzey özet).
2. `00_FLIGHT_CANDIDATE_AUDIT_TR.md` — aktif gerçek uçuş zinciri + R4R2 görev geometrisi denetimi.
3. `R16_STM_INTEGRATION_NOTES.txt`, `R16_PE9_STATIC_AUDIT.txt`, `R16_HOST_SYNTAX_CHECK.txt`, `R16_CRITICAL_SOURCE_SHA256.txt`.

> **Bilinen tutarsızlık:** `00_IMPORT_THIS_PROJECT_FIRST.txt` hâlâ CubeIDE import hedefi
> olarak `P112R12R8R35_FINAL_FULL_SYSTEM_DRYRUN`'ı gösteriyor — bu, R16/PE9'dan çok daha
> eski bir noktadır ve muhtemelen güncellenmemiş. Proje klasörünü import ederken bu dosyaya
> değil, yukarıdaki 4 belgeye güvenin.

---

## 3. Kök Dosya Sınıflandırması

| Kategori | Örnek desenler | Durum |
|---|---|---|
| R16/PE9 nihai belgeler | `00_R16_FINAL_PE9_MISSION_START_TR.md`, `00_FLIGHT_CANDIDATE_AUDIT_TR.md`, `R16_*.txt` | **AKTİF — önce oku** |
| R4R1/R4/R3R10 serisi (bir önceki nesil, hâlâ referans) | `README_R8R35R3R10R4R1_*`, `README_R8R35R3R10R4_*`, `R8R35R3R10R4R1_VALIDATION.txt`, `R3R10R3_TO_R3R10R4_NRF_DIFF.txt` | Aktif referans (nRF/SD davranışının kaynağı) |
| SD log formatı | `README_MOTOR_SD_LOG_V16_TR.md`, `MOTOR_SD_LOG_V16_VALIDATION.txt` | **AKTİF** (mevcut SD frame formatı) |
| R4R2 / R4R3 uç dallar | `00_START_HERE_R4R2_RELATIVE_3M_CEILING_4M_TR.md`, `00_START_HERE_R4R3_2TURN_MAX_TR.md` | R4R2 aktif geometriyi belgeler; R4R3 **tarihsel/dead-end** (bkz. §2.2) |
| P112R12R8R\* ara revizyonlar (R2…R35, dry-run/inert/kalibrasyon) | `00_START_HERE_P112R12R8R*_TR.md`, `P112R12R8R*_VALIDATION.txt`, `P112R12R8R*_DO_NOT_FLY.txt`, `*_FROZEN_*HASH_AUDIT.txt`, `*_HOST_TEST.txt`, `*_HOST_SYNTAX.txt`, `*_CHANGE_AUDIT.txt`, `*_MONITOR_SELFTEST.txt`, `*_STATIC_AUDIT.txt` | Sadece tarihsel — o ara adımın kanıt/denetim kaydı |
| P30-P111 dönemi (erken sensör/ESKF/scheduler/needle bench) | `README_P30…P111*`, `VALIDATION_P*.txt`, `*_SOURCE_SHA256.txt`, `CLANG_SYNTAX_P*.txt`, `LIVE_EXPRESSIONS_P*.txt`, `*_CUBEIDE_IMPORT_TR.txt`, `START_HERE_P77/P78/P80…P96` | Sadece tarihsel |
| V8.x / V2x-V5x dönemi (donanım entegrasyon günlüğü) | `README_V8_*`, `README_V2x/V30/V38…V55*`, `VALIDATION_V*.txt`, `LIVE_EXPRESSIONS_V*.txt`, `*_FILE_HASHES.txt` | Sadece tarihsel |
| nRF RX-ONLY yan dalı | `000_START_HERE_NRF_RX_ONLY_100HZ*_TR.md`, `CHANGE_AUDIT_NRF_RX_ONLY_100HZ.txt`, `250KBPS_CHANGE_AUDIT.txt` | Ana hat dışı, özel amaçlı — tarihsel/referans |
| MATLAB kaynak referansları | `REFERENCE_MATLAB_*.txt` | Arşiv/köken referansı (kod değil) |
| Genel eski üst-düzey denetim | `FLIGHT_CANDIDATE_CRITICAL_SHA256.txt` (R4R2 döneminden geniş dosya seti) | Tarihsel — güncel dar kapsam için `R16_CRITICAL_SOURCE_SHA256.txt`'ye bakın |
| CubeIDE proje meta | `.cproject`, `.project`, `.mxproject`, `.settings/` | Aktif (araç yapılandırması — bu endeksin kapsamı dışında) |

---

## 4. Ground-Station Script Tablosu (`monitor_uart_*.py`, `decode_flight_*.py`, SD okuyucular)

Kök dizinde 84 Python script'i var. Aşağıdaki tablo en güncel/kullanılabilir olanları
tek tek, geri kalanını dönem/gruplar hâlinde özetler.

| Dosya | İlişkili revizyon | Durum |
|---|---|---|
| `monitor_uart_p112r12r8r35r3r10r4r1_postflight_sd_safe_replay_LIVE.py` | R8R35R3R10R4R1 (nRF stable TDD + postflight SD safe replay) | **GÜNCEL** — R16/PE9 katmanı UART `$TGY73,…` alan listesini değiştirmedi (protokolde zaten `pe9_open`, `fl_*` görev-mantığı alanları var); bu, mevcut donanımla kullanılacak script'tir. |
| `sd_flight_reader_motor_v16.py` | SD log format V16 (192 B compact + V14 384 B) | **GÜNCEL** — uçuş sonrası `flight.bin` çözümleme; V14/V15/V16 formatlarının hepsini destekler. |
| `monitor_uart_p112r12r8r35r3r10r4_nrf_stable_tdd_LIVE.py` | R8R35R3R10R4 | Tarihsel — bir önceki nesil, R4R1'in temeli. |
| `monitor_uart_p112r12r8r35r3r10r3_final_candidate_LIVE.py` | R8R35R3R10R3 | Tarihsel — Final Candidate ara adımı. |
| `monitor_uart_p112r12r8r35r3r10r2_estop_fb_recovery_LIVE.py`, `…r10r1_60s_endurance_LIVE.py`, `…r10_ram_first_flight_logger_LIVE.py` | R8R35R3R10 ailesi | Tarihsel — RAM-first logger'ın kademeli geliştirme adımları. |
| `monitor_uart_p112r12r8r35r3r9_sdio_750khz_actuator_quiet_LIVE.py` … `…r3r2*`/`…r3r3*`/`…r3r4*`/`…r3r6*`/`…r3r7*`/`…r3r7r1*`/`…r3r8*` (7 dosya) | R8R35R3R2…R3R9 | Tarihsel — SD/SDIO/UART hotfix zincirinin her adımına özel bench script'i. |
| `monitor_uart_p112r12r8r35_final_full_system_dryrun.py`, `…r35r1_diagnostic_closure.py`, `…r35r2_closure_fixes.py` | R8R35 / R35R1 / R35R2 | Tarihsel — final dry-run ve kapanış düzeltmeleri. |
| `monitor_uart_p112r12r8r34_rcs_quiet_upright.py`, `…r33_real_flight_logic_physical_rcs.py`, `…r32_real_flight_logic_physical_needle.py` | R8R34/R33/R32 | Tarihsel — fiziksel RCS/needle handoff'un ilk etkinleştirildiği adımlar. |
| `monitor_uart_p112r12r8r29_paired_axis_derived_z_cal.py` … `r28/r27/r26/r24` (5 dosya) | R8R24-R8R29 | Tarihsel — IMU→rocket-frame kalibrasyon bench script'leri. |
| `monitor_uart_p112r12r8r22_real_eskf_compute_only.py`, `…r21_single_flight_authority_dryrun.py` | R8R21/R8R22 | Tarihsel — flight-logic dry-run/compute-only aşaması. |
| `monitor_uart_p112r12r8r{2,7,8,9,11,12,13}*` (7 dosya) | R8R2…R8R13 | Tarihsel — sensör/scheduler/ESKF determinizmi ve nRF coexistence bench'leri. |
| `monitor_uart_p112r12*`, `…r12r1*`, `…r12r3*`, `…r12r8_production_candidate_inert.py`, `…r11r2_mech_commission.py`, `…r10r3_gnc_motor_bench.py`, `…r112_deterministic_adaptive.py`, `…r112r1_adc_serialized.py`, `…r112r2_static_burst.py`, `…p111_5cycle_adaptive_learning.py` | P112/P112R1…R12 ailesi | Tarihsel — needle valve adaptif kontrolcü (P110-P112) doğrulama script'leri. |
| `monitor_uart_p34.py` … `monitor_uart_p93_boost_400adc.py` (yaklaşık 25 dosya, P34-P93) | P34-P93 | Tarihsel — erken sensör/ESKF/scheduler/needle bench dönemi, her biri tek bir P-notuna eşlenir. |
| `monitor_uart_v54.py`, `monitor_uart_v55.py`, `monitor_uart_v56.py` | V54/V55/V56 | Tarihsel — P-serisi öncesi son V-serisi script'leri. |
| `decode_flight_v12_needle.py` … `decode_flight_v14_p53.py` (9 dosya), `decode_r3r10_flight_bin.py` | SD format V12-V14, R3R10 öncesi | Tarihsel — `sd_flight_reader_motor_v16.py` ile yer değiştirmiştir; sadece eski `.bin` dosyalarını çözmek için saklanmalıdır. |

---

## 5. Bu Dosyanın Canlı Tutulması

Bu endeks **statik bir anlık görüntü değil, canlı bir belge** olarak tasarlanmıştır.

- Yeni bir `00_START_HERE_*` / `README_*` revizyon notu eklendiğinde, §2 (zaman
  çizelgesi) ve §2.5 (nihai belge listesi) güncellenmelidir.
- Yeni bir `monitor_uart_*.py` veya SD okuyucu script'i eklendiğinde §4 tablosuna bir
  satır eklenmeli ve bir önceki "GÜNCEL" script "Tarihsel" olarak işaretlenmelidir.
- `App/Common/app_version.h` içindeki `APP_VERSION_STRING` güncellenirse, §2.3'teki
  "bilinen tutarsızlık" notu kaldırılmalı veya güncellenmelidir.
- Bu dosya salt-ekleme ilkesiyle hazırlanmıştır; mevcut hiçbir revizyon notu, script veya
  kaynak dosya bu endeks nedeniyle taşınmamalı/silinmemelidir.
