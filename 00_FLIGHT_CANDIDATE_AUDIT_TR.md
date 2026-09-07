# TARAGAY-T1 R8R35R3R10R4R2 — RELATIVE 3 m / CEILING 4 m FLIGHT AUDIT

Bu paket R4R1 son test paketinden türetilmiştir. R4R2'de yalnız düşey görev geometrisi ve bağımsız ana-itki tavan koruması kasıtlı olarak değiştirilmiştir. ESKF, RCS V7.13.4, needle düşük-seviye kontrolü, STOP, SD ve nRF yolları korunmuştur.

## Aktif gerçek uçuş zinciri

PE9 ayrılma olayı -> `PreflightTrigger_IsFlightActive()` -> `App_IsActuatorAuthorized()` ->
Full-State ESKF gerçek/fresh veri -> `TaragayFlightLogic_Service200Hz()` -> 100 Hz kontrol adımı ->

- Ana iğne vana: `R8R32_ApplyTaragayPhysicalNeedleOutput()` -> P111/P110/P112 adaptive actuator -> motor
- RCS: `R8R33_ApplyTaragayPhysicalRCSOutput()` -> `SolenoidOutput_SetMask()` -> fiziksel röle/selenoid çıkışı

Her iki yol da STOP, preflight fault, SystemMonitor actuator fault ve ESKF inhibit kapılarıyla korunmaktadır.

## Kritik bayrak notu

`APP_P112R12_INERT_OUTPUT_ISOLATION_MODE = 1U` değerini uçuş için 0 yapmayın.
Bu isim eski/yanıltıcıdır: R8R33 gerçek RCS handoff yolu bu izolasyon altında özel olarak çalışabilirken,
aynı bayrak ground-vent ve servo gibi uçuşta gerekmeyen diğer fiziksel yolları güvenli durumda tutar.
Bu bayrağı 0 yapmak uçuş otoritesini "açmak" için gerekli değildir ve fiziksel çıktı yüzeyini genişletebilir.

## Mevcut görev profili

`TaragayFlightLogic` R4R2 görev geometrisi:

- PE9 ayrılmasından sonraki **ilk geçerli 100 Hz CG yüksekliği** `z_sep` olarak tutulur.
- Ascent/hover hedefi: **`z_sep + 3.00 m`** (`TFL_Z_RISE_DELTA_M`).
- Ana-itki emniyet tavanı: **`z_sep + 4.00 m`** (`TFL_Z_CEILING_DELTA_M`).
- Tavan tetiklenince `target_force_n=0` ve `valve_cmd=0`; RCS etkilenmez.
- Tavan koruması iniş kontrolünü kalıcı öldürmez; **`z_sep + 3.80 m`** altına inince yeniden ana-itki izni verir (0.20 m histerezis).
- Touchdown CG yüksekliği: **0.4013 m** (`TFL_Z_TOUCH_M`).
- Kontrol çekirdeği: Hover V19.6; RCS: V7.13.4.
- Flight logic gerçek ESKF verisi ile 100 Hz adım; servis 200 Hz çağrıda 2'ye bölünür.
- Hover durumunda maksimum bekleme: yaklaşık **8 s**; ardından descent.
- Ana iğne vana komutu Hover V19.6 `0.00..0.30` alanından mekanik `0..1` komuta ölçeklenir.

Örnek: `z_sep=1.00 m` ise hedef **4.00 m**, ana-itki tavanı **5.00 m** olur.

## SD davranışı

R8R35R3R10/R4R1 tasarımına göre uçuş tarafında yeni SDIO DMA yazımı başlatılmaz.
Uçuş frame'leri RAM'de tutulur; STOP/E-STOP sonrası iğne safe-close tamamlanıp motor/RCS tamamen OFF olduktan ve 3 s quiet koşulu sağlandıktan sonra SD'ye replay edilir.

## nRF / UART

- nRF flight TDD: kabul edilen komuta tek FAST cevap; RX komut heartbeat'ini korumak için tasarlanmıştır.
- STOP tek yönlü latch'tir.
- UART RX actuator commands kapalıdır; UART TX yalnız diagnostiktir.

## Denetimde görülen önemli gerçek

Kaynak yorumlarının bazıları hâlâ “inert/depressurized / dry-run” ifadeleri taşıyor. Ancak runtime zincirinde:

- R8R32 fiziksel needle handoff aktiftir.
- R8R33 fiziksel RCS handoff aktiftir.
- `ATT_CTRL_DRY_RUN = 0` olduğundan `SolenoidOutput` fiziksel yazım yapabilir.

Dolayısıyla bu paket basit bir compute-only test yazılımı gibi ele alınmamalıdır.

## Değişiklik politikası

R4R1'e göre yalnız şu davranışlar değişmiştir: sabit 5.00 m hedef kaldırılmış, PE9-sonrası +3.00 m göreli hedef eklenmiş ve +4.00 m göreli ana-itki tavan koruması eklenmiştir. Kontrol kazançları/state geçiş mantığının geri kalanı, RCS pulse mantığı, needle controller, ESKF, scheduler, SD, nRF ve mevcut STOP/interlock kapıları değiştirilmemiştir.

## Flight qualification notu

Bu dosya kaynak denetimidir; donanım üzerinde ARM target build, kablolama/pin doğrulaması, basınçsız actuator/RCS çıkış doğrulaması,
fail-safe/STOP doğrulaması ve saha risk değerlendirmesinin yerini tutmaz.
