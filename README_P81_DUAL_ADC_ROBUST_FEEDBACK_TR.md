# TARAGAY-T1 P81 — Dual-ADC Robust Needle Feedback

**Durum: DO-NOT-FLY / bench firmware.** P81 motoru hareket ettirmez. Diagnostic mode nedeniyle BTS7960 enable pinleri zorla LOW, LPWM/RPWM sıfır ve `DriveOpen/DriveClose()` no-op kalır. RCS de sürekli safe tutulur.

## P80 neden P81'e dönüştürüldü?

P80 bench kaydında ADC1 ve ADC2 aynı PC1 hattında birbirine yakın davranırken PC1 ölçümü belirgin şekilde gürültülüydü. ADC timeout görülmedi ve dahili VREFINT değişimi PC1'e göre oldukça küçüktü. Bu, tek bir ADC peripheral arızasından çok dış analog feedback hattını işaret ediyor.

P81 fiziksel arızayı 'yok saymak' için yapılmadı. Amaç, tek/ani bozuk ADC örneğinin gelecekte vana kontrolüne doğrudan gitmesini engelleyecek güvenlik katmanını bench'te ölçmek.

## P81 robust feedback zinciri

1. PC1, ADC1 ve ADC2 ile bağımsız 480-cycle sample time kullanılarak yaklaşık 500 Hz okunur.
2. `|ADC1-ADC2| > 30` ise o pair feedback'e alınmaz ve last-good tutulur.
3. Uyumlu pair'in ortalaması `pair_candidate` olur.
4. Son 7 kabul edilmiş pair üzerinde median filtre uygulanır.
5. Median, last-good filtered değerden 35 ADC'den fazla uzaklaşırsa tek örnekte kabul edilmez.
6. Yeni seviye üç ardışık median boyunca +/-20 ADC içinde kalırsa gerçek/sürekli hareket olarak kabul edilir (yaklaşık 6 ms confirmation).
7. Normal küçük değişimlerde 3 ADC deadband + hafif 1/4 IIR uygulanır.
8. `feedback_valid` ve `confidence_pct` ayrı olarak yayınlanır. Büyük sıçrama confirmation beklerken feedback `INVALID` yapılır.
9. Her 1 saniyede raw p-p / filtered p-p ve raw max-step / filtered max-step yayınlanır.
10. VREFINT p-p ve toplam ADC timeout sayısı izlenmeye devam eder.

## UART son 20 alan

- `p81_diag_flags`
- `p81_samples_total`
- `p81_adc1_raw`
- `p81_adc2_raw`
- `p81_pair_diff`
- `p81_pair_candidate`
- `p81_median7`
- `p81_filtered_adc`
- `p81_feedback_valid`
- `p81_confidence_pct`
- `p81_pair_reject_count`
- `p81_spike_reject_count`
- `p81_confirmed_jump_count`
- `p81_hold_count`
- `p81_adc_timeout_count`
- `p81_win_raw_pp`
- `p81_win_filtered_pp`
- `p81_win_raw_max_step`
- `p81_win_filtered_max_step`
- `p81_vref_win_pp_raw12`

## Beklenen bench sonucu

İyi bir yazılım bastırması için raw p-p yüksek olsa bile filtered p-p belirgin şekilde düşük olmalı. Örnek hedef davranış:

- raw p-p: 40–80 ADC olabilir (mevcut fiziksel sorun çözülmeden)
- filtered p-p: tercihen <= 5–10 ADC
- `feedback_valid = 1` çoğu zamanda
- `confidence_pct >= 60`, tercihen >=80
- `adc_timeout_count = 0`
- motor çıktıları `needle_enabled/rpwm/lpwm = 0/0/0`

Filtered p-p de 20+ ADC kalırsa bu firmware'i motor kontrolü için kullanmayacağız; elektronik feedback hattı fiziksel olarak düzeltilmelidir.

## Test

1. Gaz/basınç yok.
2. Mümkünse motor ve RCS güçlerini fiziksel olarak kapalı tut.
3. CubeIDE: Clean Project -> Build Project -> Flash.
4. Çalıştır:

```text
python -u monitor_uart_p81_robust_feedback.py --port COM21 --log uart_p81_robust_feedback.txt
```

5. 20–30 saniye pot, mil ve kablolara dokunma.
6. `uart_p81_robust_feedback.txt` dosyasını inceleme için gönder.

## Elektronik tarafta P80'in işaret ettiği yerler

Öncelik sırası:

1. Potansiyometrenin wiper (orta uç) teması / pot iç sürgüsü.
2. Wiper -> PC1 kablosu, konnektör pini/crimp, lehim veya breadboard/header teması.
3. Pot GND dönüşü. Özellikle GND motor/BTS7960 veya yüksek akım dönüşüyle kötü paylaşılmışsa analog ground bounce oluşabilir.
4. Pot 3.3 V besleme dalı. VREFINT'in stabil olması MCU VDDA'sını destekler ama pot başka bir 3.3 V dalındaysa o dalın stabil olduğunu kanıtlamaz.
5. Çok yüksek pot/source empedansı veya ADC pinine büyük seri direnç. P80 short-vs-long farkı bunu destekleyebilecek bir bulguydu.
6. PC1 PCB izi/header/lehim kusuru veya yüksek empedanslı PC1 hattının PWM/motor kablolarına yakın routing'i.
7. VDDA/VSSA decoupling/analog ground problemi daha düşük ihtimal; P80 VREFINT nispeten stabil olduğundan ana şüpheli değil.
8. Pot 5 V ile besleniyorsa ayrıca hatalı/riskli bağlantıdır; PC1'in 0–3.3 V sınırında kalması gerekir.

P81 bunların hiçbirini fiziksel olarak çözmez; sadece feedback'in güvenli şekilde reddedilip filtrelenip filtrelenemediğini ölçer.
