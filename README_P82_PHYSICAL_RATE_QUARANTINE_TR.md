# P82 - Physical Rate + Quarantine Robust Feedback (DO-NOT-FLY)

P81 normal ADC gürültüsünü iyi bastırdı; fakat uzun kayıtta birkaç örnek süren ortak-mod bir glitch, 6 ms persistence filtresini gerçek hareket gibi geçebildi. P82 bu spesifik arıza tipini hedefler.

## Güvenlik
- Motor compile-time kilitli: enable=0, LPWM=0, RPWM=0.
- DriveOpen/DriveClose no-op.
- RCS sürekli safe.
- PA0 hareket komutu üretmez.
- NRF kapalı, UART RX komutları kapalı.
- Bu sürüm uçuş için değildir.

## P82 filtre zinciri
1. ADC1 + ADC2 PC1 uzun sample @ ~500 Hz.
2. |ADC1-ADC2| > 30 ise pair reject.
3. Accepted pair ortalaması.
4. 2 ms'de >80 ADC raw step => fiziksel olarak imkansız kabul + quarantine.
5. 7-sample median.
6. 2 ms'de >20 ADC median step => fiziksel rate reject + quarantine.
7. Filtered değerden >24 ADC excursion => 8 direction-consistent sample (~16 ms) bekle.
8. Confirm edilmiş hareket bile outputta en fazla 6 ADC/sample ilerler; teleport yok.
9. Rate glitch sonrası min 100 ms quarantine; output last-good'da donar, feedback_valid=0.
10. Quarantine çıkışı için ayrıca last-good çevresinde +/-25 ADC bandında 25 stabil median (~50 ms) gerekir.
11. 3 ardışık pair reject de quarantine başlatır.

## UART P82 tail (20 field)
`p82_diag_flags,p82_samples_total,p82_adc1_raw,p82_adc2_raw,p82_pair_diff,p82_pair_candidate,p82_median7,p82_filtered_adc,p82_feedback_valid,p82_confidence_pct,p82_pair_reject_count,p82_rate_reject_count,p82_quarantine_count,p82_quarantine_rem_ms,p82_adc_timeout_count,p82_win_raw_pp,p82_win_filtered_pp,p82_win_raw_max_step,p82_win_filtered_max_step,p82_vref_win_pp_raw12`

## Test
1. No gas / no pressure.
2. Tercihen motor ve RCS fiziksel gücü de kapalı.
3. CubeIDE Clean -> Build -> Flash.
4. Banner `8.19M-P82-PHYSICAL-RATE-QUARANTINE-FEEDBACK` olmalı.
5. `python -u monitor_uart_p82_physical_rate_quarantine.py --port COM21 --log uart_p82_physical_rate_quarantine.txt`
6. En az 60-120 s hiç dokunmadan kaydet. P81'de kritik glitch ~83 s civarında görüldüğü için 20-30 s test artık yeterli değil.

Beklenen iyi sonuç: normal pencerelerde filtered p-p raw p-p'den çok küçük; glitch olduğunda rate_reject ve quarantine_count artar, filtered büyük adım yapmaz, feedback_valid quarantine boyunca 0 olur, timeout=0 kalır.
