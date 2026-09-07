# P80 — ADC1 / ADC2 / VREFINT yazılımsal çapraz kontrol

Bu sürüm P79 ADC diagnostic üzerine kuruludur ve **vana/motor hareketi yapmaz**.
Amaç, fiziksel bağlantıya dokunmadan PC1 geri besleme probleminde üç ana sınıfı
ayırt etmektir: dış analog sinyal/pot-kablo, ADC1'e özgü problem, analog
besleme/referans problemi.

## Ölçüm mimarisi

- PC1 aynı anda kullanılabilen `ADC1_IN11` ve `ADC2_IN11` üzerinden bağımsız
  tek-shot okumalarla yaklaşık 500 Hz izlenir.
- İki ADC'nin bir saniyelik peak-to-peak değerleri ve maksimum aralarındaki
  fark kaydedilir.
- Ardışık örneklerde 25 ADC10 veya daha büyük değişimler için `both`,
  `ADC1-only`, `ADC2-only` korelasyon sayaçları tutulur.
- Her 50 ms'de ADC1 dahili VREFINT kanalını okur; ardından PC1'i önce 3-cycle,
  sonra 480-cycle acquisition ile okur. Bu kısa/uzun fark kaynak empedansı ve
  settling davranışını görünür yapar.
- VREFINT'in bir saniyelik raw12 peak-to-peak değeri analog referans
  stabilitesi için kullanılır.

## P80 UART son 20 alanı

`p80_diag_flags,p80_samples_total,p80_adc1_long10,p80_adc2_long10,`
`p80_adc1_short10,p80_adc12_diff,p80_short_long_diff,p80_win_adc1_pp,`
`p80_win_adc2_pp,p80_win_pair_diff_max,p80_win_short_long_max,`
`p80_both_jump_count,p80_adc1_only_jump_count,p80_adc2_only_jump_count,`
`p80_vref_raw12,p80_vdda_mv_est,p80_vref_win_pp_raw12,`
`p80_adc1_timeout_count,p80_adc2_timeout_count,p80_vref_timeout_count`

Toplam UART sözleşmesi 356 field olarak korunmuştur.

## Güvenlik

Bu bir **DO-NOT-FLY** bench image'dır. `APP_NEEDLE_ADC_DIAGNOSTIC_MODE=1`
altında motor enable istekleri donanım servisinde 0'a zorlanır ve open/close
fonksiyonları PWM üretmez. RCS safe komutu da sürekli uygulanır.

## Çalıştırma

```text
CubeIDE -> Clean Project -> Build Project -> Flash
python -u monitor_uart_p80_adc_crosscheck.py --port COM21 --log uart_p80_adc_crosscheck.txt
```

20–30 saniye mekanizmaya ve kablolara dokunmadan kayıt alın.
