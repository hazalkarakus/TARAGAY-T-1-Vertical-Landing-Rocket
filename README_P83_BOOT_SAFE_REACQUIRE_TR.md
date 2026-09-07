# P83 Boot Acquisition + Safe Re-Acquisition

P82 testinde `filtered_adc` acilis anindaki gecici bir degeri last-good olarak
alabildi. Sonraki gercek PC1 sinyali bu eski anchor'dan cok uzakta oldugu icin
quarantine recovery `abs(median-filtered) <= 25` kosulunu hic saglayamadi.

P83 bu problemi iki katmanla cozer:

- **Boot acquisition:** reset sonrasi hicbir ADC degeri aninda pozisyon olarak
  kabul edilmez. 7-ornek median sinyali yaklasik 200 ms boyunca toplam 24 ADC
  bandinda stabil kalirsa ilk anchor olusturulur.
- **Safe re-acquisition:** quarantine sonrasinda once eski last-good cevresine
  normal donus aranir. Sinyal baska bir bolgede yaklasik 500 ms boyunca stabil
  bir cluster olusturursa P83 bu yeni cluster'i yeniden anchor yapabilir.

Son madde yalnizca P83'te motor cikisinin compile-time kilitli olmasi nedeniyle
guvenlidir. Motor aktif bir firmware'de buyuk position mismatch otomatik olarak
re-anchor edilmemelidir.

P82'den korunan katmanlar:
- ADC1+ADC2 PC1 cross-check
- pair reject >30 ADC
- raw physical-rate reject >80 ADC / 2 ms
- 7-ornek median
- median physical-rate reject >20 ADC / 2 ms
- 16 ms direction-consistent trajectory confirmation
- 6 ADC/sample output slew limit
- 100 ms minimum quarantine
- VREFINT ve ADC timeout diagnostics

P83 UART tail yine 20 alandir ve toplam monitor field sayisi 356 kalir.
