# P112R12R1 — Production Autonomous 950+ / Same-Target Guard / INERT

Bu profil, P112R12 inert production zincirindeki sabit CLOSED hedef etrafinda gorulen gereksiz P110 tekrar hareketlerini filtrelemek icin hazirlandi.

## Degisen tek kontrol davranisi

- Gercek GNC hedefi > +/-8 ADC degisirse: **beklemeden normal P111/P110 hareketi**.
- Ayni hedef devam ederken:
  - hata <= 12 ADC: HOLD,
  - hata 13..20 ADC: mekanik deadband/HOLD,
  - hata > 20 ADC: ancak **200 ms kesintisiz** kalirsa correction baslar.
- Hata yeniden <=20 ADC olursa 200 ms sayaci sifirlanir.

Bu guard sadece degismeyen hedefe tekrar correction uygulanmasini filtreler. GNC retarget yolu, P110 adaptive motor katmani, P83 feedback, TIM7, SD logger ve authorization zinciri degistirilmedi.

## Guvenlik / kapsam

**DO NOT FLY.** Bu paket sadece tamamen basincsiz/inert bench dogrulamasi icindir. RCS, ground vent ve vent-servo yazilimda SAFE kilitlidir; fiziksel yukler de izole kalmalidir. UART TX diagnostiktir, actuator RX komut yolu kapali kalir.

## Beklenen inert test

PE9 bagliyken preflight READY bekle. Sonra yalniz inert bench dogrulamasi icin PE9 ayrildiginda, masa ustunde GNC CLOSED hedefte kalirsa `p111_moves_completed` **0** kalmalidir. ADC jitter/backlash `+/-20 ADC` icinde motor hareketi baslatmamalidir.

Gercek GNC hedefi degisirse motor hareketi normal production zincirinden baslamalidir.
