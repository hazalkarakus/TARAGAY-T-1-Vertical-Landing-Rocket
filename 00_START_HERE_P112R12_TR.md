# P112R12 - Production Autonomous Needle / 950+ / Inert Validation

**DO NOT FLY.** Bu surum yalnizca tamamen basincsiz/inert bench dogrulamasi icindir.
Ana itki hatti, gaz/basinc, piro/atesleme ve RCS/vent fiziksel yukleri izole edilmis olmalidir.

## Ne degisti?

- P112R11 PA0 commissioning sequencer devre disi.
- P112R10 sanal GNC/descent stimulus devre disi.
- Needle komutunun tek sahibi tekrar production zinciri:
  `GeneratedFlightControl -> authorization -> P111 -> P110/P112 -> motor`.
- Mekanik testte dogrulanan feedback konvansiyonu korunur:
  - CLOSED referans >= 950 ADC
  - OPEN yonunde ADC azalir.
- R10R2 adaptive correction guard ve R10R3 same-target settled hold korunur.
- PE9/preflight/SystemMonitor/P83/command freshness interlocklari korunur.
- UART RX actuator komut yolu kapali; UART sadece TX diagnostik.
- Inert validation icin RCS, ground vent ve vent servo fiziksel cikislari yazilimda safe-state'e kilitlidir.

## Beklenen inert bench akisi

1. Vana gercek CLOSED konumda, feedback >=950 ADC.
2. Kart acilir; zero/reference otomatik yakalanir.
3. Tum preflight readiness geldiginde PE9 hala bagli iken motor kapali kalir.
4. Yalnizca tamamen basincsiz/inert bench'te PE9 ayrildiginda production authorization aktif olur.
5. Gercek GNC cikisi 0 ise vana CLOSED'da kalir; sentetik hareket yoktur.
6. GNC gercek sensorden nonzero hedef uretirse production P111/P110/P112 yolu motoru surer.

Bu build tam uclus/ucus kalifikasyonu degildir; non-needle fiziksel cikislar bilerek kilitlidir.
