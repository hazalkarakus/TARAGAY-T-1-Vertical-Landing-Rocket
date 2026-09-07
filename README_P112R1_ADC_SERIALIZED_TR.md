# P112R1 — ADC Serialized Deterministic Adaptive Bench

P112 logunda P83 boot acquisition hic tamamlanmadi. P112, P83 PC1 orneklemesini TIM7 ISR icine tasirken 20 Hz VREF probe main-contextte kalmisti. Iki yol da ADC1 SQR3/SMPR1 kayitlarina dokundugu icin race olusuyordu.

P112R1 degisiklikleri:
- P83 dual ADC PC1 sampling: TIM7, 500 Hz.
- VREF probe: ayni TIM7 contextinde 20 Hz, tamamen serialized.
- Main context ADC1/VREF'e dokunmaz.
- P111 otomatik commissioning baseline < 200 ADC ise motoru hareket ettirmez ve START_RANGE ile durur.
- UART/SD timing isolation, adaptive controller, hard powered-time cutoff ve deceleration P112 ile aynidir.

BENCH ONLY. Gaz/basinc baglama. Ilk testte motor beslemesini fiziksel olarak kesmeye hazir ol.
