# P112R12R8R34 – RCS QUIET UPRIGHT / ANTI-CHATTER

Bu revizyon R8R33 fiziksel needle + fiziksel RCS authority tabanini korur.
Sadece RCS V7.13.4 low-angle reference-tracking yolu sakinlestirildi.

## Degisiklikler
- Fine tracking START deadband: 1.50 deg -> 2.50 deg
- Fine tracking STOP deadband: 0.80 deg -> 1.20 deg
- Fine tracking gyro rate LPF: alpha=0.25 (100 Hz'de ~30 ms sinifi zaman sabiti)
- Fine tracking pulse baslatma: ayni isaretli 3 ardışık ornek = 30 ms confirmation
- 10 deg predictive/hard safety selector: raw gyro rate ile eski V7.13.4 davranisini korur
- Fixed IMU->rocket matrix, RCS mapping, needle authority ve PE9/safety gate'leri degismedi

## Basinçsiz bench testi
1. Sistemi ac, READY=1 / system_ok=1 / eskf_ok=1 bekle.
2. PE9'u ayir.
3. Roketi 4-5 saniye olabildigince dik ve hareketsiz tut. Bu bolumde RCS kliklememeli.
4. Sonra roketi yavasca yaklasik 4-6 derece bir yone eg. RCS fiziksel cevap vermeli.
5. Diger eksende de istersen ayni kontrolu yap.

Monitor:
`py monitor_uart_p112r12r8r34_rcs_quiet_upright.py --port COM21 --duration 30`

Bu paket atis kalifikasyonu degildir; test basinçsiz bench dogrulamasidir.
