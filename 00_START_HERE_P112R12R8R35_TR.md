# P112R12R8R35 — Final Full-System Pressureless Dry-Run

Bu surum **atis firmware'i degildir**. Son fiziksel entegrasyon/dry-run adayidir.

Korunan otoriteler:
- Full-State ESKF + R8R30 sabit IMU->rocket matrisi
- Hover V19.6 -> P111/P110/P112 -> fiziksel needle (3 tur / 585 ADC)
- Landing-aware horizontal gate (absolute X/Y aid yokken position hold kapali)
- RCS V7.13.4 + R8R34 quiet-upright anti-chatter -> fiziksel RCS relays

R8R35 duzeltmeleri:
1. Legacy V49 `AttitudeControl_ForceSafe()` artik 200 Hz'de V7.13.4 fiziksel maskesini silemez. V49 sadece ic pulse/state kayitlarini safe'e alir.
2. `fl_rcs_applied_mask` gercek `SolenoidOutput_GetAppliedMask()` ile ayni anda raporlanir.
3. Fiziksel RCS pulse suresi telemetry: count / last / max. V7.13.4 normal pulse hedefi ~40 ms.
4. Needle low-level root-cause telemetry: autonomous fault, stall/powered ms, P110 state/result/abort, speed/stop-distance/correction ve command reject.
5. Ground vent ve servo fiziksel olarak izole kalir (`APP_P112R12_INERT_OUTPUT_ISOLATION_MODE=1`).

## Test — sadece basincsiz / depressurized

```bash
py monitor_uart_p112r12r8r35_final_full_system_dryrun.py --port COM21 --duration 35
```

Sira:
1. PE9 bagli: `READY=1`, `SYS=1`, `ESKF=1`, `CAL=12/1/0`. Needle/RCS fiziksel cikis olmamali.
2. PE9 ayir: flight active + auth. Needle Hover V19.6 komutunu takip etmeli.
3. 4-5 s dik ve hareketsiz tut: RCS sessiz.
4. Sonra yavasca 4-6 derece eg: RCS fiziksel pulse vermeli.
5. Monitor final gate: no faults, no misses/realign, RCS applied==physical, en az bir ~40 ms pulse.

Bu gate PASS olmadan bu build atis icin kabul edilmez.
