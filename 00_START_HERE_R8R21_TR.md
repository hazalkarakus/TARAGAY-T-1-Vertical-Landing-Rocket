# P112R12R8R21 — SINGLE FLIGHT AUTHORITY / 3 TURN / DRY-RUN INERT

## Karar
Bu revizyonda tek aktif mission flight-logic otoritesi `TaragayFlightLogic`tir.

Kaynak otorite:
- Hover: V19.6 (basinc/load-cell geri beslemesi yok, adaptif trim, kontrollu inis, dogrulamali touchdown)
- Yatay kontrol: Hover_Durum uyumlu landing-aware son surum
- RCS: V7.13.4, 40 ms GEVAX baseline, prediction/brake/break-before-make/touchdown kill
- Needle travel: 3 tur = 585 ADC maksimum travel

## Legacy landing retirement
Eski `GeneratedFlightControl` / Simulink vertical outer-loop ve `VerticalLandingControl` kaynaklari
telemetri/provenance uyumlulugu icin dosya olarak kalabilir; ancak `APP_LEGACY_VERTICAL_LANDING_RETIRED=1`
ile komut uretmeleri engellenmistir. Yanlislikla servisleri cagrilsa bile safe/zero cikis verir.

## Neden yeni atilan daha eski kodlara gecilmedi?
- Hover V15 `F_meas` itki geri beslemesine baglidir; ucus mimarisinde load-cell yoktur.
- RCS V7.5.4 75 ms Simulink baseline ve eski touchdown mantigidir.
- 4-girisli yatay kontrol landing state/diklestirme mantigini bilmez.
Mevcut V19.6 + landing-aware yatay kontrol + V7.13.4 daha uygun ve daha gunceldir.

## Test
Bu paket halen INERT / DRY-RUN'dir. Fiziksel needle ve RCS cikislari calismamalidir.

```bash
py monitor_uart_p112r12r8r21_single_flight_authority_dryrun.py --port COM21 --duration 25
```

Beklenen:
- state: 0 -> 1 -> 2 -> 3 -> 4
- RCS request hesaplanabilir
- fiziksel RCS mask = 0
- needle PWM = 0
- legacy generated main command = 0 / invalid-safe

NOT FLIGHT QUALIFIED.
