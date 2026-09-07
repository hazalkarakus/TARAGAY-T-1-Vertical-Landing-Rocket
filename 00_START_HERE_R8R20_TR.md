# P112R12R8R20 — 3 Tur Ana İğne Vana Limiti + Flight Logic Dry-Run (INERT)

Bu rev R8R19 flight-logic dry-run üzerine yalnız ana iğne vananın maksimum yazılımsal hareketini 4 turdan 3 tura indirir.

## Ana motor / iğne vana limiti
- Ölçülmüş kalibrasyon: **195 ADC/tur**
- Eski maksimum: 4 tur = 780 ADC
- Yeni maksimum: **3 tur = 585 ADC**
- Öğrenilmiş CLOSED referans 1018 ADC ise teorik maksimum OPEN hedef: **1018 - 585 = 433 ADC**
- CLOSED referans bootta dinamik öğrenildiği için gerçek minimum hedef `closed_ref - 585 ADC` olarak hesaplanır.
- Hem düşük seviye NeedleValve controller hem autonomous supervisor aynı 585 ADC limiti kullanır.

## Güvenlik
- RCS fiziksel çıkışları INERT / compute-only kalır.
- R8R19 synthetic flight-logic dry-run davranışı korunur.
- Bu paket uçuş build'i değildir. Basınç/gaz ile kullanılmaz.

## UART
```bash
py monitor_uart_p112r12r8r20_three_turn_flight_logic_dryrun.py --port COM21 --duration 25
```

## Beklenen needle aralığı
CLOSED yaklaşık 1018 ADC ise hiçbir normalize komut 433 ADC'den daha açık bir hedef üretemez.
