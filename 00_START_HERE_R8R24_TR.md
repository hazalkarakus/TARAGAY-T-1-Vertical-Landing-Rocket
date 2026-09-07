# R8R24 — TILT AXIS SOURCE FIX — INERT

Amaç: R8R23 logunda tespit edilen attitude kaynak eşleşmesi hatasını düzeltmek.

## Kritik düzeltme
Roket upright kalibrasyonunda body +Z yerçekimi eksenidir. Bu nedenle yatay iki tilt DOF:
- Kanal A: ESKF **roll** + gyro-X
- Kanal B: ESKF **pitch** + gyro-Y

ESKF **yaw**, body-Z etrafındaki heading/spin açısıdır ve RCS lateral tilt kanalı olarak kullanılmaz.

MATLAB kaynak kodundaki `pitch/yaw` kanal adları korunmuştur; STM32 giriş adaptörü fiziksel eksenleri doğru eşler.

## Test
- Basınç/gaz yok.
- RCS ve needle fiziksel çıkışları INERT kalır.
- `ready=1` ve `fl_input_valid=1` sonrası:
  1. Bir lateral eksene +5° / merkeze / -5° / merkeze.
  2. Diğer lateral eksene +5° / merkeze / -5° / merkeze.
- Her pozisyonda yaklaşık 1–2 s tut.
- `fl_v1_events`, `fl_v3_events`, `fl_v5_events`, `fl_v7_events` sayaçları ile mapping doğrulanır.

Beklenen: ikinci kanal artık heading gibi sürekli 30°+ sürüklenmemeli.

UART:
`py monitor_uart_p112r12r8r24_tilt_axis_fix.py --port COM21 --duration 30`
