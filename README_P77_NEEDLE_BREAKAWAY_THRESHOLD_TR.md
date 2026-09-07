# TARAGAY-T1 P77 — Needle Breakaway Threshold Bench

## Neden P77?
P76 donanım testinde 6/6 ham pulse adımı çalıştı fakat ADC hiç değişmedi:
- OPEN PWM35 / 10 ms -> 0 ADC
- CLOSE PWM35 / 10 ms -> 0 ADC
- OPEN PWM45 / 10 ms -> 0 ADC
- CLOSE PWM45 / 10 ms -> 0 ADC
- OPEN PWM55 / 10 ms -> 0 ADC
- CLOSE PWM55 / 10 ms -> 0 ADC

Bu, P76'nın güvenli karakterizasyon yaptığı ancak motor-redüktör-vana sisteminin 55 PWM / 10 ms altında statik sürtünmeyi aşmadığı anlamına gelir.

## P77'nin amacı
Kapalı çevrim kontrolü hâlâ kullanmadan, sadece OPEN yönünde minimum hareket eşiğini bulmak.

Arama sırası:
1. PWM65 / 10 ms
2. PWM65 / 15 ms
3. PWM65 / 20 ms
4. PWM75 / 10 ms
5. PWM75 / 15 ms
6. PWM75 / 20 ms

İlk ölçülebilir doğru yön hareketinde P77 otomatik olarak DONE olur ve daha güçlü pulse denemez.

## Güvenlik
- DO-NOT-FLY bench firmware.
- Gaz / basınç BAĞLI OLMAYACAK.
- RCS elektrik gücü fiziksel olarak inhibit/kapalı tutulacak.
- NRF compile-time kapalı.
- UART RX komutları kapalı; UART yalnız telemetri TX.
- Normal closed-loop needle controller hareket üretmez.
- Pulse genişliği TIM7 1 kHz ile hard-stop edilir.
- Maksimum pulse: PWM75 / 20 ms.
- Referanstan 60 ADC uzaklaşma -> TRAVEL_GUARD abort.
- OPEN sırasında ADC >=3 artarsa -> WRONG_DIR abort.
- Pulse/settle sırasında USER butonuna basılırsa -> BUTTON_ABORT.
- flight_active / preflight fault / actuator fault / system_ok=0 -> motion yok / abort.
- RCS sürekli ForceSafe.

## Test
CubeIDE:
1. Clean Project
2. Build Project
3. Flash

Python:
```bat
python -u monitor_uart_p77_breakaway.py --port COM21 --log uart_p77_breakaway.txt
```

Önce:
- `SYS=1/0`
- `PE9=0`
- `flight=0`
- `ADC ~1023`

gör.

### USER #1
Mavi USER butonuna bir kez bas-bırak.
Bu yalnız kapalı referansı yakalar. Motor hareket etmez.

### Sonraki basışlar
Monitör hangi pulse'ın sırada olduğunu ekranda söyler.

Her basıştan sonra 500 ms settle bitene kadar butona tekrar basma.

Beklenen:
- hareket yoksa `NO_MOTION` ve bir sonraki adıma geçer.
- ilk gerçek OPEN hareketinde `THRESHOLD_FOUND` + `DONE`.

`THRESHOLD_FOUND` gördüğün anda TEST BİTTİ. Tekrar butona basma. Logu gönder.

OPEN yönünde ADC'nin azalması gerekir:
`1023 -> 1018 -> ...`

## Neden otomatik CLOSE yok?
P76'da tüm düşük pulse'lar hareketsizdi; henüz CLOSE breakaway eşiği bilinmiyor. P77 ilk OPEN eşiğini bulduktan sonra aynı logdan güvenli dönüş/closed-loop ayarını ayrı bench sürümünde çıkaracağız. Bu, kapalı hard-stop'a gereksiz CLOSE darbesi vermemizi önler.

## Uçuş
P77 uçuş firmware'i değildir. Test bittikten sonra uçuş baseline'ı olarak P71 saklanmalıdır.
