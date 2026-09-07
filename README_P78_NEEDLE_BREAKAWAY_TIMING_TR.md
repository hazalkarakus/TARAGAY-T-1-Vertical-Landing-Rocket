# TARAGAY-T1 P78 — Needle Breakaway Timing Bench

## Amaç
P77 testinde OPEN yönünde PWM65 ve PWM75 darbeleri 20 ms'ye kadar uygulanmasına rağmen ADC hareketi görülmedi. P74'te ise daha uzun süreli sürüşte vana aşırı hızlanıp hedefi geçti. P78'in amacı PWM değerini sabit tutup **hareketin kaçıncı milisaniyede başladığını** doğrudan ölçmektir.

P78 bir **DO-NOT-FLY bench firmware**'idir. P71 ana/uçuş baseline'ı değiştirilmemiştir.

## P78 testi
- PWM: **75 / 255**
- Yön: yalnız **OPEN**
- Maksimum sürüş süresi: **80 ms**
- ADC hareket eşiği: **3 ADC**
- ADC örnekleme: pulse boyunca yaklaşık **1 ms aralıkla**, main context içinde
- Hareket görülür görülmez: **PWM=0 + dynamic brake**
- Sonrasında: +100 ms, +250 ms, +500 ms ADC ölçümü
- Otomatik CLOSE: **YOK**

TIM7 ayrıca 80 ms mutlak hard-stop sağlar. ADC algılama çalışmasa bile motor 80 ms'den uzun sürülmez.

## İzole güvenlik profili
P78'de:
- NRF compile-time kapalıdır.
- UART RX komutları kapalıdır; UART yalnız TX diagnostiktir.
- RCS fiziksel kontrol yolu bench profilde kapalıdır ve `SolenoidOutput_ForceSafe()` sürekli uygulanır.
- `flight_active`, preflight fault, actuator fault veya `system_ok=0` hareketi engeller/abort eder.
- ADC <=3 -> anında kesme/abort.
- OPEN sırasında ADC artarsa >=3 ADC -> wrong-direction abort.
- Referanstan >30 ADC ayrılma -> anında travel-guard brake/abort.
- Pulse/settle sırasında mavi USER butonuna tekrar basmak -> local abort.

İlk testte **gaz/basınç bağlı olmayacak** ve RCS elektrik gücü fiziksel olarak inhibit/kapalı tutulacaktır.

## Kullanım
1. CubeIDE: `Clean Project -> Build Project -> Flash`.
2. Python monitor:

```bat
python -u monitor_uart_p78_breakaway_timing.py --port COM21 --log uart_p78_breakaway_timing.txt
```

3. `SYS=1/0`, `flight=0`, `PE9=0` bekle.
4. Mavi USER butonu #1: kapalı konum referansını alır. Motor hareket etmez.
5. Monitörde `P78 READY/IDLE` gör.
6. Mavi USER butonu #2: **tek** PWM75 / max80ms OPEN testi başlar.
7. Test sırasında butona dokunma.
8. Sonuç `BREAKAWAY_FOUND` veya `NO_MOTION_80MS` olunca bir daha butona basma ve logu gönder.

## Beklenen PASS örneği
Örnek:

```text
PWM=75
max/actual/detect=80/33/33ms
det=1
start ADC=1023
detect ADC=1020
brake ADC=1020
```

Bu durumda gerçek breakaway yaklaşık 33 ms kabul edilir. +100/+250/+500 ms değerleri brake sonrası mekanik sürüklenmeyi gösterir.

## NO_MOTION sonucu
Eğer:

```text
actual=80ms
detect=0ms
det=0
ADC=1023 -> 1023
```

ise PWM75 / 80ms altında dahi breakaway oluşmamıştır. Bir sonraki adımda PWM yükseltilir; P78 içinde otomatik olarak daha yüksek PWM denenmez.

## Önemli
P78 yalnız karakterizasyon içindir. Testten sonra uçuş sistemi için P71 baseline korunur; P78 uçuş firmware'i olarak kullanılmaz.
