# TARAGAY-T1 P74 — Needle Valve PA0 Safe-Step Bench

## Amaç
P71 uçuş baseline'ını değiştirmeden ana iğne vana aktüatörünü yerel bir fiziksel butonla test etmek.
P74 yalnız bench firmware'idir; uçuş için kullanılmaz.

## P71 korunuyor
P74 doğrudan P71 kaynak ağacından üretildi. NRF komut protokolü kullanılmıyor ve P74'te NRF compile-time kapalıdır.
UART sadece STM32 -> PC diagnostik çıkışıdır; PC -> STM32 RX komutu gerekmez.

## Donanım
- PE9 konnektörü bağlı kalacak.
- Gaz/basınç bağlı OLMAYACAK.
- RCS güç/enerji hattı fiziksel olarak inhibit/disconnect tutulacak.
- Needle motor sürücüsü test için bağlı olabilir.
- Geçici test butonu: PA0 <-> 3.3 V. P74 PA0'ı internal pulldown input yapar.
- İstersen PA0 ile buton arasına 1k–4.7k seri direnç ekleyebilirsin.
- UART izleme: STM32 PA2(TX) -> USB-UART RX, GND -> GND. PA3 gerekli değil.

## Güvenlik
- NRF P74'te kapalı; aktüatör komutu NRF'den gelmez.
- Flight active olursa test anında abort/STOP.
- Preflight fault veya SystemMonitor actuator fault olursa abort/STOP.
- Needle fault olursa abort/STOP.
- Bir hareket sürerken PA0'a basmak local acil abort/STOP'tur.
- Her hareket 4 s timeout'ludur.
- OPEN sırasında ADC artarsa, CLOSE sırasında ADC azalırsa wrong-direction abort çalışır.
- RCS çıkışları bench supervisor tarafından ForceSafe tutulur.
- Test sadece %5 ve %10'a kadar gider; %25/%50/%100 yoktur.

## PA0 adım sırası
Sistem açıldıktan sonra SYS=1/0, flight=0, PE9 open=0, needle fault=0 bekle.

1. PA0 bas-bırak: CLOSED ZERO yakala. Motor hareket etmez.
2. PA0 bas-bırak: driver ENABLE. Motor hareket etmez.
3. PA0 bas-bırak: %5 OPEN. Yaklaşık 39 ADC / 0.20 tur. Hedefe kilitlenmesini bekle.
4. PA0 bas-bırak: %0 CLOSE. ZERO'ya kilitlenmesini bekle.
5. PA0 bas-bırak: %10 OPEN. Yaklaşık 78 ADC / 0.40 tur. Hedefe kilitlenmesini bekle.
6. PA0 bas-bırak: %0 CLOSE. ZERO'ya kilitlenince test PASS ve driver otomatik STOP olur.
7. PASS/ABORT sonrasında PA0 bas-bırak: test state'i başa alınır.

ÖNEMLİ: Motor hareket ederken PA0'a basarsan ilerlemez; güvenlik gereği anında ABORT/STOP yapar.

## Beklenen ADC yönü
Kapalı ZERO yaklaşık 1015–1023 civarıysa OPEN'da ADC düşmelidir.
Örnek zero=1023:
- %5 target yaklaşık 984
- %10 target yaklaşık 945

## UART monitor
```bat
python -u monitor_uart_p74_needle_pa0.py --port COM21 --log uart_p74_needle.txt
```

## Test bittikten sonra
P74'ü uçuş firmware'i olarak bırakma. Test tamamlanınca P71 baseline firmware'ine geri dön.
