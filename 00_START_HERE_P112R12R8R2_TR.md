# P112R12R8R2 — LiDAR I2C2 Bus-Clear Production Candidate / INERT

## Neden R12R8R2?

R12R8 ve aynı-binary R12R8R1 fiziksel loglarında LiDAR uzun süre çalıştıktan sonra kalıcı olarak `lidar_valid=0` oldu. R12R8R1 koşusunda dropout `t=26.931 s`'de başladı ve test sonuna kadar 59.872 s boyunca geri gelmedi.

Dropout anından sonra `lidar_reads=3659` değerinde tamamen durdu. Sadece bir LiDAR error/timeout oluşmasına rağmen recovery mekanizması sürekli döndü: test sonunda `recovery attempts=246`, `success=0`, `failures=82`. Recovery step maksimum süresi yaklaşık `35.945 ms`, LiDAR task maksimum süresi yaklaşık `36.179 ms` oldu.

Bu imza, yalnız `HAL_I2C_DeInit()+MX_I2C2_Init()` yapmanın elektriksel olarak sıkışmış I2C hattını açamadığını gösteriyor. R12R8R2, HAL I2C init'ten önce PB10=SCL ve PB11=SDA üzerinde bounded GPIO bus-clear yapar: SDA düşükse en fazla 9 SCL clock üretir, explicit STOP üretir, iki hattın da HIGH serbest kaldığını doğrular, I2C2 peripheral reset atar ve ardından normal `MX_I2C2_Init()` yoluna döner.

## Değişmeyenler

Production GNC, ESKF, P111, P110/P112 actuator core, TIM7, SD logger, UART protocol/field count, RCS/vent output mantığı ve R12R8 production freshness guard değiştirilmedi. Test stimulus/fault injection/no-motion hook eklenmedi.

## Test

Yalnız tamamen **basınçsız/inert bench**. Propellant/gaz/pyro/ignition/enerjik donanım ve RCS/vent fiziksel yükleri izole olsun.

CubeIDE:

1. Import project.
2. Clean Project.
3. Build Project.
4. Flash + Reset.
5. PE9 bağlı başlasın.
6. Monitoru çalıştır:

```bash
py monitor_uart_p112r12r8r2_lidar_bus_clear.py --port COM21 --duration 90
```

`READY_PE9_CONNECTED` görüldüğünde PE9'ı ayır. Sonra 90 s sisteme dokunma.

## Beklenen

En iyi sonuç: 90 s boyunca LiDAR dropout olmaması.

Eğer dropout oluşursa asıl R12R8R2 başarı kriteri, LiDAR'ın bounded recovery ile tekrar valid olmasıdır. Monitor `LIDAR RECOVERED` yazmalı; `lidar_rec_success` artmalı ve `lidar_reads` tekrar artmaya başlamalıdır. Recovery sırasında actuator request/PWM güvenli kalmalıdır.

Özellikle şunları gönder:

- oluşan `uart_p112r12r8r2_lidar_bus_clear.txt`
- CubeIDE build hatası varsa tam build output

Fiziksel motor hareketi, RCS çıkışı veya beklenmeyen fault olursa testi kesip logu gönder.
