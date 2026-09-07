# TARAGAY-T1 R8R35R3R7R1 — NO RUNTIME HAL_DELAY

Amaç: App_Run başladıktan sonra hiçbir `HAL_Delay()` çağrısının uçuş döngüsünü bloke edememesi.

Değişiklikler:
- NRF24 `SetRxMode` / `SetTxMode` içindeki 2 ms `HAL_Delay` kaldırıldı.
- Kullanılmayan legacy senkron `NRF24_SendPayload()` polling döngüsü non-blocking async enqueue'a çevrildi.
- STM32 HAL'in weak `HAL_Delay()` fonksiyonu güçlü bir runtime guard ile override edildi.
- App_Init sonunda runtime guard aktif olur. Bundan sonra herhangi bir gizli/vendor `HAL_Delay()` çağrısı anında geri döner ve violation sayacını artırır.
- Boot sırasında datasheet gerektiren sensör/SD power-reset settling davranışı korunur. Bu beklemeler uçuş scheduler'ı başlamadan öncedir.
- SDIO motor-EMI R3R7 diagnostic alanları ve R3R6 transient-start retry davranışı değişmedi.
- Needle/RCS/ESKF/TaragayFlightLogic kontrol otoriteleri değiştirilmedi.

Not: Bu revizyon 'runtime blocking = 0' hedefidir. Boot-time donanım beklemelerini körlemesine silmek IMU/BMP585/SD kart başlangıcını bozabileceği için onlar uçuş yolundan ayrı tutulmuştur.
