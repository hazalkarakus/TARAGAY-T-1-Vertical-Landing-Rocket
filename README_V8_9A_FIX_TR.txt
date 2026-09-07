TGY V8.9A FIX
=============

V8.9 BUILD HATALARI DÜZELTİLDİ.

1) remote_control.c
-------------------
V8.9 üretiminde şu iki reset satırı yanlışlıkla GLOBAL scope'a düşmüştü:

    remote_rx_irq_count = 0UL;
    remote_rx_last_packet_age_ms = 0xFFFFFFFFUL;

Bu yüzden derleyici bunları implicit-int global tanımlar gibi yorumlayıp
header'daki volatile uint32_t deklarasyonlarıyla çakıştırıyordu.

V8.9A:
- yanlış global atamalar kaldırıldı
- gerçek volatile uint32_t tanımları korundu
- resetler RemoteControl_Init() içine taşındı

2) app_tasks.c
--------------
V8.9 monitor kodu:

    APP_NRF24_CHANNEL

makrosunu kullanıyordu ama app_config.h doğrudan include edilmemişti.

V8.9A:
    #include "Common/app_config.h"

eklendi.

Ekrandaki unused-function warning'leri:
- IMU_StartRawDMARead defined but not used
- IMU_ReadMultiBlocking defined but not used

build'i durdurmaz; şu aşamada temizlenmedi.

BUILD
-----
1) Eski V8.9 debug session -> Terminate
2) V8.9A FIX import
3) Project -> Clean
4) Build Project

Build geçince:
- SD kart boot öncesi takılı
- motor PSU kapalı
- Debug -> Resume

İlk NRF monitor Live Expressions:

v89_nrf_initialized
v89_nrf_connected
v89_nrf_mode
v89_nrf_rf_ch_reg
v89_nrf_rf_setup_reg
v89_nrf_hw_ok

v89_nrf_task_delta_100ms
v89_nrf_task_max_exec_us
v89_nrf_task_overrun_count
v89_nrf_task_deadline_miss_count

v89_spi3_ok
v89_spi3_fault
v89_spi3_transaction_count
v89_spi3_error_count
v89_spi3_timeout_count

v89_scheduler_ok
v89_integration_ok

Beklenen transmitter olmadan:
initialized = 1
connected = 1
mode = RX
rf_ch = 76
rf_setup = 6
nrf_hw_ok = 1

NRF task /100ms ~= 20
SPI3 fault/error/timeout = 0

remote_link_active = 0 ve packet_count = 0 transmitter yoksa NORMALDIR.
