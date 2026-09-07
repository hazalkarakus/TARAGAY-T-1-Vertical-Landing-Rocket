TGY V8.15C - RAM GUARD / CCM SD RING
=====================================

V8.15B ekranındaki flag=5/18/59, devasa sayaç ve imkansız float değerleri
estimator tuning değildir. Bu sürüm iki ihtimali doğrudan ayırır:

1) yanlış ELF / yanlış firmware eşleşmesi
2) main SRAM / stack marjı ve RAM corruption

FLASH MAGIC
-----------
v815c_flash_magic = 0x815C15C1
v815c_firmware_identity_ok = 1

Bu magic FLASH'tadır. Yanlışsa STM32 üzerinde V8.15C çalışmıyordur veya
debugger yanlış ELF kullanıyordur.

RAM MAGIC
---------
v815c_ram_magic = 0x815C15C2
v815c_ram_magic_ok = 1

Bu sonradan bozulursa gerçek RAM corruption vardır.

SD RING
-------
Eski:
256 x 288 = 73,728 byte MAIN SRAM
5.12 s backlog

Yeni:
128 x 288 = 36,864 byte CCMRAM
2.56 s backlog

SD writer hâlâ:
32 frame = 0.64 s

Dolayısıyla 2.56 s CPU ring marjı yeterlidir.

SDIO DMA buffer'ları main SRAM'de bırakıldı.
CCMRAM'e sadece CPU capture ring taşındı.

Beklenen:
sd_logger_ring_struct_bytes = 36864
sd_logger_ring_address      = 0x1000....
sd_logger_ring_end_address  < 0x10010000
v815c_sd_ring_in_ccm        = 1

MAIN SRAM / STACK GAP
---------------------
v815c_bss_end_address
v815c_msp_address
v815c_main_sram_gap_bytes
v815c_main_sram_gap_min_bytes

gap = current MSP - end of BSS.

0 kesin FAIL.
Birkaç KB'den belirgin büyük bir sayı istiyoruz.
Ring taşıması nedeniyle V8.15B'ye göre ciddi marj oluşmalı.

BASIC SANITY
------------
v815c_estimator_basic_sanity_ok = 1

Bu ESKF'nin 0/1 flaglerinin gerçekten 0/1 kaldığını doğrudan kontrol eder.

İLK TEST
--------
Motor PSU kapalı.
Terminate eski debug.
V8.15C import -> Clean -> Build -> Debug -> Resume.

ÖNCE SADECE:
v815c_flash_magic
v815c_firmware_identity_ok
v815c_ram_magic
v815c_ram_magic_ok

sd_logger_ring_struct_bytes
sd_logger_ring_address
sd_logger_ring_end_address
v815c_sd_ring_in_ccm

v815c_bss_end_address
v815c_msp_address
v815c_main_sram_gap_bytes
v815c_main_sram_gap_min_bytes

v815c_estimator_basic_sanity_ok

cpu_load_percent
cpu_idle_percent

Sonra estimator:
attitude_update_count
full_eskf_predict_count
full_eskf_public_output_count
full_eskf_covariance_predict_count

full_eskf_enabled
full_eskf_initialized
full_eskf_healthy
full_eskf_shadow_mode
full_eskf_baro_fresh
full_eskf_baro_reference_ready
full_eskf_lidar_fresh
full_eskf_lidar_reference_ready
full_eskf_origin_zeroed
full_eskf_origin_zero_count
full_eskf_horizontal_position_valid
full_eskf_vertical_position_valid

Baro yokken hedef:
baro_fresh = 0
baro_reference_ready = 0

LIDAR:
fresh = 1
reference_ready = 1

origin_zeroed = 1
origin_zero_count = 1

horizontal_valid = 0
vertical_valid = 1
