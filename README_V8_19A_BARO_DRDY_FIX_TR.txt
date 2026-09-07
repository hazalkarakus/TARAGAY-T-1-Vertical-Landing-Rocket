TARAGAY-T1 V8.19A - BMP585 DRDY SOURCE FIX

Bulgu:
V8.19 testinde BMP585 SPI haberlesmesi ve konfigurasyonu basariliydi:
- CHIP_ID = 0x51
- config_ok = 1
- OSR_EFF = 0x99
- communication_error_count = 0

Fakat:
- bmp585_drdy_count = 0
- barometer_update_count = 0
- pressure = 0
- qualification baro rate = 0 Hz

Sebep:
V8.19 INT_STATUS (0x27) registerindeki DRDY bitini polling yapiyordu fakat
INT_SOURCE (0x15) registerinde drdy_data_reg_en biti etkinlestirilmemisti.

V8.19A:
- INT_SOURCE 0x15 bit0 = 1 yapar.
- Fiziksel INT pini gerektirmez; INT_STATUS polling devam eder.
- INT_SOURCE, ODR_CONFIG, OSR_CONFIG ve OSR_EFF read-back dogrulamalarini yapar.
- SensorQualification flags bit30 = DRDY source enabled.
- Barometre 200 Hz task tarafindan servis edilir, sadece gercek DRDY geldiginde
  yeni sample kabul edilir; hedef gercek sample hizi ~100 Hz.
- 200 sample ground calibration ile ~2 saniye sonra barometer_calibrated = 1.

Yeni Live Expressions:
bmp585_int_source_reg
bmp585_drdy_source_ok
bmp585_odr_config_reg
bmp585_osr_config_reg
bmp585_int_status

Beklenen:
bmp585_chip_id          = 81 (0x51)
bmp585_config_ok        = 1
bmp585_osr_eff          = 153 (0x99)
bmp585_int_source_reg   = 1
bmp585_drdy_source_ok   = 1
bmp585_drdy_count       artiyor (~100/s)
barometer_update_count  artiyor (~100/s)
sensor_qual_baro_rate_hz ~100
barometer_pressure_pa   ~ortam basinci
barometer_calibrated    = 1 (~2 s sonra)
barometer_healthy       = 1
sensor_qual_pass        = 1 (3 temiz pencere sonrasi)

Gaz/basincli sistem bagli olmadan sensor qualification testi yapin.
