Bu proje TAM sistem projesidir: sensörler + scheduler + attitude/ESKF/RCS + SD logger + UART telemetry + nRF + tahliye servosu.
Tahliye servo ayarı: nRF 0 -> 1500 us (kapalı), nRF 1 -> 2500 us (yaklaşık +90 derece; mekanik olarak doğrulanmalı).
Sensörleri görmek için Debug > Live Expressions içinde örnekler:
IMU: imu_drv_connected, imu_sample_valid, sensor_gyro_x_dps, sensor_gyro_y_dps, sensor_gyro_z_dps, sensor_accel_x_g, sensor_accel_y_g, sensor_accel_z_g
Attitude: attitude_roll_deg, attitude_pitch_deg, attitude_yaw_deg, attitude_healthy
Baro: sensor_baro_valid, sensor_baro_pressure_pa, sensor_baro_temperature_c, sensor_baro_altitude_m
Lidar: lidar_connected, lidar_distance_m, lidar_filtered_distance_m
Servo/nRF: remote_rx_link_active, remote_rx_command, vent_servo_pulse_us, vent_servo_target_open
