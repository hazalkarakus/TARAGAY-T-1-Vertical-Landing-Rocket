#ifndef APP_SERVICES_SENSOR_QUALIFICATION_H
#define APP_SERVICES_SENSOR_QUALIFICATION_H

#include <stdint.h>

typedef enum
{
    SENSOR_QUAL_STATE_WARMUP = 0,
    SENSOR_QUAL_STATE_QUALIFYING = 1,
    SENSOR_QUAL_STATE_PASS = 2,
    SENSOR_QUAL_STATE_FAIL = 3
} SensorQualificationState_t;

void SensorQualification_Init(void);
void SensorQualification_Update10Hz(void);

extern volatile uint8_t sensor_qual_state;
extern volatile uint8_t sensor_qual_pass;
extern volatile uint8_t sensor_qual_good_windows;
extern volatile uint32_t sensor_qual_flags;
extern volatile float sensor_qual_imu_rate_hz;
extern volatile float sensor_qual_baro_rate_hz;
extern volatile float sensor_qual_lidar_rate_hz;

#endif
