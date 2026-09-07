#ifndef P43_HOST_STUB_APP_TASKS_H
#define P43_HOST_STUB_APP_TASKS_H

void Task_IMU_1kHz(void);
void Task_BarometerValveHealth_200Hz(void);
void Task_Lidar_Service_1kHz(void);
void Task_NRFMonitor_200Hz(void);
void Task_FullESKFCorrection_200Hz(void);
void Task_FullESKFCovariance_25Hz(void);
void Task_IntegrationMonitor_10Hz(void);

#endif
