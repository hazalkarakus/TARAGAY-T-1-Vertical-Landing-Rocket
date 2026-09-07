#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"
#include "Modules/Estimation/FullStateESKF/full_state_eskf.h"
#include "Modules/Sensors/SensorManager/sensor_manager.h"

static FullStateESKFData_t g_eskf;
static SensorData_t g_sensor;
static uint32_t g_now_us=1000000UL;
static float g_Rri_true[3][3];

const FullStateESKFData_t *FullStateESKF_GetDataPtr(void){return &g_eskf;}
const SensorData_t *SensorManager_GetDataPtr(void){return &g_sensor;}
uint32_t micros(void){return g_now_us;}
uint32_t millis(void){return g_now_us/1000UL;}
uint8_t Timebase_HasElapsedMs(uint32_t n,uint32_t p,uint32_t i){return (uint8_t)((uint32_t)(n-p)>=i);}
uint8_t Timebase_HasElapsedUs(uint32_t n,uint32_t p,uint32_t i){return (uint8_t)((uint32_t)(n-p)>=i);}
void Timebase_Init(void){}
uint8_t SolenoidOutput_GetAppliedMask(void){return 0U;}

static void mm(const float A[3][3],const float B[3][3],float C[3][3]){
    int i,j,k; for(i=0;i<3;i++)for(j=0;j<3;j++){C[i][j]=0;for(k=0;k<3;k++)C[i][j]+=A[i][k]*B[k][j];}
}
static void mt(const float A[3][3],float T[3][3]){int i,j;for(i=0;i<3;i++)for(j=0;j<3;j++)T[i][j]=A[j][i];}
static void rx(float d,float R[3][3]){float a=d*(float)M_PI/180,c=cosf(a),s=sinf(a);float q[3][3]={{1,0,0},{0,c,-s},{0,s,c}};memcpy(R,q,sizeof(q));}
static void ry(float d,float R[3][3]){float a=d*(float)M_PI/180,c=cosf(a),s=sinf(a);float q[3][3]={{c,0,s},{0,1,0},{-s,0,c}};memcpy(R,q,sizeof(q));}
static void rz(float d,float R[3][3]){float a=d*(float)M_PI/180,c=cosf(a),s=sinf(a);float q[3][3]={{c,-s,0},{s,c,0},{0,0,1}};memcpy(R,q,sizeof(q));}
static void mat_to_q(const float R[3][3],float *qw,float *qx,float *qy,float *qz){
    float tr=R[0][0]+R[1][1]+R[2][2],S;
    if(tr>0){S=sqrtf(tr+1.0f)*2;*qw=.25f*S;*qx=(R[2][1]-R[1][2])/S;*qy=(R[0][2]-R[2][0])/S;*qz=(R[1][0]-R[0][1])/S;}
    else if(R[0][0]>R[1][1]&&R[0][0]>R[2][2]){S=sqrtf(1+R[0][0]-R[1][1]-R[2][2])*2;*qw=(R[2][1]-R[1][2])/S;*qx=.25f*S;*qy=(R[0][1]+R[1][0])/S;*qz=(R[0][2]+R[2][0])/S;}
    else if(R[1][1]>R[2][2]){S=sqrtf(1+R[1][1]-R[0][0]-R[2][2])*2;*qw=(R[0][2]-R[2][0])/S;*qx=(R[0][1]+R[1][0])/S;*qy=.25f*S;*qz=(R[1][2]+R[2][1])/S;}
    else{S=sqrtf(1+R[2][2]-R[0][0]-R[1][1])*2;*qw=(R[1][0]-R[0][1])/S;*qx=(R[0][2]+R[2][0])/S;*qy=(R[1][2]+R[2][1])/S;*qz=.25f*S;}
}

static void set_pose(const float Rwr[3][3]){
    float Rwi[3][3],Rwit[3][3];
    mm(Rwr,g_Rri_true,Rwi); /* sensor/IMU -> rocket -> world */
    mt(Rwi,Rwit);
    mat_to_q(Rwi,&g_eskf.q_w,&g_eskf.q_x,&g_eskf.q_y,&g_eskf.q_z);
    /* static accelerometer measures world UP expressed in IMU frame */
    g_sensor.accel_x_filtered_g=Rwit[0][2];
    g_sensor.accel_y_filtered_g=Rwit[1][2];
    g_sensor.accel_z_filtered_g=Rwit[2][2];
    g_sensor.accel_filtered_norm_g=1.0f;
}
static void tick_pose(const float Rwr[3][3],uint32_t ms){
    uint32_t i; set_pose(Rwr);
    for(i=0;i<ms*2U;i++){
        g_now_us+=5000UL;g_eskf.last_public_output_timestamp_us=g_now_us-1000;g_sensor.imu_sample_timestamp_us=g_now_us-500;
        TaragayFlightLogic_Service200Hz();
    }
}
static float maxerr(const TaragayFlightLogicStatus_t *s){
    float r[3][3]={{s->mount_r00,s->mount_r01,s->mount_r02},{s->mount_r10,s->mount_r11,s->mount_r12},{s->mount_r20,s->mount_r21,s->mount_r22}};
    float e=0,d;int i,j;for(i=0;i<3;i++)for(j=0;j<3;j++){d=fabsf(r[i][j]-g_Rri_true[i][j]);if(d>e)e=d;}return e;
}
int main(void){
    float A[3][3],B[3][3],C[3][3],I[3][3]={{1,0,0},{0,1,0},{0,0,1}},Rtilt[3][3],Rroll[3][3],Rpitch[3][3];
    TaragayFlightLogicStatus_t s;
    /* true mounting: arbitrary yaw + small pitch/roll misalignment */
    rz(17,A);ry(4,B);rx(-3,C);mm(A,B,Rtilt);mm(Rtilt,C,g_Rri_true);
    memset(&g_eskf,0,sizeof(g_eskf));memset(&g_sensor,0,sizeof(g_sensor));
    g_eskf.initialized=1;g_eskf.healthy=1;g_eskf.covariance_integrity_ok=1;g_eskf.vertical_position_valid=1;g_eskf.origin_zeroed=1;g_eskf.lidar_reference_m=.2f;
    g_sensor.imu_valid=1;g_sensor.accel_filtered_norm_g=1.0f;
    TaragayFlightLogic_Init();

    tick_pose(I,2300);s=TaragayFlightLogic_GetStatus();
    printf("after upright phase=%u valid=%u upright=%u\n",s.mount_cal_phase,s.mount_cal_valid,s.mount_cal_upright_samples);
    if(s.mount_cal_phase!=2 || s.mount_cal_valid) return 2;

    ry(16,Rtilt);tick_pose(Rtilt,1700);s=TaragayFlightLogic_GetStatus();
    printf("after tilt phase=%u valid=%u fault=%u tilt=%.2f err=%.5f\n",s.mount_cal_phase,s.mount_cal_valid,s.mount_cal_fault,s.mount_cal_tilt_deg,maxerr(&s));
    printf("R= %.5f %.5f %.5f / %.5f %.5f %.5f / %.5f %.5f %.5f\n",s.mount_r00,s.mount_r01,s.mount_r02,s.mount_r10,s.mount_r11,s.mount_r12,s.mount_r20,s.mount_r21,s.mount_r22);
    if(!s.mount_cal_valid || s.mount_cal_fault || maxerr(&s)>.015f) return 3;

    rx(10,Rroll);tick_pose(Rroll,900);s=TaragayFlightLogic_GetStatus();
    printf("roll test corrected A/B=%.2f/%.2f events=%lu/%lu/%lu/%lu\n",s.input_pitch_deg,s.input_yaw_deg,(unsigned long)s.rcs_v1_event_count,(unsigned long)s.rcs_v3_event_count,(unsigned long)s.rcs_v5_event_count,(unsigned long)s.rcs_v7_event_count);
    if(fabsf(s.input_pitch_deg-10.0f)>.4f || fabsf(s.input_yaw_deg)>.4f) return 4;

    ry(-11,Rpitch);tick_pose(Rpitch,900);s=TaragayFlightLogic_GetStatus();
    printf("pitch test corrected A/B=%.2f/%.2f\n",s.input_pitch_deg,s.input_yaw_deg);
    if(fabsf(s.input_pitch_deg)>.4f || fabsf(s.input_yaw_deg+11.0f)>.4f) return 5;
    if((s.rcs_v1_event_count+s.rcs_v3_event_count+s.rcs_v5_event_count+s.rcs_v7_event_count)==0) return 6;
    puts("PASS R8R26 mount calibration + rocket-frame tilt transform");return 0;
}
