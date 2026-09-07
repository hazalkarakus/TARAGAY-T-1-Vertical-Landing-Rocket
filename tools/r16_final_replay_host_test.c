#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "Modules/Control/TaragayFlightLogic/taragay_flight_logic.h"

/* Step100Hz() only needs this physical-state telemetry hook under the active
 * project feature macros. All other hardware services live in discarded
 * function sections in this host replay. */
typedef struct FullStateESKFData_struct FullStateESKFData_t;
typedef struct SensorData_struct SensorData_t;
uint8_t SolenoidOutput_GetAppliedMask(void) { return 0U; }
const FullStateESKFData_t *FullStateESKF_GetDataPtr(void) { return 0; }
const SensorData_t *SensorManager_GetDataPtr(void) { return 0; }
uint32_t micros(void) { return 0; }
uint8_t PreflightTrigger_IsFlightActive(void) { return 0; }
uint8_t PreflightTrigger_HasFault(void) { return 0; }
uint8_t SystemMonitor_IsActuatorFaultActive(void) { return 0; }
uint8_t NeedleValveAutonomousControl_HasFault(void) { return 0; }
float needle_valve_position_turns = 0.0f;



int main(int argc, char **argv)
{
    FILE *fp;
    char line[1024];
    unsigned row = 0U;
    unsigned state_mismatch = 0U;
    unsigned safety_mismatch = 0U;
    float max_valve_err = 0.0f, max_bhat_err = 0.0f, max_auth_err = 0.0f, max_afilt_err = 0.0f;
    double sum_valve2 = 0.0, sum_bhat2 = 0.0, sum_auth2 = 0.0, sum_afilt2 = 0.0;
    float first_state_time[9];
    unsigned i;

    if (argc != 2) {
        fprintf(stderr, "usage: %s r16e6_replay.csv\n", argv[0]);
        return 2;
    }
    for (i=0U;i<9U;i++) first_state_time[i] = NAN;
    fp = fopen(argv[1], "r");
    if (!fp) { perror("fopen"); return 2; }
    if (!fgets(line, sizeof(line), fp)) { fprintf(stderr,"empty csv\n"); return 2; }

    TaragayFlightLogic_Init();

    while (fgets(line, sizeof(line), fp)) {
        TaragayFlightLogicInput_t in;
        TaragayFlightLogicStatus_t s;
        double t,z,auth_ref,turns,release,valve_ref,afilt_ref,bhat_ref,vz,state_ref,itki,safety_ref,thrust;
        int n = sscanf(line,
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &t,&z,&auth_ref,&turns,&release,&valve_ref,&afilt_ref,&bhat_ref,&vz,&state_ref,&itki,&safety_ref,&thrust);
        float e;
        (void)itki; (void)thrust;
        if (n != 13) { fprintf(stderr,"parse error row %u: %s",row+2U,line); return 2; }
        memset(&in,0,sizeof(in));
        in.z_cg_m=(float)z;
        in.vz_mps=(float)vz;
        in.turns_actual=(float)turns;
        in.health_ok=1.0f;
        in.release_event=(float)release;
        in.mass_kg=27.5f;
        /* Isolate vertical replay. Horizontal/RCS inputs are valid zeros. */
        in.x_m=0.0f; in.y_m=0.0f; in.vx_mps=0.0f; in.vy_mps=0.0f;
        in.pitch_rad=0.0f; in.yaw_rad=0.0f;
        in.pitch_rate_rad_s=0.0f; in.yaw_rate_rad_s=0.0f;

        TaragayFlightLogic_Step100Hz(&in);
        s=TaragayFlightLogic_GetStatus();

        if ((unsigned)s.mission_state < 9U && isnan(first_state_time[s.mission_state]))
            first_state_time[s.mission_state]=(float)t;

        if (s.mission_state != (uint8_t)llround(state_ref)) {
            if (state_mismatch < 20U) printf("STATE DIFF t=%.2f fw=%u ref=%u valve=%.4f/%.4f bhat=%.3f/%.3f auth=%.3f/%.3f turns=%.3f\n",
                t,s.mission_state,(unsigned)llround(state_ref),s.valve_cmd,(float)valve_ref,s.b_hat,(float)bhat_ref,s.authority_ratio,(float)auth_ref,(float)turns);
            state_mismatch++;
        }
        if (s.safety_trip != (uint8_t)(safety_ref > 0.5)) safety_mismatch++;

        e=fabsf(s.valve_cmd-(float)valve_ref); if(e>max_valve_err)max_valve_err=e; sum_valve2+=(double)e*e;
        e=fabsf(s.b_hat-(float)bhat_ref);
        if (e > 0.20f && row < 140U) printf("BHAT DIFF t=%.2f fw=%.3f ref=%.3f state=%u valve=%.4f turns=%.3f af=%.3f\n",t,s.b_hat,(float)bhat_ref,s.mission_state,s.valve_cmd,(float)turns,s.a_filt);
        if(e>max_bhat_err)max_bhat_err=e; sum_bhat2+=(double)e*e;
        e=fabsf(s.authority_ratio-(float)auth_ref); if(e>max_auth_err)max_auth_err=e; sum_auth2+=(double)e*e;
        e=fabsf(s.a_filt-(float)afilt_ref); if(e>max_afilt_err)max_afilt_err=e; sum_afilt2+=(double)e*e;
        row++;
    }
    fclose(fp);

    if (row == 0U) return 2;
    printf("R16 FINAL HOST REPLAY rows=%u\n",row);
    printf("state_mismatch=%u (%.3f%%)\n",state_mismatch,100.0*(double)state_mismatch/(double)row);
    printf("safety_mismatch=%u\n",safety_mismatch);
    printf("valve RMSE=%.6f max=%.6f\n",sqrt(sum_valve2/row),max_valve_err);
    printf("b_hat RMSE=%.6f max=%.6f\n",sqrt(sum_bhat2/row),max_bhat_err);
    printf("authority_ratio RMSE=%.6f max=%.6f\n",sqrt(sum_auth2/row),max_auth_err);
    printf("a_filt RMSE=%.6f max=%.6f\n",sqrt(sum_afilt2/row),max_afilt_err);
    printf("state times: INIT %.2f SELF %.2f PRE %.2f READY %.2f ASC %.2f CAP %.2f HOVER %.2f DEG %.2f SAFE %.2f\n",
        first_state_time[0],first_state_time[1],first_state_time[2],first_state_time[3],first_state_time[4],
        first_state_time[5],first_state_time[6],first_state_time[7],first_state_time[8]);

    /* Strict enough to expose semantic port errors, tolerant of 1-frame Unit Delay transition timing. */
    if (state_mismatch > 2U || safety_mismatch != 0U ||
        max_valve_err > 0.035f || max_bhat_err > 0.35f ||
        max_auth_err > 0.12f || max_afilt_err > 0.08f) {
        puts("R16 FINAL HOST REPLAY: CHECK");
        return 1;
    }
    puts("R16 FINAL HOST REPLAY: PASS");
    return 0;
}
