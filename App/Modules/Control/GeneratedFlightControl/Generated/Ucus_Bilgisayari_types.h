/*
 * Ucus_Bilgisayari_types.h
 *
 * Code generation for model "Ucus_Bilgisayari".
 *
 * Model version              : 1.214
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C source code generated on : Fri Aug 21 15:46:51 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Ucus_Bilgisayari_types_h_
#define Ucus_Bilgisayari_types_h_
#include "rtwtypes.h"

/* Custom Type definition for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
#ifndef struct_tag_sXzzr8I9NVxmx2gyMIcZCVE
#define struct_tag_sXzzr8I9NVxmx2gyMIcZCVE

struct tag_sXzzr8I9NVxmx2gyMIcZCVE
{
  real_T prev_angle;
  real_T filtered_rate;
  real_T filtered_trend;
  boolean_T rate_initialized;
  boolean_T trend_initialized;
  uint8_T mode;
  uint8_T phase;
  int8_T desired;
  int8_T applied;
  int8_T pending;
  boolean_T event_active;
  boolean_T event_armed;
  boolean_T hard_latched;
  boolean_T brake_used;
  int8_T hazard_sign;
  boolean_T dead_active;
  uint16_T dead_age;
  uint16_T off_age;
  uint16_T event_age;
  uint16_T event_end_age;
  uint16_T safe_age;
  uint16_T brake_age;
  uint16_T window_age;
  uint8_T reversal_count;
  uint16_T event_window_age;
  uint8_T event_count;
  uint8_T pred_count;
  real_T target;
  real_T stopping;
  real_T braking_distance;
  real_T switch_error;
  boolean_T track_active;
  int8_T track_sign;
  uint16_T track_age;
  uint16_T track_pulse_n;
  uint16_T track_cooldown;
};

#endif                                 /* struct_tag_sXzzr8I9NVxmx2gyMIcZCVE */

#ifndef typedef_sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T
#define typedef_sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T

typedef struct tag_sXzzr8I9NVxmx2gyMIcZCVE sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T;

#endif                             /* typedef_sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T */

/* Parameters (default storage) */
typedef struct P_Ucus_Bilgisayari_T_ P_Ucus_Bilgisayari_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_Ucus_Bilgisayari_T RT_MODEL_Ucus_Bilgisayari_T;

#endif                                 /* Ucus_Bilgisayari_types_h_ */
