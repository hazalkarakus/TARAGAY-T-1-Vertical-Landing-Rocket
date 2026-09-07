/*
 * Ucus_Bilgisayari.h
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

#ifndef Ucus_Bilgisayari_h_
#define Ucus_Bilgisayari_h_
#ifndef Ucus_Bilgisayari_COMMON_INCLUDES_
#define Ucus_Bilgisayari_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Ucus_Bilgisayari_COMMON_INCLUDES_ */

/* P49R1: generated model type declarations are embedded here so the
 * CubeIDE build does not depend on a separately discovered generated header.
 * The original Ucus_Bilgisayari_types.h is still kept in the project. */
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

#include "rtGetNaN.h"
#include <float.h>
#include <string.h>
#include <stddef.h>
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T TSamp;                        /* '<S1>/TSamp' */
  real_T V1;                           /* '<Root>/RCS_Denge_Kontrol' */
  real_T V3;                           /* '<Root>/RCS_Denge_Kontrol' */
  real_T V5;                           /* '<Root>/RCS_Denge_Kontrol' */
  real_T V7;                           /* '<Root>/RCS_Denge_Kontrol' */
  real_T Valve_Cmd;                    /* '<Root>/Otomatik_Gorev_Secimi' */
} B_Ucus_Bilgisayari_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T P;   /* '<Root>/RCS_Denge_Kontrol' */
  sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T Y;   /* '<Root>/RCS_Denge_Kontrol' */
  real_T UnitDelay_DSTATE;             /* '<Root>/Unit Delay' */
  real_T UnitDelay2_DSTATE;            /* '<Root>/Unit Delay2' */
  real_T UnitDelay4_DSTATE;            /* '<Root>/Unit Delay4' */
  real_T UnitDelay6_DSTATE;            /* '<Root>/Unit Delay6' */
  real_T UnitDelay8_DSTATE;            /* '<Root>/Unit Delay8' */
  real_T UnitDelay1_DSTATE;            /* '<Root>/Unit Delay1' */
  real_T UnitDelay3_DSTATE;            /* '<Root>/Unit Delay3' */
  real_T UD_DSTATE;                    /* '<S1>/UD' */
  real_T PrevY;                        /* '<Root>/Rate Limiter2' */
  real_T PrevY_c;                      /* '<Root>/Rate Limiter' */
  real_T PrevY_n;                      /* '<Root>/Rate Limiter1' */
  real_T valve_prev;                   /* '<Root>/MATLAB Function2' */
  real_T valve_filt;                   /* '<Root>/MATLAB Function2' */
  real_T F_meas_filt;                  /* '<Root>/Hover_Kontrol' */
  real_T valve_previous;               /* '<Root>/Hover_Kontrol' */
  uint32_T sample_count;               /* '<Root>/RCS_Denge_Kontrol' */
  uint32_T arm_count;                  /* '<Root>/Hover_Kontrol' */
  uint32_T hover_streak_count;         /* '<Root>/Hover_Kontrol' */
  uint32_T hover_best_count;           /* '<Root>/Hover_Kontrol' */
  uint32_T shortage_count;             /* '<Root>/Hover_Kontrol' */
  uint8_T gorev_p;                     /* '<Root>/Otomatik_Gorev_Secimi' */
  uint8_T brake_phase;                 /* '<Root>/MATLAB Function2' */
  uint8_T state_p;                     /* '<Root>/Hover_Kontrol' */
  boolean_T hard_fault;                /* '<Root>/RCS_Denge_Kontrol' */
  boolean_T landed_latch;              /* '<Root>/RCS_Denge_Kontrol' */
  boolean_T gorev_kilitli;             /* '<Root>/Otomatik_Gorev_Secimi' */
  boolean_T shortage_latch;            /* '<Root>/Hover_Kontrol' */
  boolean_T touchdown_latch;           /* '<Root>/Hover_Kontrol' */
  boolean_T capture_brake_active;      /* '<Root>/Hover_Kontrol' */
  boolean_T capture_brake_done;        /* '<Root>/Hover_Kontrol' */
} DW_Ucus_Bilgisayari_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Pitch;                        /* '<Root>/Pitch' */
  real_T Yaw;                          /* '<Root>/Yaw' */
  real_T yPOS;                         /* '<Root>/yPOS' */
  real_T XPOS;                         /* '<Root>/XPOS' */
  real_T Vy;                           /* '<Root>/Vy' */
  real_T vx;                           /* '<Root>/vx' */
  real_T zpos;                         /* '<Root>/zpos' */
  real_T zvel;                         /* '<Root>/zvel' */
  real_T ZposIMU;                      /* '<Root>/ZposIMU' */
  real_T m_guncel;                     /* '<Root>/m_guncel' */
  real_T P_main_bar;                   /* '<Root>/P_main_bar' */
  real_T Gercek_Itki_N;                /* '<Root>/Gercek_Itki_N' */
  real_T pitch_rate;                   /* '<Root>/pitch_rate' */
  real_T yaw_rate;                     /* '<Root>/yaw_rate' */
} ExtU_Ucus_Bilgisayari_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T V1;                           /* '<Root>/V1' */
  real_T V3;                           /* '<Root>/V3' */
  real_T V5;                           /* '<Root>/V5' */
  real_T V7;                           /* '<Root>/V7' */
  real_T Anaitki;                      /* '<Root>/Ana itki' */
} ExtY_Ucus_Bilgisayari_T;

/* Parameters (default storage) */
struct P_Ucus_Bilgisayari_T_ {
  real_T DiscreteDerivative_ICPrevScaled;
                              /* Mask Parameter: DiscreteDerivative_ICPrevScaled
                               * Referenced by: '<S1>/UD'
                               */
  real_T UnitDelay_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay'
                                        */
  real_T UnitDelay2_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay2'
                                        */
  real_T UnitDelay4_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay4'
                                        */
  real_T UnitDelay6_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay6'
                                        */
  real_T UnitDelay8_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay8'
                                        */
  real_T RateLimiter2_RisingLim;       /* Expression: 8
                                        * Referenced by: '<Root>/Rate Limiter2'
                                        */
  real_T RateLimiter2_FallingLim;      /* Expression: -4
                                        * Referenced by: '<Root>/Rate Limiter2'
                                        */
  real_T RateLimiter2_IC;              /* Expression: 0
                                        * Referenced by: '<Root>/Rate Limiter2'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 1
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: 0
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T UnitDelay1_InitialCondition;  /* Expression: 300
                                        * Referenced by: '<Root>/Unit Delay1'
                                        */
  real_T UnitDelay3_InitialCondition;  /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay3'
                                        */
  real_T RateLimiter_RisingLim;        /* Expression: 0.0698132
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  real_T RateLimiter_FallingLim;       /* Expression: -0.0698132
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  real_T Gain5_Gain;                   /* Expression: 180/pi
                                        * Referenced by: '<Root>/Gain5'
                                        */
  real_T RateLimiter1_RisingLim;       /* Expression: 0.0698132
                                        * Referenced by: '<Root>/Rate Limiter1'
                                        */
  real_T RateLimiter1_FallingLim;      /* Expression: -0.0698132
                                        * Referenced by: '<Root>/Rate Limiter1'
                                        */
  real_T RateLimiter1_IC;              /* Expression: 0
                                        * Referenced by: '<Root>/Rate Limiter1'
                                        */
  real_T Gain6_Gain;                   /* Expression: 180/pi
                                        * Referenced by: '<Root>/Gain6'
                                        */
  real_T Gain_Gain;                    /* Expression: 1
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Gain3_Gain;                   /* Expression: pi/180
                                        * Referenced by: '<Root>/Gain3'
                                        */
  real_T Gain4_Gain;                   /* Expression: pi/180
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T Gain2_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S1>/TSamp'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Ucus_Bilgisayari_T {
  const char_T *errorStatus;
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_Ucus_Bilgisayari_T Ucus_Bilgisayari_P;

/* Block signals (default storage) */
extern B_Ucus_Bilgisayari_T Ucus_Bilgisayari_B;

/* Block states (default storage) */
extern DW_Ucus_Bilgisayari_T Ucus_Bilgisayari_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Ucus_Bilgisayari_T Ucus_Bilgisayari_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Ucus_Bilgisayari_T Ucus_Bilgisayari_Y;

/* Model entry point functions */
extern void Ucus_Bilgisayari_initialize(void);
extern void Ucus_Bilgisayari_output(void);
extern void Ucus_Bilgisayari_update(void);
extern void Ucus_Bilgisayari_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Ucus_Bilgisayari_T *const Ucus_Bilgisayari_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Ucus_Bilgisayari'
 * '<S1>'   : 'Ucus_Bilgisayari/Discrete Derivative'
 * '<S2>'   : 'Ucus_Bilgisayari/Hover_Kontrol'
 * '<S3>'   : 'Ucus_Bilgisayari/MATLAB Function1'
 * '<S4>'   : 'Ucus_Bilgisayari/MATLAB Function2'
 * '<S5>'   : 'Ucus_Bilgisayari/Otomatik_Gorev_Secimi'
 * '<S6>'   : 'Ucus_Bilgisayari/RCS_Denge_Kontrol'
 */
#endif                                 /* Ucus_Bilgisayari_h_ */
