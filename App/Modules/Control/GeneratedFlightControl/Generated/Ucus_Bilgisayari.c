/*
 * Ucus_Bilgisayari.c
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

#include "Ucus_Bilgisayari.h"
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include <math.h>
#include "Ucus_Bilgisayari_private.h"
#include <string.h>
#include "rt_defines.h"
#define Ucus_Bilgisayari_period        (0.01)

/* Block signals (default storage) */
B_Ucus_Bilgisayari_T Ucus_Bilgisayari_B;

/* Block states (default storage) */
DW_Ucus_Bilgisayari_T Ucus_Bilgisayari_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Ucus_Bilgisayari_T Ucus_Bilgisayari_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Ucus_Bilgisayari_T Ucus_Bilgisayari_Y;

/* Real-time model */
static RT_MODEL_Ucus_Bilgisayari_T Ucus_Bilgisayari_M_;
RT_MODEL_Ucus_Bilgisayari_T *const Ucus_Bilgisayari_M = &Ucus_Bilgisayari_M_;

/* Forward declaration for local functions */
static void Ucus_Bilgisayari_initAxis(uint16_T minOffN,
  sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A);
static void Ucus_Bilgisayari_updateRates(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  real_T angle_deg, real_T gyro_rate_dps, real_T Ts, real_T rateAlpha, real_T
  trendAlpha, real_T *rate_dps, real_T *trend_dps);
static void Ucus_Bilgisayar_ageAxisCounters(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  uint16_T windowN);
static void Ucus_Bilgisayari_startEvent(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  int8_T hazardSign, boolean_T hardLimit, uint8_T maxEvents, boolean_T *started,
  boolean_T *eventFault);
static void Ucus_Bilgisayari_finishEvent(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  boolean_T immediateRearm, uint16_T minOffN);
static boolean_T Ucus_Bilgisayari_selectAxis(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  real_T angle, real_T rate, real_T pred, boolean_T predValid, real_T noFire,
  real_T limitDeg, real_T safeTarget, real_T alpha, real_T brakeMargin, real_T
  brakeDelayS, real_T brakeDone, real_T predHyst, uint8_T predConfirmN, real_T
  sub5ReactionS, real_T sub5MinOutwardDps, real_T sub5GrowthMarginDeg, uint16_T
  minCorrN, uint16_T maxBrakeN, uint16_T rearmN, uint16_T hardRearmN, uint8_T
  maxEvents, uint16_T minOffN);
static void Ucus_Bil_applyReferenceTracking(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  real_T angle, real_T rate, real_T refAngle, real_T noFireDeg, real_T
  maxAbsAngle, real_T startDb, real_T stopDb, real_T lookahead, real_T rateFar,
  real_T rateMid, real_T rateNear, uint16_T cooldownFarN, uint16_T cooldownMidN,
  uint16_T cooldownNearN, uint16_T pulseSmallN, uint16_T pulseMedN, uint16_T
  pulseLargeN);
static boolean_T Ucus_Bilgisay_processAxisOutput(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T
  *A, uint16_T deadN, uint16_T minOffN, uint8_T maxReversals);

/* Function for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
static void Ucus_Bilgisayari_initAxis(uint16_T minOffN,
  sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A)
{
  /* '<S6>:1:680' */
  A->prev_angle = 0.0;

  /* '<S6>:1:682' */
  A->filtered_rate = 0.0;

  /* '<S6>:1:683' */
  A->filtered_trend = 0.0;

  /* '<S6>:1:685' */
  A->rate_initialized = false;

  /* '<S6>:1:686' */
  A->trend_initialized = false;

  /* '<S6>:1:688' */
  A->mode = 0U;

  /* '<S6>:1:689' */
  A->phase = 0U;

  /* '<S6>:1:691' */
  A->desired = 0;

  /* '<S6>:1:692' */
  A->applied = 0;

  /* '<S6>:1:693' */
  A->pending = 0;

  /* '<S6>:1:695' */
  A->event_active = false;

  /* '<S6>:1:696' */
  A->event_armed = true;

  /* '<S6>:1:698' */
  A->hard_latched = false;

  /* '<S6>:1:699' */
  A->brake_used = false;

  /* '<S6>:1:701' */
  A->hazard_sign = 0;

  /* '<S6>:1:703' */
  A->dead_active = false;

  /* '<S6>:1:704' */
  A->dead_age = 0U;

  /* '<S6>:1:706' */
  A->off_age = minOffN;

  /* '<S6>:1:708' */
  A->event_age = 0U;

  /* '<S6>:1:709' */
  A->event_end_age = MAX_uint16_T;

  /* '<S6>:1:711' */
  A->safe_age = 0U;

  /* '<S6>:1:712' */
  A->brake_age = 0U;

  /* '<S6>:1:714' */
  A->window_age = 0U;

  /* '<S6>:1:716' */
  A->reversal_count = 0U;

  /* '<S6>:1:718' */
  A->event_window_age = 0U;

  /* '<S6>:1:719' */
  A->event_count = 0U;

  /* '<S6>:1:721' */
  A->pred_count = 0U;

  /* '<S6>:1:723' */
  A->target = 0.0;

  /* '<S6>:1:724' */
  A->stopping = 0.0;

  /* '<S6>:1:725' */
  A->braking_distance = 0.0;

  /* '<S6>:1:726' */
  A->switch_error = 0.0;

  /* '<S6>:1:728' */
  A->track_active = false;

  /* '<S6>:1:729' */
  A->track_sign = 0;

  /* '<S6>:1:731' */
  A->track_age = 0U;

  /* '<S6>:1:732' */
  A->track_pulse_n = 0U;

  /* '<S6>:1:733' */
  A->track_cooldown = 0U;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T tmp;
  int32_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
static void Ucus_Bilgisayari_updateRates(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  real_T angle_deg, real_T gyro_rate_dps, real_T Ts, real_T rateAlpha, real_T
  trendAlpha, real_T *rate_dps, real_T *trend_dps)
{
  real_T delta;
  if (!A->rate_initialized) {
    /* '<S6>:1:763' */
    /* '<S6>:1:765' */
    A->filtered_rate = gyro_rate_dps;

    /* '<S6>:1:766' */
    A->rate_initialized = true;
  } else {
    /* '<S6>:1:770' */
    A->filtered_rate += (gyro_rate_dps - A->filtered_rate) * rateAlpha;
  }

  if (!A->trend_initialized) {
    /* '<S6>:1:776' */
    /* '<S6>:1:778' */
    A->prev_angle = angle_deg;

    /* '<S6>:1:780' */
    A->filtered_trend = 0.0;

    /* '<S6>:1:782' */
    A->trend_initialized = true;
  } else {
    /* '<S6>:1:786' */
    delta = angle_deg - A->prev_angle;
    if (delta > 180.0) {
      /* '<S6>:1:789' */
      /* '<S6>:1:791' */
      delta -= 360.0;
    } else if (delta < -180.0) {
      /* '<S6>:1:793' */
      /* '<S6>:1:795' */
      delta += 360.0;
    }

    /* '<S6>:1:799' */
    /* '<S6>:1:801' */
    A->filtered_trend += (delta / Ts - A->filtered_trend) * trendAlpha;

    /* '<S6>:1:805' */
    A->prev_angle = angle_deg;
  }

  /* '<S6>:1:809' */
  *rate_dps = A->filtered_rate;

  /* '<S6>:1:810' */
  *trend_dps = A->filtered_trend;
}

/* Function for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
static void Ucus_Bilgisayar_ageAxisCounters(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  uint16_T windowN)
{
  uint32_T tmp;
  uint16_T x;

  /* '<S6>:1:838' */
  x = A->event_end_age;
  if (A->event_end_age < 65535) {
    /* '<S6>:1:2164' */
    /* '<S6>:1:2166' */
    tmp = A->event_end_age + 1U;
    if (A->event_end_age + 1U > 65535U) {
      tmp = 65535U;
    }

    x = (uint16_T)tmp;
  }

  A->event_end_age = x;
  if (A->event_active) {
    /* '<S6>:1:841' */
    /* '<S6>:1:843' */
    x = A->event_age;
    if (A->event_age < 65535) {
      /* '<S6>:1:2164' */
      /* '<S6>:1:2166' */
      tmp = A->event_age + 1U;
      if (A->event_age + 1U > 65535U) {
        tmp = 65535U;
      }

      x = (uint16_T)tmp;
    }

    A->event_age = x;
  }

  if (A->phase == 2) {
    /* '<S6>:1:848' */
    /* '<S6>:1:850' */
    x = A->brake_age;
    if (A->brake_age < 65535) {
      /* '<S6>:1:2164' */
      /* '<S6>:1:2166' */
      tmp = A->brake_age + 1U;
      if (A->brake_age + 1U > 65535U) {
        tmp = 65535U;
      }

      x = (uint16_T)tmp;
    }

    A->brake_age = x;
  }

  if (A->phase == 4) {
    /* '<S6>:1:855' */
    /* '<S6>:1:857' */
    x = A->safe_age;
    if (A->safe_age < 65535) {
      /* '<S6>:1:2164' */
      /* '<S6>:1:2166' */
      tmp = A->safe_age + 1U;
      if (A->safe_age + 1U > 65535U) {
        tmp = 65535U;
      }

      x = (uint16_T)tmp;
    }

    A->safe_age = x;
  }

  if ((A->applied == 0) && (!A->dead_active)) {
    /* '<S6>:1:862' */
    /* '<S6>:1:863' */
    /* '<S6>:1:865' */
    x = A->off_age;
    if (A->off_age < 65535) {
      /* '<S6>:1:2164' */
      /* '<S6>:1:2166' */
      tmp = A->off_age + 1U;
      if (A->off_age + 1U > 65535U) {
        tmp = 65535U;
      }

      x = (uint16_T)tmp;
    }

    A->off_age = x;
  } else {
    /* '<S6>:1:870' */
    A->off_age = 0U;
  }

  /* '<S6>:1:874' */
  x = A->window_age;
  if (A->window_age < 65535) {
    /* '<S6>:1:2164' */
    /* '<S6>:1:2166' */
    tmp = A->window_age + 1U;
    if (A->window_age + 1U > 65535U) {
      tmp = 65535U;
    }

    x = (uint16_T)tmp;
  }

  A->window_age = x;
  if (x >= windowN) {
    /* '<S6>:1:877' */
    /* '<S6>:1:879' */
    A->window_age = 0U;

    /* '<S6>:1:880' */
    A->reversal_count = 0U;
  }

  /* '<S6>:1:884' */
  x = A->event_window_age;
  if (A->event_window_age < 65535) {
    /* '<S6>:1:2164' */
    /* '<S6>:1:2166' */
    tmp = A->event_window_age + 1U;
    if (A->event_window_age + 1U > 65535U) {
      tmp = 65535U;
    }

    x = (uint16_T)tmp;
  }

  A->event_window_age = x;
  if (x >= windowN) {
    /* '<S6>:1:887' */
    /* '<S6>:1:889' */
    A->event_window_age = 0U;

    /* '<S6>:1:890' */
    A->event_count = 0U;
  }
}

/* Function for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
static void Ucus_Bilgisayari_startEvent(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  int8_T hazardSign, boolean_T hardLimit, uint8_T maxEvents, boolean_T *started,
  boolean_T *eventFault)
{
  uint32_T tmp;

  /* '<S6>:1:1568' */
  *started = false;

  /* '<S6>:1:1569' */
  *eventFault = false;
  if (hazardSign != 0) {
    /* '<S6>:1:1575' */
    tmp = A->event_count + 1U;
    if (A->event_count + 1U > 255U) {
      tmp = 255U;
    }

    A->event_count = (uint8_T)tmp;
    if (A->event_count > maxEvents) {
      /* '<S6>:1:1578' */
      /* '<S6>:1:1580' */
      *eventFault = true;
    } else {
      /* '<S6>:1:1586' */
      A->event_active = true;

      /* '<S6>:1:1587' */
      A->event_armed = false;

      /* '<S6>:1:1589' */
      A->hard_latched = hardLimit;

      /* '<S6>:1:1590' */
      A->brake_used = false;

      /* '<S6>:1:1592' */
      A->hazard_sign = hazardSign;

      /* '<S6>:1:1594' */
      A->phase = 1U;

      /* '<S6>:1:1595' */
      A->mode = 1U;

      /* '<S6>:1:1597' */
      A->event_age = 0U;

      /* '<S6>:1:1598' */
      A->pred_count = 0U;

      /* '<S6>:1:1600' */
      if (hazardSign > 0) {
        /* '<S6>:1:2084' */
        /* '<S6>:1:2086' */
        A->desired = -1;
      } else if (hazardSign < 0) {
        /* '<S6>:1:2088' */
        /* '<S6>:1:2090' */
        A->desired = 1;
      } else {
        /* '<S6>:1:2094' */
        A->desired = 0;
      }

      /* '<S6>:1:1604' */
      *started = true;
    }
  } else {
    /* '<S6>:1:1571' */
  }
}

/* Function for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
static void Ucus_Bilgisayari_finishEvent(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  boolean_T immediateRearm, uint16_T minOffN)
{
  /* '<S6>:1:1613' */
  A->event_active = false;

  /* '<S6>:1:1615' */
  A->hard_latched = false;

  /* '<S6>:1:1616' */
  A->brake_used = false;

  /* '<S6>:1:1618' */
  A->hazard_sign = 0;

  /* '<S6>:1:1620' */
  A->desired = 0;

  /* '<S6>:1:1622' */
  A->pred_count = 0U;

  /* '<S6>:1:1624' */
  A->event_end_age = 0U;

  /* '<S6>:1:1626' */
  A->safe_age = 0U;

  /* '<S6>:1:1627' */
  A->event_age = 0U;

  /* '<S6>:1:1628' */
  A->brake_age = 0U;

  /* '<S6>:1:1630' */
  A->target = 0.0;

  /* '<S6>:1:1631' */
  A->switch_error = 0.0;

  /* '<S6>:1:1633' */
  A->track_active = false;

  /* '<S6>:1:1634' */
  A->track_sign = 0;

  /* '<S6>:1:1636' */
  A->track_age = 0U;

  /* '<S6>:1:1637' */
  A->track_pulse_n = 0U;

  /* '<S6>:1:1638' */
  A->track_cooldown = 0U;
  if (immediateRearm) {
    /* '<S6>:1:1640' */
    /* '<S6>:1:1642' */
    A->phase = 0U;

    /* '<S6>:1:1643' */
    A->event_armed = true;

    /* '<S6>:1:1644' */
    A->mode = 0U;
  } else {
    /* '<S6>:1:1648' */
    A->phase = 4U;

    /* '<S6>:1:1649' */
    A->event_armed = false;

    /* '<S6>:1:1650' */
    A->mode = 4U;
  }

  if (A->applied == 0) {
    /* '<S6>:1:1654' */
    /* '<S6>:1:1656' */
    A->off_age = minOffN;
  }
}

/* Function for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
static boolean_T Ucus_Bilgisayari_selectAxis(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  real_T angle, real_T rate, real_T pred, boolean_T predValid, real_T noFire,
  real_T limitDeg, real_T safeTarget, real_T alpha, real_T brakeMargin, real_T
  brakeDelayS, real_T brakeDone, real_T predHyst, uint8_T predConfirmN, real_T
  sub5ReactionS, real_T sub5MinOutwardDps, real_T sub5GrowthMarginDeg, uint16_T
  minCorrN, uint16_T maxBrakeN, uint16_T rearmN, uint16_T hardRearmN, uint8_T
  maxEvents, uint16_T minOffN)
{
  real_T absAngle;
  real_T absRate;
  real_T reactionPred;
  real_T speedAfterDelay;
  real_T tmp_0;
  uint32_T tmp;
  int8_T threatSign;
  boolean_T eventFault;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T hardThreat;
  boolean_T midPredThreat;
  boolean_T predSafe;
  boolean_T reactionGrowing;
  boolean_T sub5ReactionThreat;
  boolean_T tmp_1;
  boolean_T tmp_2;
  boolean_T touchGrowing;

  /* '<S6>:1:908' */
  eventFault = false;

  /* '<S6>:1:910' */
  absAngle = fabs(angle);

  /* '<S6>:1:911' */
  absRate = fabs(rate);

  /* '<S6>:1:913' */
  /* '<S6>:1:920' */
  hardThreat = (absAngle >= limitDeg - 0.05);

  /* '<S6>:1:927' */
  reactionPred = rate * sub5ReactionS + angle;

  /* '<S6>:1:930' */
  if (reactionPred < -180.0) {
    /* '<S6>:1:2144' */
    /* '<S6>:1:2146' */
    reactionPred = -180.0;
  } else if (reactionPred > 180.0) {
    /* '<S6>:1:2148' */
    /* '<S6>:1:2150' */
    reactionPred = 180.0;
  } else {
    /* '<S6>:1:2154' */
  }

  /* '<S6>:1:937' */
  speedAfterDelay = fabs(reactionPred);
  reactionGrowing = (speedAfterDelay >= absAngle + sub5GrowthMarginDeg);
  if ((absAngle < noFire) && (absRate >= sub5MinOutwardDps) && reactionGrowing &&
      (speedAfterDelay >= limitDeg)) {
    /* '<S6>:1:943' */
    /* '<S6>:1:944' */
    /* '<S6>:1:945' */
    /* '<S6>:1:946' */
    sub5ReactionThreat = true;
  } else {
    sub5ReactionThreat = false;
  }

  if (predValid && (fabs(pred) >= absAngle + predHyst)) {
    /* '<S6>:1:953' */
    /* '<S6>:1:954' */
    touchGrowing = true;
  } else {
    touchGrowing = false;
  }

  if ((absAngle < noFire) && predValid && (absRate >= sub5MinOutwardDps) &&
      touchGrowing && (fabs(pred) >= limitDeg)) {
    /* '<S6>:1:958' */
    /* '<S6>:1:959' */
    /* '<S6>:1:960' */
    /* '<S6>:1:962' */
  } else {
    touchGrowing = false;
  }

  touchGrowing = (sub5ReactionThreat || touchGrowing);
  if ((absAngle >= noFire) && (absAngle < limitDeg - 0.05) && (absRate >=
       sub5MinOutwardDps) && reactionGrowing && (speedAfterDelay >= limitDeg)) {
    /* '<S6>:1:980' */
    /* '<S6>:1:981' */
    /* '<S6>:1:982' */
    /* '<S6>:1:983' */
    /* '<S6>:1:984' */
  } else {
    reactionGrowing = false;
  }

  if (predValid && (absAngle >= noFire) && (absAngle < limitDeg - 0.05) && (fabs
       (pred) >= limitDeg)) {
    /* '<S6>:1:991' */
    /* '<S6>:1:992' */
    /* '<S6>:1:993' */
    /* '<S6>:1:994' */
    midPredThreat = true;
  } else {
    midPredThreat = false;
  }

  midPredThreat = (touchGrowing || reactionGrowing || midPredThreat);
  tmp_1 = !hardThreat;
  if (tmp_1 && (!midPredThreat) && ((!predValid) || (fabs(pred) <= limitDeg -
        predHyst))) {
    /* '<S6>:1:1008' */
    /* '<S6>:1:1009' */
    /* '<S6>:1:1010' */
    /* '<S6>:1:1011' */
    predSafe = true;
  } else {
    predSafe = false;
  }

  /* '<S6>:1:1018' */
  speedAfterDelay = alpha * brakeDelayS + absRate;

  /* '<S6>:1:1022' */
  /* '<S6>:1:1027' */
  tmp_0 = 2.0 * alpha;
  A->stopping = rate * rate / tmp_0;

  /* '<S6>:1:1030' */
  A->braking_distance = ((0.5 * alpha * brakeDelayS * brakeDelayS + absRate *
    brakeDelayS) + speedAfterDelay * speedAfterDelay / tmp_0) + brakeMargin;

  /* '<S6>:1:1036' */
  A->target = 0.0;

  /* '<S6>:1:1037' */
  A->switch_error = 0.0;
  if (!A->event_active) {
    /* '<S6>:1:1043' */
    if (midPredThreat) {
      if (A->pred_count < 255) {
        /* '<S6>:1:1047' */
        /* '<S6>:1:1049' */
        tmp = A->pred_count + 1U;
        if (A->pred_count + 1U > 255U) {
          tmp = 255U;
        }

        A->pred_count = (uint8_T)tmp;
      }
    } else {
      /* '<S6>:1:1056' */
      A->pred_count = 0U;
    }
  }

  /* '<S6>:1:1062' */
  tmp_2 = !touchGrowing;
  if ((absAngle < noFire) && tmp_2) {
    /* '<S6>:1:1075' */
    /* '<S6>:1:1076' */
    /* '<S6>:1:1078' */
    Ucus_Bilgisayari_finishEvent(A, true, minOffN);
  } else {
    if (A->phase == 4) {
      /* '<S6>:1:1092' */
      if ((hardThreat || midPredThreat) && (A->event_end_age >= minOffN)) {
        /* '<S6>:1:1101' */
        /* '<S6>:1:1102' */
        /* '<S6>:1:1104' */
        A->event_armed = true;

        /* '<S6>:1:1106' */
        A->phase = 0U;

        /* '<S6>:1:1107' */
        A->mode = 0U;
      } else if (predSafe) {
        if (A->safe_age >= rearmN) {
          /* '<S6>:1:1112' */
          /* '<S6>:1:1114' */
          A->event_armed = true;

          /* '<S6>:1:1116' */
          A->phase = 0U;

          /* '<S6>:1:1117' */
          A->mode = 0U;
        }
      } else {
        /* '<S6>:1:1123' */
        A->safe_age = 0U;
      }

      if (hardThreat && (A->event_end_age >= hardRearmN)) {
        /* '<S6>:1:1128' */
        /* '<S6>:1:1129' */
        /* '<S6>:1:1131' */
        A->event_armed = true;

        /* '<S6>:1:1133' */
        A->phase = 0U;

        /* '<S6>:1:1134' */
        A->mode = 0U;
      }
    }

    guard1 = false;
    if (!A->event_active) {
      /* '<S6>:1:1144' */
      if (!A->event_armed) {
        /* '<S6>:1:1146' */
        /* '<S6>:1:1148' */
        A->mode = 4U;

        /* '<S6>:1:1149' */
        A->desired = 0;
      } else if (tmp_1 && (A->pred_count < predConfirmN)) {
        /* '<S6>:1:1155' */
        /* '<S6>:1:1156' */
        /* '<S6>:1:1158' */
        A->mode = 0U;

        /* '<S6>:1:1159' */
        A->desired = 0;
      } else {
        guard2 = false;
        if (hardThreat) {
          /* '<S6>:1:1167' */
          /* '<S6>:1:1169' */
          if (angle > 0.0) {
            /* '<S6>:1:2124' */
            /* '<S6>:1:2126' */
            threatSign = 1;
          } else if (angle < 0.0) {
            /* '<S6>:1:2128' */
            /* '<S6>:1:2130' */
            threatSign = -1;
          } else {
            /* '<S6>:1:2134' */
            threatSign = 0;
            guard2 = true;
          }
        } else if (sub5ReactionThreat || reactionGrowing) {
          /* '<S6>:1:1177' */
          if (reactionPred > 0.0) {
            /* '<S6>:1:2124' */
            /* '<S6>:1:2126' */
            threatSign = 1;
          } else if (reactionPred < 0.0) {
            /* '<S6>:1:2128' */
            /* '<S6>:1:2130' */
            threatSign = -1;
          } else {
            /* '<S6>:1:2134' */
            threatSign = 0;
            guard2 = true;
          }

          /* '<S6>:1:1182' */
        } else if (pred > 0.0) {
          /* '<S6>:1:2124' */
          /* '<S6>:1:2126' */
          threatSign = 1;
        } else if (pred < 0.0) {
          /* '<S6>:1:2128' */
          /* '<S6>:1:2130' */
          threatSign = -1;
        } else {
          /* '<S6>:1:2134' */
          threatSign = 0;
          guard2 = true;
        }

        if (guard2) {
          /* '<S6>:1:1187' */
          /* '<S6>:1:1189' */
          if (rate > 0.0) {
            /* '<S6>:1:2124' */
            /* '<S6>:1:2126' */
            threatSign = 1;
          } else if (rate < 0.0) {
            /* '<S6>:1:2128' */
            /* '<S6>:1:2130' */
            threatSign = -1;
          } else {
            /* '<S6>:1:2134' */
          }
        }

        /* '<S6>:1:1194' */
        Ucus_Bilgisayari_startEvent(A, threatSign, hardThreat, maxEvents,
          &sub5ReactionThreat, &eventFault);

        /* '<S6>:1:1194' */
        if ((!sub5ReactionThreat) || eventFault) {
          /* '<S6>:1:1201' */
          /* '<S6>:1:1194' */
          /* '<S6>:1:1203' */
          A->desired = 0;
        } else {
          guard1 = true;
        }
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (hardThreat) {
        /* '<S6>:1:1215' */
        /* '<S6>:1:1217' */
        A->hard_latched = true;
      }

      if (A->phase == 1) {
        /* '<S6>:1:1225' */
        /* '<S6>:1:1227' */
        if (A->hazard_sign > 0) {
          /* '<S6>:1:2084' */
          /* '<S6>:1:2086' */
          threatSign = -1;
        } else if (A->hazard_sign < 0) {
          /* '<S6>:1:2088' */
          /* '<S6>:1:2090' */
          threatSign = 1;
        } else {
          /* '<S6>:1:2094' */
          threatSign = 0;
        }

        if ((absAngle < noFire) && touchGrowing) {
          /* '<S6>:1:1245' */
          /* '<S6>:1:1248' */
          A->target = 0.0;

          /* '<S6>:1:1249' */
          A->switch_error = 0.0;

          /* '<S6>:1:1251' */
          A->mode = 1U;

          /* '<S6>:1:1252' */
          A->desired = threatSign;
        } else {
          /* '<S6>:1:1262' */
          /* '<S6>:1:1265' */
          absAngle = (real_T)A->hazard_sign * angle - safeTarget;

          /* '<S6>:1:1268' */
          reactionPred = -(real_T)A->hazard_sign * rate;

          /* '<S6>:1:1271' */
          A->target = (real_T)A->hazard_sign * safeTarget;

          /* '<S6>:1:1275' */
          A->switch_error = absAngle - A->braking_distance;
          if (A->hard_latched && A->brake_used && tmp_1) {
            /* '<S6>:1:1287' */
            /* '<S6>:1:1288' */
            /* '<S6>:1:1289' */
            /* '<S6>:1:1291' */
            Ucus_Bilgisayari_finishEvent(A, false, minOffN);
          } else if ((!A->hard_latched) && predValid && (A->event_age >=
                      minCorrN) && (fabs(pred) <= limitDeg - predHyst)) {
            /* '<S6>:1:1303' */
            /* '<S6>:1:1304' */
            /* '<S6>:1:1305' */
            /* '<S6>:1:1306' */
            /* '<S6>:1:1309' */
            Ucus_Bilgisayari_finishEvent(A, false, minOffN);
          } else if ((!A->hard_latched) && tmp_1 && (absRate <= brakeDone) && ((
                       !predValid) || (fabs(pred) <= limitDeg - predHyst))) {
            /* '<S6>:1:1321' */
            /* '<S6>:1:1322' */
            /* '<S6>:1:1323' */
            /* '<S6>:1:1324' */
            /* '<S6>:1:1325' */
            /* '<S6>:1:1328' */
            Ucus_Bilgisayari_finishEvent(A, false, minOffN);
          } else if (reactionPred <= 0.0) {
            /* '<S6>:1:1340' */
            /* '<S6>:1:1342' */
            A->mode = 1U;

            /* '<S6>:1:1343' */
            A->desired = threatSign;
          } else if ((!A->hard_latched) && tmp_1 && (reactionPred <= brakeDone) &&
                     (absAngle <= brakeMargin)) {
            /* '<S6>:1:1351' */
            /* '<S6>:1:1352' */
            /* '<S6>:1:1353' */
            /* '<S6>:1:1354' */
            /* '<S6>:1:1356' */
            Ucus_Bilgisayari_finishEvent(A, false, minOffN);
          } else if (absAngle <= A->braking_distance) {
            /* '<S6>:1:1370' */
            if ((!A->brake_used) && (reactionPred > brakeDone)) {
              /* '<S6>:1:1373' */
              /* '<S6>:1:1374' */
              /* '<S6>:1:1376' */
              A->phase = 2U;

              /* '<S6>:1:1378' */
              A->brake_used = true;

              /* '<S6>:1:1379' */
              A->brake_age = 0U;

              /* '<S6>:1:1381' */
              A->mode = 2U;

              /* '<S6>:1:1383' */
              if (A->hazard_sign > 0) {
                /* '<S6>:1:2104' */
                /* '<S6>:1:2106' */
                A->desired = 1;
              } else if (A->hazard_sign < 0) {
                /* '<S6>:1:2108' */
                /* '<S6>:1:2110' */
                A->desired = -1;
              } else {
                /* '<S6>:1:2114' */
                A->desired = 0;
              }
            } else if (A->hard_latched) {
              /* '<S6>:1:1391' */
              /* '<S6>:1:1393' */
              A->mode = 1U;

              /* '<S6>:1:1394' */
              A->desired = threatSign;
            } else {
              /* '<S6>:1:1400' */
              Ucus_Bilgisayari_finishEvent(A, false, minOffN);
            }
          } else {
            /* '<S6>:1:1414' */
            A->mode = 1U;

            /* '<S6>:1:1415' */
            A->desired = threatSign;
          }
        }
      } else if (A->phase == 2) {
        /* '<S6>:1:1425' */
        /* '<S6>:1:1427' */
        /* '<S6>:1:1430' */
        A->mode = 2U;
        if ((-(real_T)A->hazard_sign * rate <= brakeDone) || (A->brake_age >=
             maxBrakeN)) {
          /* '<S6>:1:1433' */
          /* '<S6>:1:1434' */
          if (A->hard_latched) {
            /* '<S6>:1:1437' */
            if (hardThreat) {
              /* '<S6>:1:1443' */
              /* '<S6>:1:1445' */
              A->phase = 1U;

              /* '<S6>:1:1446' */
              A->mode = 1U;

              /* '<S6>:1:1448' */
              A->brake_age = 0U;

              /* '<S6>:1:1450' */
              if (A->hazard_sign > 0) {
                /* '<S6>:1:2084' */
                /* '<S6>:1:2086' */
                A->desired = -1;
              } else if (A->hazard_sign < 0) {
                /* '<S6>:1:2088' */
                /* '<S6>:1:2090' */
                A->desired = 1;
              } else {
                /* '<S6>:1:2094' */
                A->desired = 0;
              }
            } else {
              /* '<S6>:1:1461' */
              Ucus_Bilgisayari_finishEvent(A, false, minOffN);
            }
          } else if (hardThreat) {
            /* '<S6>:1:1474' */
            /* '<S6>:1:1478' */
            A->hard_latched = true;

            /* '<S6>:1:1480' */
            A->phase = 1U;

            /* '<S6>:1:1481' */
            A->mode = 1U;

            /* '<S6>:1:1483' */
            A->brake_age = 0U;

            /* '<S6>:1:1485' */
            if (A->hazard_sign > 0) {
              /* '<S6>:1:2084' */
              /* '<S6>:1:2086' */
              A->desired = -1;
            } else if (A->hazard_sign < 0) {
              /* '<S6>:1:2088' */
              /* '<S6>:1:2090' */
              A->desired = 1;
            } else {
              /* '<S6>:1:2094' */
              A->desired = 0;
            }
          } else {
            /* '<S6>:1:1495' */
            Ucus_Bilgisayari_finishEvent(A, false, minOffN);
          }

          /* '<S6>:1:1508' */
        } else if (A->hazard_sign > 0) {
          /* '<S6>:1:2104' */
          /* '<S6>:1:2106' */
          A->desired = 1;
        } else if (A->hazard_sign < 0) {
          /* '<S6>:1:2108' */
          /* '<S6>:1:2110' */
          A->desired = -1;
        } else {
          /* '<S6>:1:2114' */
          A->desired = 0;
        }
      } else if (A->phase == 3) {
        /* '<S6>:1:1520' */
        /* '<S6>:1:1522' */
        /* '<S6>:1:1525' */
        A->mode = 5U;

        /* '<S6>:1:1527' */
        A->desired = 0;
        if (tmp_1 && tmp_2) {
          /* '<S6>:1:1529' */
          /* '<S6>:1:1530' */
          /* '<S6>:1:1532' */
          Ucus_Bilgisayari_finishEvent(A, false, minOffN);
        } else if ((real_T)A->hazard_sign * rate >= 0.0) {
          /* '<S6>:1:1542' */
          /* '<S6>:1:1544' */
          Ucus_Bilgisayari_finishEvent(A, true, minOffN);
        }
      } else {
        /* '<S6>:1:1558' */
        A->mode = 4U;

        /* '<S6>:1:1559' */
        A->desired = 0;
      }
    }
  }

  return eventFault;
}

/* Function for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
static void Ucus_Bil_applyReferenceTracking(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T *A,
  real_T angle, real_T rate, real_T refAngle, real_T noFireDeg, real_T
  maxAbsAngle, real_T startDb, real_T stopDb, real_T lookahead, real_T rateFar,
  real_T rateMid, real_T rateNear, uint16_T cooldownFarN, uint16_T cooldownMidN,
  uint16_T cooldownNearN, uint16_T pulseSmallN, uint16_T pulseMedN, uint16_T
  pulseLargeN)
{
  real_T errorMag;
  real_T errorNow;
  real_T errorPred;
  real_T rateHold;
  uint32_T qY;
  uint16_T nextCooldownN;
  uint16_T x;
  int8_T controlSign;
  int8_T errorSign;
  boolean_T crossingTarget;

  /* '<S6>:1:1685' */
  errorPred = fabs(angle);
  if ((errorPred < noFireDeg) && (fabs(refAngle) <= 0.5)) {
    /* '<S6>:1:1687' */
    /* '<S6>:1:1688' */
    /* '<S6>:1:1690' */
    A->track_active = false;

    /* '<S6>:1:1691' */
    A->track_sign = 0;

    /* '<S6>:1:1693' */
    A->track_age = 0U;

    /* '<S6>:1:1694' */
    A->track_pulse_n = 0U;

    /* '<S6>:1:1695' */
    A->track_cooldown = 0U;
    if (!A->event_active) {
      /* '<S6>:1:1697' */
      /* '<S6>:1:1699' */
      A->desired = 0;

      /* '<S6>:1:1700' */
      A->mode = 0U;
    }
  } else if (fabs(refAngle) <= 0.5) {
    /* '<S6>:1:1722' */
    /* '<S6>:1:1724' */
    A->track_active = false;

    /* '<S6>:1:1725' */
    A->track_sign = 0;

    /* '<S6>:1:1727' */
    A->track_age = 0U;

    /* '<S6>:1:1728' */
    A->track_pulse_n = 0U;

    /* '<S6>:1:1729' */
    A->track_cooldown = 0U;
    if (!A->event_active) {
      /* '<S6>:1:1731' */
      /* '<S6>:1:1733' */
      A->desired = 0;

      /* '<S6>:1:1734' */
      A->mode = 0U;
    }
  } else if (A->event_active || (A->phase != 0) || (errorPred >= maxAbsAngle)) {
    /* '<S6>:1:1746' */
    /* '<S6>:1:1747' */
    /* '<S6>:1:1748' */
    /* '<S6>:1:1750' */
    A->track_active = false;

    /* '<S6>:1:1751' */
    A->track_sign = 0;

    /* '<S6>:1:1753' */
    A->track_age = 0U;

    /* '<S6>:1:1754' */
    A->track_pulse_n = 0U;

    /* '<S6>:1:1755' */
    A->track_cooldown = 0U;
  } else {
    /* '<S6>:1:1765' */
    errorNow = angle - refAngle;

    /* '<S6>:1:1768' */
    errorPred = rate * lookahead + errorNow;

    /* '<S6>:1:1772' */
    errorMag = fabs(errorPred);
    if (errorMag >= 4.0) {
      /* '<S6>:1:1779' */
      /* '<S6>:1:1781' */
      rateHold = rateFar;

      /* '<S6>:1:1782' */
      nextCooldownN = cooldownFarN;
    } else if (errorMag >= 2.0) {
      /* '<S6>:1:1784' */
      /* '<S6>:1:1786' */
      rateHold = rateMid;

      /* '<S6>:1:1787' */
      nextCooldownN = cooldownMidN;
    } else {
      /* '<S6>:1:1791' */
      rateHold = rateNear;

      /* '<S6>:1:1792' */
      nextCooldownN = cooldownNearN;
    }

    if (A->track_active) {
      /* '<S6>:1:1800' */
      /* '<S6>:1:1802' */
      A->mode = 6U;

      /* '<S6>:1:1804' */
      if (A->track_sign > 0) {
        /* '<S6>:1:2084' */
        /* '<S6>:1:2086' */
        A->desired = -1;
      } else if (A->track_sign < 0) {
        /* '<S6>:1:2088' */
        /* '<S6>:1:2090' */
        A->desired = 1;
      } else {
        /* '<S6>:1:2094' */
        A->desired = 0;
      }

      /* '<S6>:1:1808' */
      x = A->track_age;
      if (A->track_age < 65535) {
        /* '<S6>:1:2164' */
        /* '<S6>:1:2166' */
        qY = A->track_age + 1U;
        if (A->track_age + 1U > 65535U) {
          qY = 65535U;
        }

        x = (uint16_T)qY;
      }

      A->track_age = x;
      if (x >= A->track_pulse_n) {
        /* '<S6>:1:1811' */
        /* '<S6>:1:1814' */
        A->track_active = false;

        /* '<S6>:1:1815' */
        A->track_sign = 0;

        /* '<S6>:1:1817' */
        A->track_age = 0U;

        /* '<S6>:1:1818' */
        A->track_pulse_n = 0U;

        /* '<S6>:1:1820' */
        A->track_cooldown = nextCooldownN;

        /* '<S6>:1:1823' */
        A->desired = 0;
      }
    } else if (A->track_cooldown > 0) {
      /* '<S6>:1:1835' */
      /* '<S6>:1:1837' */
      qY = A->track_cooldown - 1U;
      if (A->track_cooldown - 1U > A->track_cooldown) {
        qY = 0U;
      }

      A->track_cooldown = (uint16_T)qY;

      /* '<S6>:1:1840' */
      A->desired = 0;

      /* '<S6>:1:1841' */
      A->mode = 6U;
    } else if ((fabs(errorNow) <= stopDb) && (errorMag <= startDb)) {
      /* '<S6>:1:1851' */
      /* '<S6>:1:1852' */
      /* '<S6>:1:1854' */
      A->desired = 0;

      /* '<S6>:1:1855' */
      A->mode = 0U;
    } else {
      /* '<S6>:1:1861' */
      if (errorNow > 0.0) {
        /* '<S6>:1:2124' */
        /* '<S6>:1:2126' */
        errorSign = 1;

        /* '<S6>:1:1864' */
        /* '<S6>:1:1866' */
        errorNow = -(real_T)errorSign * rate;
      } else if (errorNow < 0.0) {
        /* '<S6>:1:2128' */
        /* '<S6>:1:2130' */
        errorSign = -1;

        /* '<S6>:1:1864' */
        /* '<S6>:1:1866' */
        errorNow = -(real_T)errorSign * rate;
      } else {
        /* '<S6>:1:2134' */
        errorSign = 0;

        /* '<S6>:1:1871' */
        errorNow = 0.0;
      }

      if ((errorSign != 0) && ((errorPred > 0.0) || (errorPred < 0.0))) {
        /* '<S6>:1:1876' */
        /* '<S6>:1:2124' */
        /* '<S6>:1:2126' */
        /* '<S6>:1:2128' */
        /* '<S6>:1:2130' */
        /* '<S6>:1:1877' */
        if (errorPred > 0.0) {
          /* '<S6>:1:2124' */
          /* '<S6>:1:2126' */
          controlSign = 1;
        } else if (errorPred < 0.0) {
          /* '<S6>:1:2128' */
          /* '<S6>:1:2130' */
          controlSign = -1;
        } else {
          /* '<S6>:1:2134' */
          controlSign = 0;
        }

        if (controlSign != errorSign) {
          /* '<S6>:1:1878' */
          crossingTarget = true;
        } else {
          crossingTarget = false;
        }
      } else {
        /* '<S6>:1:2134' */
        crossingTarget = false;
      }

      if (crossingTarget || (errorNow >= rateHold)) {
        /* '<S6>:1:1885' */
        /* '<S6>:1:1887' */
        A->desired = 0;

        /* '<S6>:1:1888' */
        A->mode = 6U;
      } else if (errorMag < startDb) {
        /* '<S6>:1:1898' */
        /* '<S6>:1:1900' */
        A->desired = 0;

        /* '<S6>:1:1901' */
        A->mode = 0U;
      } else {
        /* '<S6>:1:1911' */
        if (errorPred > 0.0) {
          /* '<S6>:1:2124' */
          /* '<S6>:1:2126' */
          controlSign = 1;
        } else if (errorPred < 0.0) {
          /* '<S6>:1:2128' */
          /* '<S6>:1:2130' */
          controlSign = -1;
        } else {
          /* '<S6>:1:2134' */
          /* '<S6>:1:1914' */
          /* '<S6>:1:1916' */
          controlSign = errorSign;
        }

        if (controlSign == 0) {
          /* '<S6>:1:1920' */
          /* '<S6>:1:1922' */
          A->desired = 0;

          /* '<S6>:1:1923' */
          A->mode = 0U;
        } else {
          if (errorMag >= 4.0) {
            /* '<S6>:1:1929' */
            /* '<S6>:1:1931' */
            A->track_pulse_n = pulseLargeN;
          } else if (errorMag >= 2.0) {
            /* '<S6>:1:1933' */
            /* '<S6>:1:1935' */
            A->track_pulse_n = pulseMedN;
          } else {
            /* '<S6>:1:1939' */
            A->track_pulse_n = pulseSmallN;
          }

          /* '<S6>:1:1943' */
          A->track_active = true;

          /* '<S6>:1:1945' */
          A->track_sign = controlSign;

          /* '<S6>:1:1947' */
          A->track_age = 0U;

          /* '<S6>:1:1949' */
          /* '<S6>:1:1951' */
          if (controlSign > 0) {
            /* '<S6>:1:2084' */
            /* '<S6>:1:2086' */
            A->desired = -1;
          } else if (controlSign < 0) {
            /* '<S6>:1:2088' */
            /* '<S6>:1:2090' */
            A->desired = 1;
          } else {
            /* '<S6>:1:2094' */
            A->desired = 0;
          }

          /* '<S6>:1:1955' */
          A->mode = 6U;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
static boolean_T Ucus_Bilgisay_processAxisOutput(sXzzr8I9NVxmx2gyMIcZCVE_Ucus__T
  *A, uint16_T deadN, uint16_T minOffN, uint8_T maxReversals)
{
  int32_T tmp_0;
  uint32_T tmp;
  uint16_T x;
  boolean_T chatterFault;

  /* '<S6>:1:1967' */
  chatterFault = false;
  if (A->dead_active) {
    /* '<S6>:1:1971' */
    /* '<S6>:1:1973' */
    A->applied = 0;

    /* '<S6>:1:1975' */
    A->mode = 3U;

    /* '<S6>:1:1977' */
    x = A->dead_age;
    if (A->dead_age < 65535) {
      /* '<S6>:1:2164' */
      /* '<S6>:1:2166' */
      tmp = A->dead_age + 1U;
      if (A->dead_age + 1U > 65535U) {
        tmp = 65535U;
      }

      x = (uint16_T)tmp;
    }

    A->dead_age = x;
    if (A->desired == 0) {
      /* '<S6>:1:1980' */
      /* '<S6>:1:1982' */
      A->pending = 0;

      /* '<S6>:1:1984' */
      A->dead_active = false;

      /* '<S6>:1:1986' */
      A->dead_age = 0U;

      /* '<S6>:1:1988' */
      A->off_age = 0U;
    } else {
      if (A->desired != A->pending) {
        /* '<S6>:1:1994' */
        /* '<S6>:1:1996' */
        A->pending = A->desired;

        /* '<S6>:1:1998' */
        A->dead_age = 0U;
      }

      if (A->dead_age >= deadN) {
        /* '<S6>:1:2002' */
        /* '<S6>:1:2004' */
        A->applied = A->pending;

        /* '<S6>:1:2006' */
        A->pending = 0;

        /* '<S6>:1:2008' */
        A->dead_active = false;

        /* '<S6>:1:2010' */
        A->dead_age = 0U;

        /* '<S6>:1:2012' */
        A->off_age = 0U;
      }
    }
  } else if (A->desired != A->applied) {
    tmp_0 = -A->applied;
    if (-A->applied > 127) {
      tmp_0 = 127;
    }

    if ((A->applied != 0) && (A->desired == tmp_0)) {
      /* '<S6>:1:2030' */
      /* '<S6>:1:2031' */
      /* '<S6>:1:2033' */
      A->applied = 0;

      /* '<S6>:1:2035' */
      A->pending = A->desired;

      /* '<S6>:1:2037' */
      A->dead_active = true;

      /* '<S6>:1:2039' */
      A->dead_age = 0U;

      /* '<S6>:1:2041' */
      A->off_age = 0U;

      /* '<S6>:1:2043' */
      tmp = A->reversal_count + 1U;
      if (A->reversal_count + 1U > 255U) {
        tmp = 255U;
      }

      A->reversal_count = (uint8_T)tmp;
      if (A->reversal_count > maxReversals) {
        /* '<S6>:1:2046' */
        /* '<S6>:1:2048' */
        chatterFault = true;
      }
    } else if (A->desired == 0) {
      /* '<S6>:1:2058' */
      /* '<S6>:1:2060' */
      A->applied = 0;

      /* '<S6>:1:2062' */
      A->off_age = 0U;
    } else if (A->off_age >= minOffN) {
      /* '<S6>:1:2070' */
      /* '<S6>:1:2072' */
      A->applied = A->desired;

      /* '<S6>:1:2074' */
      A->off_age = 0U;
    }
  } else {
    /* '<S6>:1:2022' */
  }

  return chatterFault;
}

/* Model output function */
void Ucus_Bilgisayari_output(void)
{
  real_T F_available;
  real_T F_weight;
  real_T a_edge_max;
  real_T ay_cmd;
  real_T edge_x;
  real_T edge_y;
  real_T rtb_F_Hedef_N;
  real_T rtb_Hover_Durum;
  real_T rtb_Hover_Suresi;
  real_T rtb_Saturation;
  real_T rtb_UnitDelay;
  real_T rtb_UnitDelay2;
  real_T rtb_UnitDelay4;
  real_T rtb_UnitDelay6;
  real_T tmp;
  real_T valve_ff;
  real_T vx_use;
  real_T vy_use;
  uint32_T qY;
  boolean_T guard1;
  boolean_T physical_shortage;
  boolean_T sensor_valid;

  /* UnitDelay: '<Root>/Unit Delay' */
  rtb_UnitDelay = Ucus_Bilgisayari_DW.UnitDelay_DSTATE;

  /* Outport: '<Root>/V1' */
  Ucus_Bilgisayari_Y.V1 = rtb_UnitDelay;

  /* UnitDelay: '<Root>/Unit Delay2' */
  rtb_UnitDelay2 = Ucus_Bilgisayari_DW.UnitDelay2_DSTATE;

  /* Outport: '<Root>/V3' */
  Ucus_Bilgisayari_Y.V3 = rtb_UnitDelay2;

  /* UnitDelay: '<Root>/Unit Delay4' */
  rtb_UnitDelay4 = Ucus_Bilgisayari_DW.UnitDelay4_DSTATE;

  /* Outport: '<Root>/V5' */
  Ucus_Bilgisayari_Y.V5 = rtb_UnitDelay4;

  /* UnitDelay: '<Root>/Unit Delay6' */
  rtb_UnitDelay6 = Ucus_Bilgisayari_DW.UnitDelay6_DSTATE;

  /* Outport: '<Root>/V7' */
  Ucus_Bilgisayari_Y.V7 = rtb_UnitDelay6;

  /* UnitDelay: '<Root>/Unit Delay8' */
  rtb_Saturation = Ucus_Bilgisayari_DW.UnitDelay8_DSTATE;

  /* RateLimiter: '<Root>/Rate Limiter2' */
  vx_use = rtb_Saturation - Ucus_Bilgisayari_DW.PrevY;
  tmp = Ucus_Bilgisayari_P.RateLimiter2_RisingLim * Ucus_Bilgisayari_period;
  if (vx_use > tmp) {
    rtb_Saturation = tmp + Ucus_Bilgisayari_DW.PrevY;
  } else if (vx_use < Ucus_Bilgisayari_P.RateLimiter2_FallingLim *
             Ucus_Bilgisayari_period) {
    rtb_Saturation = Ucus_Bilgisayari_P.RateLimiter2_FallingLim *
      Ucus_Bilgisayari_period + Ucus_Bilgisayari_DW.PrevY;
  }

  Ucus_Bilgisayari_DW.PrevY = rtb_Saturation;

  /* End of RateLimiter: '<Root>/Rate Limiter2' */

  /* Saturate: '<Root>/Saturation' */
  if (rtb_Saturation > Ucus_Bilgisayari_P.Saturation_UpperSat) {
    rtb_Saturation = Ucus_Bilgisayari_P.Saturation_UpperSat;
  } else if (rtb_Saturation < Ucus_Bilgisayari_P.Saturation_LowerSat) {
    rtb_Saturation = Ucus_Bilgisayari_P.Saturation_LowerSat;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Outport: '<Root>/Ana itki' incorporates:
   *  Gain: '<Root>/Gain1'
   */
  Ucus_Bilgisayari_Y.Anaitki = Ucus_Bilgisayari_P.Gain1_Gain * rtb_Saturation;

  /* MATLAB Function: '<Root>/Hover_Kontrol' incorporates:
   *  Inport: '<Root>/m_guncel'
   *  Inport: '<Root>/zpos'
   *  Inport: '<Root>/zvel'
   *  UnitDelay: '<Root>/Unit Delay1'
   *  UnitDelay: '<Root>/Unit Delay3'
   */
  /* MATLAB Function 'Hover_Kontrol': '<S2>:1' */
  /* '<S2>:1:27' */
  /* '<S2>:1:28' */
  /* '<S2>:1:30' */
  /* '<S2>:1:31' */
  /* '<S2>:1:33' */
  /* '<S2>:1:35' */
  /* '<S2>:1:40' */
  /* '<S2>:1:43' */
  /* '<S2>:1:44' */
  /* '<S2>:1:54' */
  /* '<S2>:1:56' */
  /* '<S2>:1:57' */
  /* '<S2>:1:64' */
  /* '<S2>:1:68' */
  /* '<S2>:1:69' */
  /* '<S2>:1:80' */
  /* '<S2>:1:83' */
  /* '<S2>:1:86' */
  /* '<S2>:1:94' */
  /* '<S2>:1:95' */
  /* '<S2>:1:99' */
  /* '<S2>:1:101' */
  /* '<S2>:1:103' */
  /* '<S2>:1:104' */
  /* '<S2>:1:106' */
  /* '<S2>:1:110' */
  /* '<S2>:1:111' */
  /* '<S2>:1:113' */
  /* '<S2>:1:165' */
  valve_ff = 0.0;

  /* '<S2>:1:166' */
  /* '<S2>:1:167' */
  rtb_Hover_Durum = Ucus_Bilgisayari_DW.state_p;

  /* '<S2>:1:169' */
  rtb_Hover_Suresi = (real_T)Ucus_Bilgisayari_DW.hover_best_count * 0.01;

  /* '<S2>:1:172' */
  rtb_F_Hedef_N = 0.0;

  /* '<S2>:1:173' */
  if (Ucus_Bilgisayari_DW.touchdown_latch) {
    /* '<S2>:1:177' */
    /* '<S2>:1:179' */
    Ucus_Bilgisayari_DW.state_p = 4U;

    /* '<S2>:1:181' */
    /* '<S2>:1:182' */
    /* '<S2>:1:183' */
    /* '<S2>:1:185' */
    Ucus_Bilgisayari_DW.valve_previous = 0.0;

    /* '<S2>:1:187' */
    Ucus_Bilgisayari_DW.capture_brake_active = false;

    /* '<S2>:1:188' */
    Ucus_Bilgisayari_DW.capture_brake_done = true;

    /* '<S2>:1:190' */
    rtb_Hover_Durum = 4.0;

    /* '<S2>:1:192' */
    /* '<S2>:1:195' */
  } else if ((!rtIsInf(Ucus_Bilgisayari_U.zpos)) && (!rtIsNaN
              (Ucus_Bilgisayari_U.zpos)) && ((!rtIsInf(Ucus_Bilgisayari_U.zvel))
              && (!rtIsNaN(Ucus_Bilgisayari_U.zvel))) && ((!rtIsInf
               (Ucus_Bilgisayari_U.m_guncel)) && (!rtIsNaN
               (Ucus_Bilgisayari_U.m_guncel))) && ((!rtIsInf
               (Ucus_Bilgisayari_DW.UnitDelay1_DSTATE)) && (!rtIsNaN
               (Ucus_Bilgisayari_DW.UnitDelay1_DSTATE))) && ((!rtIsInf
               (Ucus_Bilgisayari_DW.UnitDelay3_DSTATE)) && (!rtIsNaN
               (Ucus_Bilgisayari_DW.UnitDelay3_DSTATE)))) {
    /* '<S2>:1:205' */
    /* '<S2>:1:206' */
    /* '<S2>:1:207' */
    /* '<S2>:1:208' */
    /* '<S2>:1:209' */
    /* '<S2>:1:224' */
    if (Ucus_Bilgisayari_U.zpos <= 0.4313) {
      /* '<S2>:1:227' */
      /* '<S2>:1:229' */
      Ucus_Bilgisayari_DW.touchdown_latch = true;

      /* '<S2>:1:230' */
      Ucus_Bilgisayari_DW.state_p = 4U;

      /* '<S2>:1:232' */
      /* '<S2>:1:233' */
      /* '<S2>:1:234' */
      /* '<S2>:1:236' */
      Ucus_Bilgisayari_DW.valve_previous = 0.0;

      /* '<S2>:1:238' */
      Ucus_Bilgisayari_DW.capture_brake_active = false;

      /* '<S2>:1:239' */
      Ucus_Bilgisayari_DW.capture_brake_done = true;

      /* '<S2>:1:241' */
      rtb_Hover_Durum = 4.0;

      /* '<S2>:1:243' */
      /* '<S2>:1:246' */
    } else if ((Ucus_Bilgisayari_DW.UnitDelay1_DSTATE < 5.0) ||
               (Ucus_Bilgisayari_DW.UnitDelay1_DSTATE > 350.0)) {
      /* '<S2>:1:255' */
      /* '<S2>:1:256' */
      /* '<S2>:1:258' */
      /* '<S2>:1:259' */
      Ucus_Bilgisayari_DW.valve_previous = 0.0;

      /* '<S2>:1:261' */
      Ucus_Bilgisayari_DW.capture_brake_active = false;
    } else {
      /* '<S2>:1:269' */
      if (Ucus_Bilgisayari_U.m_guncel >= 27.5) {
        rtb_Hover_Durum = Ucus_Bilgisayari_U.m_guncel;
      } else {
        rtb_Hover_Durum = 27.5;
      }

      if (!(rtb_Hover_Durum <= 35.0)) {
        rtb_Hover_Durum = 35.0;
      }

      /* '<S2>:1:273' */
      F_weight = rtb_Hover_Durum * 9.80665;

      /* '<S2>:1:278' */
      /* '<S2>:1:281' */
      /* '<S2>:1:284' */
      vx_use = Ucus_Bilgisayari_DW.UnitDelay1_DSTATE / 300.0;
      if (!(vx_use >= 0.0)) {
        vx_use = 0.0;
      }

      if (!(vx_use <= 1.0)) {
        vx_use = 1.0;
      }

      F_available = 460.0 * vx_use;

      /* '<S2>:1:289' */
      /* '<S2>:1:292' */
      if (Ucus_Bilgisayari_DW.UnitDelay3_DSTATE >= 0.0) {
        tmp = Ucus_Bilgisayari_DW.UnitDelay3_DSTATE;
      } else {
        tmp = 0.0;
      }

      Ucus_Bilgisayari_DW.F_meas_filt += (tmp - Ucus_Bilgisayari_DW.F_meas_filt)
        * 0.15;
      if (Ucus_Bilgisayari_DW.state_p == 0) {
        /* '<S2>:1:299' */
        /* '<S2>:1:301' */
        qY = Ucus_Bilgisayari_DW.arm_count + 1U;
        if (Ucus_Bilgisayari_DW.arm_count + 1U < Ucus_Bilgisayari_DW.arm_count)
        {
          qY = MAX_uint32_T;
        }

        Ucus_Bilgisayari_DW.arm_count = qY;

        /* '<S2>:1:304' */
        Ucus_Bilgisayari_DW.valve_previous = 0.0;

        /* '<S2>:1:306' */
        Ucus_Bilgisayari_DW.capture_brake_active = false;

        /* '<S2>:1:307' */
        Ucus_Bilgisayari_DW.capture_brake_done = false;
        if ((real_T)Ucus_Bilgisayari_DW.arm_count * 0.01 >= 0.02) {
          /* '<S2>:1:309' */
          /* '<S2>:1:311' */
          Ucus_Bilgisayari_DW.state_p = 1U;
        }

        /* '<S2>:1:315' */
        rtb_Hover_Durum = Ucus_Bilgisayari_DW.state_p;
      } else {
        guard1 = false;
        if (Ucus_Bilgisayari_DW.state_p == 1) {
          /* '<S2>:1:324' */
          /* '<S2>:1:328' */
          /* '<S2>:1:330' */
          if (4.8 - Ucus_Bilgisayari_U.zpos >= 0.0) {
            /* '<S2>:1:333' */
            /* '<S2>:1:335' */
            /* '<S2>:1:340' */
            if (4.8 - Ucus_Bilgisayari_U.zpos >= 0.0) {
              tmp = 4.8 - Ucus_Bilgisayari_U.zpos;
            } else {
              tmp = 0.0;
            }

            rtb_Hover_Suresi = sqrt(tmp);
            if ((rtb_Hover_Suresi >= 1.4) || rtIsNaN(rtb_Hover_Suresi)) {
              rtb_Hover_Suresi = 1.4;
            }
          } else {
            /* '<S2>:1:345' */
            /* '<S2>:1:350' */
            if (-(4.8 - Ucus_Bilgisayari_U.zpos) >= 0.0) {
              tmp = -(4.8 - Ucus_Bilgisayari_U.zpos);
            } else {
              tmp = 0.0;
            }

            rtb_F_Hedef_N = sqrt(tmp);
            if ((rtb_F_Hedef_N >= 0.55) || rtIsNaN(rtb_F_Hedef_N)) {
              rtb_Hover_Suresi = -0.55;
            } else {
              rtb_Hover_Suresi = -rtb_F_Hedef_N;
            }
          }

          /* '<S2>:1:355' */
          /* '<S2>:1:359' */
          /* '<S2>:1:364' */
          vx_use = (4.8 - Ucus_Bilgisayari_U.zpos) * 0.7 + (rtb_Hover_Suresi -
            Ucus_Bilgisayari_U.zvel) * 4.0;
          if (!(vx_use >= -7.0)) {
            vx_use = -7.0;
          }

          if (!(vx_use <= 3.0)) {
            vx_use = 3.0;
          }

          vx_use = (vx_use + 9.80665) * rtb_Hover_Durum;
          if (Ucus_Bilgisayari_U.zpos >= 3.8) {
            /* '<S2>:1:369' */
            /* '<S2>:1:371' */
            Ucus_Bilgisayari_DW.state_p = 2U;

            /* '<S2>:1:373' */
            Ucus_Bilgisayari_DW.hover_streak_count = 0U;

            /* '<S2>:1:374' */
            Ucus_Bilgisayari_DW.hover_best_count = 0U;

            /* '<S2>:1:376' */
            Ucus_Bilgisayari_DW.capture_brake_active = false;

            /* '<S2>:1:377' */
            Ucus_Bilgisayari_DW.capture_brake_done = false;
          }

          guard1 = true;
        } else if (Ucus_Bilgisayari_DW.state_p == 2) {
          /* '<S2>:1:381' */
          /* '<S2>:1:385' */
          /* '<S2>:1:387' */
          rtb_F_Hedef_N = fabs(4.8 - Ucus_Bilgisayari_U.zpos);
          if (rtb_F_Hedef_N <= 0.15) {
            /* '<S2>:1:392' */
            /* '<S2>:1:394' */
            rtb_Hover_Suresi = 0.0;
          } else {
            /* '<S2>:1:398' */
            /* '<S2>:1:401' */
            rtb_Hover_Suresi = (4.8 - Ucus_Bilgisayari_U.zpos) * 0.9;
            if (!(rtb_Hover_Suresi >= -0.1)) {
              rtb_Hover_Suresi = -0.1;
            }

            if (!(rtb_Hover_Suresi <= 0.1)) {
              rtb_Hover_Suresi = 0.1;
            }
          }

          /* '<S2>:1:408' */
          /* '<S2>:1:412' */
          /* '<S2>:1:419' */
          vx_use = (rtb_Hover_Suresi - Ucus_Bilgisayari_U.zvel) * 5.0;
          if (!(vx_use >= -5.0)) {
            vx_use = -5.0;
          }

          if (!(vx_use <= 1.5)) {
            vx_use = 1.5;
          }

          vx_use = rtb_Hover_Durum * vx_use + F_weight;
          if ((!Ucus_Bilgisayari_DW.capture_brake_active) &&
              (!Ucus_Bilgisayari_DW.capture_brake_done) &&
              (Ucus_Bilgisayari_U.zpos < 4.8) && (Ucus_Bilgisayari_U.zvel > 0.35))
          {
            /* '<S2>:1:425' */
            /* '<S2>:1:426' */
            /* '<S2>:1:428' */
            /* '<S2>:1:429' */
            /* '<S2>:1:431' */
            Ucus_Bilgisayari_DW.capture_brake_active = true;
          }

          if (Ucus_Bilgisayari_DW.capture_brake_active) {
            /* '<S2>:1:446' */
            /* '<S2>:1:448' */
            /* '<S2>:1:451' */
            /* '<S2>:1:454' */
            if (Ucus_Bilgisayari_U.zvel >= 0.0) {
              tmp = Ucus_Bilgisayari_U.zvel;
            } else {
              tmp = 0.0;
            }

            vx_use = -4.0 * tmp;
            if (!(vx_use >= -4.0)) {
              vx_use = -4.0;
            }

            vx_use = (vx_use + 9.80665) * rtb_Hover_Durum;
            if (Ucus_Bilgisayari_U.zvel <= 0.05) {
              /* '<S2>:1:459' */
              /* '<S2>:1:461' */
              Ucus_Bilgisayari_DW.capture_brake_active = false;

              /* '<S2>:1:462' */
              Ucus_Bilgisayari_DW.capture_brake_done = true;
            }
          }

          if ((rtb_F_Hedef_N <= 0.15) && (fabs(Ucus_Bilgisayari_U.zvel) <= 0.25))
          {
            /* '<S2>:1:471' */
            /* '<S2>:1:472' */
            /* '<S2>:1:476' */
            qY = Ucus_Bilgisayari_DW.hover_streak_count + 1U;
            if (Ucus_Bilgisayari_DW.hover_streak_count + 1U <
                Ucus_Bilgisayari_DW.hover_streak_count) {
              qY = MAX_uint32_T;
            }

            Ucus_Bilgisayari_DW.hover_streak_count = qY;
            if (Ucus_Bilgisayari_DW.hover_streak_count >
                Ucus_Bilgisayari_DW.hover_best_count) {
              /* '<S2>:1:479' */
              /* '<S2>:1:481' */
              Ucus_Bilgisayari_DW.hover_best_count =
                Ucus_Bilgisayari_DW.hover_streak_count;
            }
          } else {
            /* '<S2>:1:488' */
            Ucus_Bilgisayari_DW.hover_streak_count = 0U;
          }

          guard1 = true;
        } else {
          /* '<S2>:1:499' */
          Ucus_Bilgisayari_DW.state_p = 4U;

          /* '<S2>:1:501' */
          /* '<S2>:1:503' */
          /* '<S2>:1:504' */
          /* '<S2>:1:506' */
          Ucus_Bilgisayari_DW.valve_previous = 0.0;

          /* '<S2>:1:508' */
          Ucus_Bilgisayari_DW.capture_brake_active = false;

          /* '<S2>:1:509' */
          Ucus_Bilgisayari_DW.capture_brake_done = true;

          /* '<S2>:1:511' */
          rtb_Hover_Durum = 4.0;

          /* '<S2>:1:514' */
          /* '<S2>:1:517' */
        }

        if (guard1) {
          /* '<S2>:1:526' */
          if (vx_use <= F_available) {
            rtb_F_Hedef_N = vx_use;
          } else {
            rtb_F_Hedef_N = F_available;
          }

          /* '<S2>:1:535' */
          if (rtb_F_Hedef_N <= 0.0) {
            /* '<S2>:1:538' */
            /* '<S2>:1:540' */
            /* '<S2>:1:542' */
            Ucus_Bilgisayari_DW.valve_previous = 0.0;
          } else {
            if (F_available > 1.0) {
              /* '<S2>:1:548' */
              /* '<S2>:1:550' */
              valve_ff = rtb_F_Hedef_N / F_available;
            } else {
              /* '<S2>:1:555' */
              valve_ff = 1.0;
            }

            /* '<S2>:1:559' */
            if (!(valve_ff <= 1.0)) {
              valve_ff = 1.0;
            }

            if (Ucus_Bilgisayari_DW.F_meas_filt < 20.0) {
              /* '<S2>:1:564' */
              /* '<S2>:1:566' */
              /* '<S2>:1:569' */
              Ucus_Bilgisayari_DW.valve_previous = valve_ff;
            } else {
              /* '<S2>:1:576' */
              /* '<S2>:1:580' */
              /* '<S2>:1:585' */
              /* '<S2>:1:590' */
              /* '<S2>:1:595' */
              valve_ff += (rtb_F_Hedef_N - Ucus_Bilgisayari_DW.F_meas_filt) *
                0.001;
              if (!(valve_ff >= 0.0)) {
                valve_ff = 0.0;
              }

              if (!(valve_ff <= 1.0)) {
                valve_ff = 1.0;
              }

              if ((!(valve_ff <= Ucus_Bilgisayari_DW.valve_previous + 0.01)) &&
                  (!rtIsNaN(Ucus_Bilgisayari_DW.valve_previous + 0.01))) {
                valve_ff = Ucus_Bilgisayari_DW.valve_previous + 0.01;
              }

              if ((!(valve_ff >= Ucus_Bilgisayari_DW.valve_previous - 0.015)) &&
                  (!rtIsNaN(Ucus_Bilgisayari_DW.valve_previous - 0.015))) {
                valve_ff = Ucus_Bilgisayari_DW.valve_previous - 0.015;
              }

              if (!(valve_ff >= 0.0)) {
                valve_ff = 0.0;
              }

              if (!(valve_ff <= 1.0)) {
                valve_ff = 1.0;
              }

              /* '<S2>:1:598' */
              Ucus_Bilgisayari_DW.valve_previous = valve_ff;
            }
          }

          if ((valve_ff >= 0.995) && (Ucus_Bilgisayari_DW.F_meas_filt <
               rtb_F_Hedef_N - 15.0) && (rtb_F_Hedef_N > 0.8 * F_weight)) {
            /* '<S2>:1:608' */
            /* '<S2>:1:609' */
            /* '<S2>:1:611' */
            sensor_valid = true;
          } else {
            sensor_valid = false;
          }

          if ((Ucus_Bilgisayari_DW.state_p == 2) && (F_available < F_weight)) {
            /* '<S2>:1:617' */
            /* '<S2>:1:618' */
            physical_shortage = true;
          } else {
            physical_shortage = false;
          }

          if (sensor_valid || physical_shortage) {
            if (Ucus_Bilgisayari_DW.shortage_count < 50U) {
              /* '<S2>:1:626' */
              /* '<S2>:1:629' */
              Ucus_Bilgisayari_DW.shortage_count++;
            }

            if (Ucus_Bilgisayari_DW.shortage_count >= 50U) {
              /* '<S2>:1:634' */
              /* '<S2>:1:637' */
              Ucus_Bilgisayari_DW.shortage_latch = true;
            }
          } else {
            /* '<S2>:1:643' */
            Ucus_Bilgisayari_DW.shortage_count = 0U;
          }

          /* '<S2>:1:650' */
          rtb_Hover_Durum = Ucus_Bilgisayari_DW.state_p;

          /* '<S2>:1:653' */
          rtb_Hover_Suresi = (real_T)Ucus_Bilgisayari_DW.hover_best_count * 0.01;

          /* '<S2>:1:656' */
        }
      }
    }
  } else {
    /* '<S2>:1:211' */
    /* '<S2>:1:213' */
    /* '<S2>:1:214' */
    Ucus_Bilgisayari_DW.valve_previous = 0.0;

    /* '<S2>:1:216' */
    Ucus_Bilgisayari_DW.capture_brake_active = false;
  }

  /* End of MATLAB Function: '<Root>/Hover_Kontrol' */
  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/Vy'
   *  Inport: '<Root>/XPOS'
   *  Inport: '<Root>/vx'
   *  Inport: '<Root>/yPOS'
   */
  /* MATLAB Function 'MATLAB Function1': '<S3>:1' */
  /* '<S3>:1:18' */
  /* '<S3>:1:19' */
  /* '<S3>:1:21' */
  /* '<S3>:1:22' */
  /* '<S3>:1:25' */
  /* '<S3>:1:26' */
  /* '<S3>:1:29' */
  /* '<S3>:1:30' */
  /* '<S3>:1:33' */
  /* '<S3>:1:34' */
  /* '<S3>:1:35' */
  /* '<S3>:1:38' */
  /* '<S3>:1:44' */
  /* '<S3>:1:47' */
  /* '<S3>:1:50' */
  /* '<S3>:1:54' */
  /* '<S3>:1:56' */
  /* '<S3>:1:57' */
  /* '<S3>:1:60' */
  /* '<S3>:1:61' */
  /* '<S3>:1:65' */
  rtb_F_Hedef_N = 0.0;

  /* '<S3>:1:66' */
  F_weight = 0.0;
  if ((!rtIsInf(Ucus_Bilgisayari_U.XPOS)) && (!rtIsNaN(Ucus_Bilgisayari_U.XPOS))
      && ((!rtIsInf(Ucus_Bilgisayari_U.yPOS)) && (!rtIsNaN
        (Ucus_Bilgisayari_U.yPOS))) && ((!rtIsInf(Ucus_Bilgisayari_U.vx)) &&
       (!rtIsNaN(Ucus_Bilgisayari_U.vx))) && ((!rtIsInf(Ucus_Bilgisayari_U.Vy)) &&
       (!rtIsNaN(Ucus_Bilgisayari_U.Vy)))) {
    /* '<S3>:1:70' */
    sensor_valid = true;

    /* '<S3>:1:76' */
    F_available = Ucus_Bilgisayari_U.XPOS;

    /* '<S3>:1:77' */
    F_weight = Ucus_Bilgisayari_U.yPOS;
    rtb_F_Hedef_N = fabs(Ucus_Bilgisayari_U.XPOS);
    if ((rtb_F_Hedef_N < 0.03) && (fabs(Ucus_Bilgisayari_U.vx) < 0.03)) {
      /* '<S3>:1:81' */
      /* '<S3>:1:82' */
      F_available = 0.0;

      /* '<S3>:1:83' */
      vx_use = 0.0;
    } else {
      /* '<S3>:1:85' */
      vx_use = Ucus_Bilgisayari_U.vx;
    }

    tmp = fabs(Ucus_Bilgisayari_U.yPOS);
    if ((tmp < 0.03) && (fabs(Ucus_Bilgisayari_U.Vy) < 0.03)) {
      /* '<S3>:1:88' */
      /* '<S3>:1:89' */
      F_weight = 0.0;

      /* '<S3>:1:90' */
      vy_use = 0.0;
    } else {
      /* '<S3>:1:92' */
      vy_use = Ucus_Bilgisayari_U.Vy;
    }

    /* '<S3>:1:97' */
    /* '<S3>:1:98' */
    /* '<S3>:1:102' */
    /* '<S3>:1:105' */
    /* '<S3>:1:108' */
    edge_x = (fabs(Ucus_Bilgisayari_U.XPOS + vx_use) - 1.0) / 2.0;
    if (!(edge_x >= 0.0)) {
      edge_x = 0.0;
    }

    if (!(edge_x <= 1.0)) {
      edge_x = 1.0;
    }

    /* '<S3>:1:109' */
    edge_y = (fabs(Ucus_Bilgisayari_U.yPOS + vy_use) - 1.0) / 2.0;
    if (!(edge_y >= 0.0)) {
      edge_y = 0.0;
    }

    if (!(edge_y <= 1.0)) {
      edge_y = 1.0;
    }

    /* '<S3>:1:113' */
    /* '<S3>:1:114' */
    /* '<S3>:1:116' */
    /* '<S3>:1:117' */
    /* '<S3>:1:121' */
    F_available = -(edge_x * 0.5 + 0.25) * F_available - (edge_x *
      0.70000000000000007 + 0.9) * vx_use;

    /* '<S3>:1:122' */
    ay_cmd = -(edge_y * 0.5 + 0.25) * F_weight - (edge_y * 0.70000000000000007 +
      0.9) * vy_use;
    if ((fabs(F_available) >= 0.256796167756467) && (fabs(ay_cmd) >=
         0.256796167756467) && ((Ucus_Bilgisayari_U.XPOS * vx_use > 0.0) ||
         (Ucus_Bilgisayari_U.yPOS * vy_use > 0.0))) {
      /* '<S3>:1:127' */
      /* '<S3>:1:128' */
      /* '<S3>:1:129' */
      /* '<S3>:1:134' */
      F_weight = 0.09599310885968812;
    } else {
      sensor_valid = false;

      /* '<S3>:1:136' */
      F_weight = 0.08377580409572781;
    }

    /* '<S3>:1:141' */
    a_edge_max = 9.80665 * tan(F_weight);

    /* '<S3>:1:143' */
    /* '<S3>:1:144' */
    if (rtb_F_Hedef_N >= 3.0) {
      /* '<S3>:1:148' */
      if (Ucus_Bilgisayari_U.XPOS > 0.0) {
        /* '<S3>:1:150' */
        /* '<S3>:1:151' */
        F_available = -a_edge_max;
      } else if (Ucus_Bilgisayari_U.XPOS < 0.0) {
        /* '<S3>:1:152' */
        /* '<S3>:1:153' */
        F_available = a_edge_max;
      }
    } else if ((Ucus_Bilgisayari_U.XPOS * vx_use > 0.0) && ((vx_use * vx_use /
                 0.8 + rtb_F_Hedef_N) + 0.4 >= 3.0)) {
      /* '<S3>:1:156' */
      /* '<S3>:1:157' */
      if (vx_use > 0.0) {
        /* '<S3>:1:159' */
        /* '<S3>:1:160' */
        F_available = -a_edge_max;
      } else if (vx_use < 0.0) {
        /* '<S3>:1:161' */
        /* '<S3>:1:162' */
        F_available = a_edge_max;
      }
    }

    if (tmp >= 3.0) {
      /* '<S3>:1:169' */
      if (Ucus_Bilgisayari_U.yPOS > 0.0) {
        /* '<S3>:1:171' */
        /* '<S3>:1:172' */
        ay_cmd = -a_edge_max;
      } else if (Ucus_Bilgisayari_U.yPOS < 0.0) {
        /* '<S3>:1:173' */
        /* '<S3>:1:174' */
        ay_cmd = a_edge_max;
      }
    } else if ((Ucus_Bilgisayari_U.yPOS * vy_use > 0.0) && ((vy_use * vy_use /
                 0.8 + tmp) + 0.4 >= 3.0)) {
      /* '<S3>:1:177' */
      /* '<S3>:1:178' */
      if (vy_use > 0.0) {
        /* '<S3>:1:180' */
        /* '<S3>:1:181' */
        ay_cmd = -a_edge_max;
      } else if (vy_use < 0.0) {
        /* '<S3>:1:182' */
        /* '<S3>:1:183' */
        ay_cmd = a_edge_max;
      }
    }

    /* '<S3>:1:190' */
    /* '<S3>:1:192' */
    if (edge_x >= edge_y) {
      edge_y = edge_x;
    }

    vx_use = (F_weight - 0.069813170079773182) * edge_y + 0.069813170079773182;
    if (sensor_valid) {
      /* '<S3>:1:197' */
      vx_use = 0.09599310885968812;
    }

    /* '<S3>:1:200' */
    vx_use = 9.80665 * tan(vx_use);

    /* '<S3>:1:203' */
    /* '<S3>:1:204' */
    /* '<S3>:1:211' */
    /* '<S3>:1:212' */
    /* '<S3>:1:214' */
    /* '<S3>:1:215' */
    /* '<S3>:1:219' */
    sensor_valid = !rtIsNaN(-vx_use);
    if ((!(ay_cmd >= -vx_use)) && sensor_valid) {
      ay_cmd = -vx_use;
    }

    physical_shortage = rtIsNaN(vx_use);
    if ((!(ay_cmd <= vx_use)) && (!physical_shortage)) {
      ay_cmd = vx_use;
    }

    rtb_F_Hedef_N = -rt_atan2d_snf(ay_cmd, 9.80665);
    if (!(rtb_F_Hedef_N >= -F_weight)) {
      rtb_F_Hedef_N = -F_weight;
    }

    if (!(rtb_F_Hedef_N <= F_weight)) {
      rtb_F_Hedef_N = F_weight;
    }

    /* '<S3>:1:220' */
    if ((!(F_available >= -vx_use)) && sensor_valid) {
      F_available = -vx_use;
    }

    if ((F_available <= vx_use) || physical_shortage) {
      vx_use = F_available;
    }

    vx_use = rt_atan2d_snf(vx_use, 9.80665);
    if (!(vx_use >= -F_weight)) {
      vx_use = -F_weight;
    }

    if (vx_use <= F_weight) {
      F_weight = vx_use;
    }
  } else {
    /* '<S3>:1:70' */
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */

  /* RateLimiter: '<Root>/Rate Limiter' */
  vx_use = rtb_F_Hedef_N - Ucus_Bilgisayari_DW.PrevY_c;
  tmp = Ucus_Bilgisayari_P.RateLimiter_RisingLim * Ucus_Bilgisayari_period;
  if (vx_use > tmp) {
    F_available = tmp + Ucus_Bilgisayari_DW.PrevY_c;
  } else if (vx_use < Ucus_Bilgisayari_P.RateLimiter_FallingLim *
             Ucus_Bilgisayari_period) {
    F_available = Ucus_Bilgisayari_P.RateLimiter_FallingLim *
      Ucus_Bilgisayari_period + Ucus_Bilgisayari_DW.PrevY_c;
  } else {
    F_available = rtb_F_Hedef_N;
  }

  Ucus_Bilgisayari_DW.PrevY_c = F_available;

  /* End of RateLimiter: '<Root>/Rate Limiter' */

  /* Gain: '<Root>/Gain5' */
  vx_use = Ucus_Bilgisayari_P.Gain5_Gain * F_available;

  /* RateLimiter: '<Root>/Rate Limiter1' */
  vx_use = F_weight - Ucus_Bilgisayari_DW.PrevY_n;
  tmp = Ucus_Bilgisayari_P.RateLimiter1_RisingLim * Ucus_Bilgisayari_period;
  if (vx_use > tmp) {
    rtb_F_Hedef_N = tmp + Ucus_Bilgisayari_DW.PrevY_n;
  } else if (vx_use < Ucus_Bilgisayari_P.RateLimiter1_FallingLim *
             Ucus_Bilgisayari_period) {
    rtb_F_Hedef_N = Ucus_Bilgisayari_P.RateLimiter1_FallingLim *
      Ucus_Bilgisayari_period + Ucus_Bilgisayari_DW.PrevY_n;
  } else {
    rtb_F_Hedef_N = F_weight;
  }

  Ucus_Bilgisayari_DW.PrevY_n = rtb_F_Hedef_N;

  /* End of RateLimiter: '<Root>/Rate Limiter1' */

  /* Gain: '<Root>/Gain6' */
  vx_use = Ucus_Bilgisayari_P.Gain6_Gain * rtb_F_Hedef_N;

  /* Gain: '<Root>/Gain' incorporates:
   *  Inport: '<Root>/Pitch'
   */
  vx_use = Ucus_Bilgisayari_P.Gain_Gain * Ucus_Bilgisayari_U.Pitch;

  /* Gain: '<Root>/Gain3' incorporates:
   *  Inport: '<Root>/Pitch'
   */
  rtb_Hover_Durum = Ucus_Bilgisayari_P.Gain3_Gain * Ucus_Bilgisayari_U.Pitch;

  /* Gain: '<Root>/Gain4' incorporates:
   *  Inport: '<Root>/Yaw'
   */
  rtb_UnitDelay = Ucus_Bilgisayari_P.Gain4_Gain * Ucus_Bilgisayari_U.Yaw;

  /* MATLAB Function: '<Root>/RCS_Denge_Kontrol' incorporates:
   *  Inport: '<Root>/pitch_rate'
   *  Inport: '<Root>/yaw_rate'
   *  Inport: '<Root>/zpos'
   *  Inport: '<Root>/zvel'
   */
  /* MATLAB Function 'RCS_Denge_Kontrol': '<S6>:1' */
  /* '<S6>:1:55' */
  /* '<S6>:1:74' */
  /* '<S6>:1:78' */
  /* '<S6>:1:82' */
  /* '<S6>:1:83' */
  /* '<S6>:1:85' */
  /* '<S6>:1:86' */
  /* '<S6>:1:188' */
  /* '<S6>:1:190' */
  /* '<S6>:1:191' */
  /* '<S6>:1:245' */
  /* '<S6>:1:283' */
  qY = Ucus_Bilgisayari_DW.sample_count + 1U;
  if (Ucus_Bilgisayari_DW.sample_count + 1U < Ucus_Bilgisayari_DW.sample_count)
  {
    qY = MAX_uint32_T;
  }

  Ucus_Bilgisayari_DW.sample_count = qY;
  if ((!rtIsInf(F_available)) && (!rtIsNaN(F_available)) && ((!rtIsInf
        (rtb_F_Hedef_N)) && (!rtIsNaN(rtb_F_Hedef_N))) && ((!rtIsInf
        (rtb_Hover_Durum)) && (!rtIsNaN(rtb_Hover_Durum))) && ((!rtIsInf
        (rtb_UnitDelay)) && (!rtIsNaN(rtb_UnitDelay))) && ((!rtIsInf
        (Ucus_Bilgisayari_U.pitch_rate)) && (!rtIsNaN
        (Ucus_Bilgisayari_U.pitch_rate))) && ((!rtIsInf
        (Ucus_Bilgisayari_U.yaw_rate)) && (!rtIsNaN(Ucus_Bilgisayari_U.yaw_rate)))
      && ((!rtIsInf(Ucus_Bilgisayari_U.zpos)) && (!rtIsNaN
        (Ucus_Bilgisayari_U.zpos))) && ((!rtIsInf(Ucus_Bilgisayari_U.zvel)) && (
        !rtIsNaN(Ucus_Bilgisayari_U.zvel)))) {
    /* '<S6>:1:291' */
    /* '<S6>:1:292' */
    /* '<S6>:1:293' */
    /* '<S6>:1:294' */
    /* '<S6>:1:295' */
    /* '<S6>:1:296' */
    /* '<S6>:1:297' */
    /* '<S6>:1:298' */
  } else {
    /* '<S6>:1:300' */
    /* '<S6>:1:302' */
    Ucus_Bilgisayari_DW.hard_fault = true;

    /* '<S6>:1:303' */
  }

  if (Ucus_Bilgisayari_DW.hard_fault) {
    /* '<S6>:1:307' */
    /* '<S6>:1:309' */
    /* '<S6>:1:741' */
    Ucus_Bilgisayari_DW.P.desired = 0;

    /* '<S6>:1:742' */
    Ucus_Bilgisayari_DW.P.applied = 0;

    /* '<S6>:1:743' */
    Ucus_Bilgisayari_DW.P.pending = 0;

    /* '<S6>:1:745' */
    Ucus_Bilgisayari_DW.P.dead_active = false;

    /* '<S6>:1:747' */
    Ucus_Bilgisayari_DW.P.mode = 0U;

    /* '<S6>:1:749' */
    Ucus_Bilgisayari_DW.P.track_active = false;

    /* '<S6>:1:750' */
    Ucus_Bilgisayari_DW.P.track_sign = 0;

    /* '<S6>:1:752' */
    Ucus_Bilgisayari_DW.P.track_age = 0U;

    /* '<S6>:1:753' */
    Ucus_Bilgisayari_DW.P.track_pulse_n = 0U;

    /* '<S6>:1:754' */
    Ucus_Bilgisayari_DW.P.track_cooldown = 0U;

    /* '<S6>:1:310' */
    /* '<S6>:1:741' */
    Ucus_Bilgisayari_DW.Y.desired = 0;

    /* '<S6>:1:742' */
    Ucus_Bilgisayari_DW.Y.applied = 0;

    /* '<S6>:1:743' */
    Ucus_Bilgisayari_DW.Y.pending = 0;

    /* '<S6>:1:745' */
    Ucus_Bilgisayari_DW.Y.dead_active = false;

    /* '<S6>:1:747' */
    Ucus_Bilgisayari_DW.Y.mode = 0U;

    /* '<S6>:1:749' */
    Ucus_Bilgisayari_DW.Y.track_active = false;

    /* '<S6>:1:750' */
    Ucus_Bilgisayari_DW.Y.track_sign = 0;

    /* '<S6>:1:752' */
    Ucus_Bilgisayari_DW.Y.track_age = 0U;

    /* '<S6>:1:753' */
    Ucus_Bilgisayari_DW.Y.track_pulse_n = 0U;

    /* '<S6>:1:754' */
    Ucus_Bilgisayari_DW.Y.track_cooldown = 0U;

    /* '<S6>:1:312' */
    /* '<S6>:1:2185' */
    Ucus_Bilgisayari_B.V1 = 0.0;

    /* '<S6>:1:2188' */
    Ucus_Bilgisayari_B.V3 = 0.0;

    /* '<S6>:1:2191' */
    Ucus_Bilgisayari_B.V5 = 0.0;

    /* '<S6>:1:2194' */
    Ucus_Bilgisayari_B.V7 = 0.0;

    /* '<S6>:1:2197' */
    rtb_UnitDelay2 = 0.0;

    /* '<S6>:1:2198' */
    rtb_UnitDelay4 = 0.0;

    /* '<S6>:1:312' */
    /* '<S6>:1:315' */
    /* '<S6>:1:316' */
    /* '<S6>:1:318' */
    /* '<S6>:1:320' */
    /* '<S6>:1:321' */
    /* '<S6>:1:323' */
  } else {
    /* '<S6>:1:333' */
    if ((Ucus_Bilgisayari_U.zpos <= 0.4313) && (fabs(Ucus_Bilgisayari_U.zvel) <=
         0.5)) {
      /* '<S6>:1:335' */
      /* '<S6>:1:336' */
      /* '<S6>:1:338' */
      Ucus_Bilgisayari_DW.landed_latch = true;
    }

    if (Ucus_Bilgisayari_DW.landed_latch) {
      /* '<S6>:1:342' */
      /* '<S6>:1:344' */
      /* '<S6>:1:741' */
      Ucus_Bilgisayari_DW.P.desired = 0;

      /* '<S6>:1:742' */
      Ucus_Bilgisayari_DW.P.applied = 0;

      /* '<S6>:1:743' */
      Ucus_Bilgisayari_DW.P.pending = 0;

      /* '<S6>:1:745' */
      Ucus_Bilgisayari_DW.P.dead_active = false;

      /* '<S6>:1:747' */
      Ucus_Bilgisayari_DW.P.mode = 0U;

      /* '<S6>:1:749' */
      Ucus_Bilgisayari_DW.P.track_active = false;

      /* '<S6>:1:750' */
      Ucus_Bilgisayari_DW.P.track_sign = 0;

      /* '<S6>:1:752' */
      Ucus_Bilgisayari_DW.P.track_age = 0U;

      /* '<S6>:1:753' */
      Ucus_Bilgisayari_DW.P.track_pulse_n = 0U;

      /* '<S6>:1:754' */
      Ucus_Bilgisayari_DW.P.track_cooldown = 0U;

      /* '<S6>:1:345' */
      /* '<S6>:1:741' */
      Ucus_Bilgisayari_DW.Y.desired = 0;

      /* '<S6>:1:742' */
      Ucus_Bilgisayari_DW.Y.applied = 0;

      /* '<S6>:1:743' */
      Ucus_Bilgisayari_DW.Y.pending = 0;

      /* '<S6>:1:745' */
      Ucus_Bilgisayari_DW.Y.dead_active = false;

      /* '<S6>:1:747' */
      Ucus_Bilgisayari_DW.Y.mode = 0U;

      /* '<S6>:1:749' */
      Ucus_Bilgisayari_DW.Y.track_active = false;

      /* '<S6>:1:750' */
      Ucus_Bilgisayari_DW.Y.track_sign = 0;

      /* '<S6>:1:752' */
      Ucus_Bilgisayari_DW.Y.track_age = 0U;

      /* '<S6>:1:753' */
      Ucus_Bilgisayari_DW.Y.track_pulse_n = 0U;

      /* '<S6>:1:754' */
      Ucus_Bilgisayari_DW.Y.track_cooldown = 0U;

      /* '<S6>:1:347' */
      /* '<S6>:1:2185' */
      Ucus_Bilgisayari_B.V1 = 0.0;

      /* '<S6>:1:2188' */
      Ucus_Bilgisayari_B.V3 = 0.0;

      /* '<S6>:1:2191' */
      Ucus_Bilgisayari_B.V5 = 0.0;

      /* '<S6>:1:2194' */
      Ucus_Bilgisayari_B.V7 = 0.0;

      /* '<S6>:1:2197' */
      rtb_UnitDelay2 = 0.0;

      /* '<S6>:1:2198' */
      rtb_UnitDelay4 = 0.0;

      /* '<S6>:1:347' */
      /* '<S6>:1:350' */
      /* '<S6>:1:351' */
      /* '<S6>:1:353' */
      /* '<S6>:1:355' */
      /* '<S6>:1:356' */
      /* '<S6>:1:358' */
    } else {
      /* '<S6>:1:368' */
      rtb_UnitDelay2 = -rtb_Hover_Durum * 57.295779513082323;

      /* '<S6>:1:372' */
      rtb_UnitDelay4 = -rtb_UnitDelay * 57.295779513082323;

      /* '<S6>:1:380' */
      rtb_UnitDelay6 = -F_available * 57.295779513082323;
      if (rtb_UnitDelay6 < -5.5) {
        /* '<S6>:1:2144' */
        /* '<S6>:1:2146' */
        rtb_UnitDelay6 = -5.5;
      } else if (rtb_UnitDelay6 > 5.5) {
        /* '<S6>:1:2148' */
        /* '<S6>:1:2150' */
        rtb_UnitDelay6 = 5.5;
      } else {
        /* '<S6>:1:2154' */
      }

      /* '<S6>:1:386' */
      rtb_Hover_Durum = -rtb_F_Hedef_N * 57.295779513082323;
      if (rtb_Hover_Durum < -5.5) {
        /* '<S6>:1:2144' */
        /* '<S6>:1:2146' */
        rtb_Hover_Durum = -5.5;
      } else if (rtb_Hover_Durum > 5.5) {
        /* '<S6>:1:2148' */
        /* '<S6>:1:2150' */
        rtb_Hover_Durum = 5.5;
      } else {
        /* '<S6>:1:2154' */
      }

      /* '<S6>:1:396' */
      /* '<S6>:1:400' */
      /* '<S6>:1:404' */
      Ucus_Bilgisayari_updateRates(&Ucus_Bilgisayari_DW.P, rtb_UnitDelay2,
        -Ucus_Bilgisayari_U.pitch_rate * 57.295779513082323, 0.01, 1.0, 0.25,
        &F_available, &vx_use);

      /* '<S6>:1:404' */
      /* '<S6>:1:413' */
      Ucus_Bilgisayari_updateRates(&Ucus_Bilgisayari_DW.Y, rtb_UnitDelay4,
        -Ucus_Bilgisayari_U.yaw_rate * 57.295779513082323, 0.01, 1.0, 0.25,
        &rtb_Hover_Suresi, &rtb_F_Hedef_N);

      /* '<S6>:1:413' */
      /* '<S6>:1:426' */
      if (Ucus_Bilgisayari_U.zpos - 0.4013 >= 0.0) {
        F_weight = Ucus_Bilgisayari_U.zpos - 0.4013;
      } else {
        F_weight = 0.0;
      }

      if ((F_weight >= 0.1) && (F_weight <= 20.0) && (Ucus_Bilgisayari_U.zvel <
           -0.1)) {
        /* '<S6>:1:430' */
        /* '<S6>:1:431' */
        /* '<S6>:1:432' */
        physical_shortage = true;

        /* '<S6>:1:436' */
        F_weight /= -Ucus_Bilgisayari_U.zvel;
        if (!(F_weight <= 8.0)) {
          F_weight = 8.0;
        }
      } else {
        physical_shortage = false;

        /* '<S6>:1:443' */
        F_weight = 0.0;
      }

      /* '<S6>:1:451' */
      if (fabs(F_available) >= 7.0) {
        /* '<S6>:1:818' */
        /* '<S6>:1:820' */
        vx_use = F_available;
      } else if (!(fabs(vx_use) >= 1.5)) {
        /* '<S6>:1:828' */
        vx_use = 0.0;
      } else {
        /* '<S6>:1:822' */
        /* '<S6>:1:824' */
      }

      /* '<S6>:1:458' */
      if (fabs(rtb_Hover_Suresi) >= 7.0) {
        /* '<S6>:1:818' */
        /* '<S6>:1:820' */
        rtb_F_Hedef_N = rtb_Hover_Suresi;
      } else if (!(fabs(rtb_F_Hedef_N) >= 1.5)) {
        /* '<S6>:1:828' */
        rtb_F_Hedef_N = 0.0;
      } else {
        /* '<S6>:1:822' */
        /* '<S6>:1:824' */
      }

      if (physical_shortage) {
        /* '<S6>:1:467' */
        vx_use = vx_use * F_weight + rtb_UnitDelay2;
        if (vx_use < -180.0) {
          /* '<S6>:1:2144' */
          /* '<S6>:1:2146' */
          vx_use = -180.0;
        } else if (vx_use > 180.0) {
          /* '<S6>:1:2148' */
          /* '<S6>:1:2150' */
          vx_use = 180.0;
        } else {
          /* '<S6>:1:2154' */
        }

        /* '<S6>:1:474' */
        rtb_F_Hedef_N = rtb_F_Hedef_N * F_weight + rtb_UnitDelay4;
        if (rtb_F_Hedef_N < -180.0) {
          /* '<S6>:1:2144' */
          /* '<S6>:1:2146' */
          rtb_F_Hedef_N = -180.0;
        } else if (rtb_F_Hedef_N > 180.0) {
          /* '<S6>:1:2148' */
          /* '<S6>:1:2150' */
          rtb_F_Hedef_N = 180.0;
        } else {
          /* '<S6>:1:2154' */
        }
      } else {
        /* '<S6>:1:483' */
        vx_use = rtb_UnitDelay2;

        /* '<S6>:1:484' */
        rtb_F_Hedef_N = rtb_UnitDelay4;
      }

      /* '<S6>:1:492' */
      Ucus_Bilgisayar_ageAxisCounters(&Ucus_Bilgisayari_DW.P, 100);

      /* '<S6>:1:493' */
      Ucus_Bilgisayar_ageAxisCounters(&Ucus_Bilgisayari_DW.Y, 100);

      /* '<S6>:1:499' */
      sensor_valid = Ucus_Bilgisayari_selectAxis(&Ucus_Bilgisayari_DW.P,
        rtb_UnitDelay2, F_available, vx_use, physical_shortage, 5.0, 10.0, 7.0,
        221.0, 0.3, 0.15, 8.0, 0.5, 3, 0.075, 3.0, 0.25, 8, 30, 30, 40, 4, 8);

      /* '<S6>:1:499' */
      /* '<S6>:1:529' */
      physical_shortage = Ucus_Bilgisayari_selectAxis(&Ucus_Bilgisayari_DW.Y,
        rtb_UnitDelay4, rtb_Hover_Suresi, rtb_F_Hedef_N, physical_shortage, 5.0,
        10.0, 7.0, 221.0, 0.3, 0.15, 8.0, 0.5, 3, 0.075, 3.0, 0.25, 8, 30, 30,
        40, 4, 8);

      /* '<S6>:1:529' */
      if (sensor_valid || physical_shortage) {
        /* '<S6>:1:499' */
        /* '<S6>:1:529' */
        /* '<S6>:1:557' */
        Ucus_Bilgisayari_DW.hard_fault = true;

        /* '<S6>:1:558' */
        /* '<S6>:1:560' */
        /* '<S6>:1:741' */
        Ucus_Bilgisayari_DW.P.desired = 0;

        /* '<S6>:1:742' */
        Ucus_Bilgisayari_DW.P.applied = 0;

        /* '<S6>:1:743' */
        Ucus_Bilgisayari_DW.P.pending = 0;

        /* '<S6>:1:745' */
        Ucus_Bilgisayari_DW.P.dead_active = false;

        /* '<S6>:1:747' */
        Ucus_Bilgisayari_DW.P.mode = 0U;

        /* '<S6>:1:749' */
        Ucus_Bilgisayari_DW.P.track_active = false;

        /* '<S6>:1:750' */
        Ucus_Bilgisayari_DW.P.track_sign = 0;

        /* '<S6>:1:752' */
        Ucus_Bilgisayari_DW.P.track_age = 0U;

        /* '<S6>:1:753' */
        Ucus_Bilgisayari_DW.P.track_pulse_n = 0U;

        /* '<S6>:1:754' */
        Ucus_Bilgisayari_DW.P.track_cooldown = 0U;

        /* '<S6>:1:561' */
        /* '<S6>:1:741' */
        Ucus_Bilgisayari_DW.Y.desired = 0;

        /* '<S6>:1:742' */
        Ucus_Bilgisayari_DW.Y.applied = 0;

        /* '<S6>:1:743' */
        Ucus_Bilgisayari_DW.Y.pending = 0;

        /* '<S6>:1:745' */
        Ucus_Bilgisayari_DW.Y.dead_active = false;

        /* '<S6>:1:747' */
        Ucus_Bilgisayari_DW.Y.mode = 0U;

        /* '<S6>:1:749' */
        Ucus_Bilgisayari_DW.Y.track_active = false;

        /* '<S6>:1:750' */
        Ucus_Bilgisayari_DW.Y.track_sign = 0;

        /* '<S6>:1:752' */
        Ucus_Bilgisayari_DW.Y.track_age = 0U;

        /* '<S6>:1:753' */
        Ucus_Bilgisayari_DW.Y.track_pulse_n = 0U;

        /* '<S6>:1:754' */
        Ucus_Bilgisayari_DW.Y.track_cooldown = 0U;
      } else {
        /* '<S6>:1:574' */
        /* '<S6>:1:576' */
        Ucus_Bil_applyReferenceTracking(&Ucus_Bilgisayari_DW.P, rtb_UnitDelay2,
          F_available, rtb_UnitDelay6, 5.0, 8.0, 1.5, 0.7, 0.075, 3.5, 2.5, 1.2,
          35, 45, 55, 5, 8, 9);

        /* '<S6>:1:596' */
        Ucus_Bil_applyReferenceTracking(&Ucus_Bilgisayari_DW.Y, rtb_UnitDelay4,
          rtb_Hover_Suresi, rtb_Hover_Durum, 5.0, 8.0, 1.5, 0.7, 0.075, 3.5, 2.5,
          1.2, 35, 45, 55, 5, 8, 9);
      }

      if (Ucus_Bilgisayari_DW.sample_count <= 0U) {
        /* '<S6>:1:622' */
        /* '<S6>:1:624' */
        Ucus_Bilgisayari_DW.P.desired = 0;

        /* '<S6>:1:625' */
        Ucus_Bilgisayari_DW.Y.desired = 0;
      }

      /* '<S6>:1:633' */
      sensor_valid = Ucus_Bilgisay_processAxisOutput(&Ucus_Bilgisayari_DW.P, 8,
        8, 8);

      /* '<S6>:1:633' */
      /* '<S6>:1:640' */
      physical_shortage = Ucus_Bilgisay_processAxisOutput(&Ucus_Bilgisayari_DW.Y,
        8, 8, 8);

      /* '<S6>:1:640' */
      if (sensor_valid || physical_shortage) {
        /* '<S6>:1:633' */
        /* '<S6>:1:640' */
        /* '<S6>:1:649' */
        Ucus_Bilgisayari_DW.hard_fault = true;

        /* '<S6>:1:650' */
        /* '<S6>:1:652' */
        /* '<S6>:1:741' */
        Ucus_Bilgisayari_DW.P.desired = 0;

        /* '<S6>:1:742' */
        Ucus_Bilgisayari_DW.P.applied = 0;

        /* '<S6>:1:743' */
        Ucus_Bilgisayari_DW.P.pending = 0;

        /* '<S6>:1:745' */
        Ucus_Bilgisayari_DW.P.dead_active = false;

        /* '<S6>:1:747' */
        Ucus_Bilgisayari_DW.P.mode = 0U;

        /* '<S6>:1:749' */
        Ucus_Bilgisayari_DW.P.track_active = false;

        /* '<S6>:1:750' */
        Ucus_Bilgisayari_DW.P.track_sign = 0;

        /* '<S6>:1:752' */
        Ucus_Bilgisayari_DW.P.track_age = 0U;

        /* '<S6>:1:753' */
        Ucus_Bilgisayari_DW.P.track_pulse_n = 0U;

        /* '<S6>:1:754' */
        Ucus_Bilgisayari_DW.P.track_cooldown = 0U;

        /* '<S6>:1:653' */
        /* '<S6>:1:741' */
        Ucus_Bilgisayari_DW.Y.desired = 0;

        /* '<S6>:1:742' */
        Ucus_Bilgisayari_DW.Y.applied = 0;

        /* '<S6>:1:743' */
        Ucus_Bilgisayari_DW.Y.pending = 0;

        /* '<S6>:1:745' */
        Ucus_Bilgisayari_DW.Y.dead_active = false;

        /* '<S6>:1:747' */
        Ucus_Bilgisayari_DW.Y.mode = 0U;

        /* '<S6>:1:749' */
        Ucus_Bilgisayari_DW.Y.track_active = false;

        /* '<S6>:1:750' */
        Ucus_Bilgisayari_DW.Y.track_sign = 0;

        /* '<S6>:1:752' */
        Ucus_Bilgisayari_DW.Y.track_age = 0U;

        /* '<S6>:1:753' */
        Ucus_Bilgisayari_DW.Y.track_pulse_n = 0U;

        /* '<S6>:1:754' */
        Ucus_Bilgisayari_DW.Y.track_cooldown = 0U;
      }

      /* '<S6>:1:661' */
      /* '<S6>:1:2185' */
      Ucus_Bilgisayari_B.V1 = (Ucus_Bilgisayari_DW.P.applied == -1);

      /* '<S6>:1:2188' */
      Ucus_Bilgisayari_B.V3 = (Ucus_Bilgisayari_DW.P.applied == 1);

      /* '<S6>:1:2191' */
      Ucus_Bilgisayari_B.V5 = (Ucus_Bilgisayari_DW.Y.applied == -1);

      /* '<S6>:1:2194' */
      Ucus_Bilgisayari_B.V7 = (Ucus_Bilgisayari_DW.Y.applied == 1);

      /* '<S6>:1:2197' */
      rtb_UnitDelay2 = Ucus_Bilgisayari_DW.P.applied;

      /* '<S6>:1:2198' */
      rtb_UnitDelay4 = Ucus_Bilgisayari_DW.Y.applied;

      /* '<S6>:1:661' */
      /* '<S6>:1:666' */
      /* '<S6>:1:667' */
      /* '<S6>:1:669' */
    }
  }

  /* End of MATLAB Function: '<Root>/RCS_Denge_Kontrol' */

  /* Scope: '<Root>/Upitch_Kontrol ' */
  /* '<S6>:1:55' */

  /* MATLAB Function: '<Root>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/m_guncel'
   *  Inport: '<Root>/zpos'
   *  Inport: '<Root>/zvel'
   *  MATLAB Function: '<Root>/Otomatik_Gorev_Secimi'
   *  UnitDelay: '<Root>/Unit Delay1'
   */
  /* MATLAB Function 'MATLAB Function2': '<S4>:1' */
  /* '<S4>:1:23' */
  /* '<S4>:1:26' */
  /* '<S4>:1:29' */
  /* '<S4>:1:33' */
  /* '<S4>:1:37' */
  /* '<S4>:1:38' */
  /* '<S4>:1:40' */
  /* '<S4>:1:41' */
  /* '<S4>:1:43' */
  /* '<S4>:1:65' */
  /* '<S4>:1:67' */
  /* '<S4>:1:68' */
  /* '<S4>:1:72' */
  /* '<S4>:1:73' */
  /* '<S4>:1:75' */
  /* '<S4>:1:76' */
  /* '<S4>:1:80' */
  /* '<S4>:1:82' */
  /* '<S4>:1:83' */
  /* '<S4>:1:85' */
  /* '<S4>:1:89' */
  /* '<S4>:1:90' */
  /* '<S4>:1:94' */
  /* '<S4>:1:95' */
  /* '<S4>:1:97' */
  /* '<S4>:1:99' */
  /* '<S4>:1:100' */
  sensor_valid = ((!rtIsInf(Ucus_Bilgisayari_U.zpos)) && (!rtIsNaN
    (Ucus_Bilgisayari_U.zpos)));
  if (sensor_valid && ((!rtIsInf(Ucus_Bilgisayari_U.zvel)) && (!rtIsNaN
        (Ucus_Bilgisayari_U.zvel))) && ((!rtIsInf(Ucus_Bilgisayari_U.m_guncel)) &&
       (!rtIsNaN(Ucus_Bilgisayari_U.m_guncel))) && ((!rtIsInf
        (Ucus_Bilgisayari_DW.UnitDelay1_DSTATE)) && (!rtIsNaN
        (Ucus_Bilgisayari_DW.UnitDelay1_DSTATE)))) {
    /* '<S4>:1:105' */
    /* '<S4>:1:106' */
    /* '<S4>:1:107' */
    /* '<S4>:1:108' */
    /* '<S4>:1:120' */
    rtb_Hover_Durum = Ucus_Bilgisayari_U.m_guncel;
    if (Ucus_Bilgisayari_U.m_guncel < 27.5) {
      /* '<S4>:1:122' */
      /* '<S4>:1:124' */
      rtb_Hover_Durum = 27.5;
    } else if (Ucus_Bilgisayari_U.m_guncel > 35.0) {
      /* '<S4>:1:126' */
      /* '<S4>:1:128' */
      rtb_Hover_Durum = 35.0;
    }

    if (Ucus_Bilgisayari_DW.UnitDelay1_DSTATE > 10000.0) {
      /* '<S4>:1:134' */
      /* '<S4>:1:137' */
      rtb_UnitDelay2 = Ucus_Bilgisayari_DW.UnitDelay1_DSTATE / 100000.0;
    } else {
      /* '<S4>:1:141' */
      rtb_UnitDelay2 = Ucus_Bilgisayari_DW.UnitDelay1_DSTATE;
    }

    if (rtb_UnitDelay2 < 0.0) {
      /* '<S4>:1:145' */
      /* '<S4>:1:147' */
      rtb_UnitDelay2 = 0.0;
    } else if (rtb_UnitDelay2 > 350.0) {
      /* '<S4>:1:149' */
      /* '<S4>:1:151' */
      rtb_UnitDelay2 = 350.0;
    }

    /* '<S4>:1:157' */
    rtb_UnitDelay2 /= 300.0;
    if (rtb_UnitDelay2 > 1.0) {
      /* '<S4>:1:164' */
      /* '<S4>:1:166' */
      rtb_UnitDelay2 = 1.0;
    }

    /* '<S4>:1:170' */
    /* '<S4>:1:174' */
    /* '<S4>:1:178' */
    F_available = 460.0 * rtb_UnitDelay2;
    if (F_available < 1.0) {
      /* '<S4>:1:182' */
      /* '<S4>:1:184' */
      rtb_UnitDelay2 = 0.0;
    } else {
      /* '<S4>:1:192' */
      rtb_UnitDelay2 = Ucus_Bilgisayari_U.zpos - 0.4013;
      if (Ucus_Bilgisayari_U.zpos - 0.4013 < 0.0) {
        /* '<S4>:1:194' */
        /* '<S4>:1:195' */
        rtb_UnitDelay2 = 0.0;
      }

      /* '<S4>:1:199' */
      rtb_UnitDelay4 = -Ucus_Bilgisayari_U.zvel;
      if (-Ucus_Bilgisayari_U.zvel < 0.0) {
        /* '<S4>:1:201' */
        /* '<S4>:1:202' */
        rtb_UnitDelay4 = 0.0;
      }

      if ((Ucus_Bilgisayari_DW.brake_phase == 0) && (rtb_UnitDelay2 > 7.0) &&
          (fabs(Ucus_Bilgisayari_U.zvel) < 0.2)) {
        /* '<S4>:1:207' */
        /* '<S4>:1:208' */
        /* '<S4>:1:209' */
        /* '<S4>:1:211' */
        Ucus_Bilgisayari_DW.valve_prev = 0.0;

        /* '<S4>:1:212' */
        Ucus_Bilgisayari_DW.valve_filt = 0.0;
      }

      /* '<S4>:1:218' */
      if (rtb_UnitDelay2 - 0.02 >= 0.0) {
        tmp = rtb_UnitDelay2 - 0.02;
      } else {
        tmp = 0.0;
      }

      rtb_UnitDelay6 = sqrt(2.8 * tmp + 0.040000000000000008);
      if (rtb_UnitDelay6 > 4.5) {
        /* '<S4>:1:223' */
        /* '<S4>:1:225' */
        rtb_UnitDelay6 = 4.5;
      }

      /* '<S4>:1:234' */
      rtb_UnitDelay6 = rtb_UnitDelay4 - rtb_UnitDelay6;

      /* '<S4>:1:239' */
      /* '<S4>:1:244' */
      rtb_Hover_Suresi = 0.97 * F_available / rtb_Hover_Durum - 9.80665;

      /* '<S4>:1:248' */
      /* '<S4>:1:252' */
      if (rtb_Hover_Suresi > 0.15) {
        /* '<S4>:1:257' */
        /* '<S4>:1:259' */
        rtb_Hover_Suresi = ((rtb_UnitDelay4 + 1.1767979999999998) *
                            (rtb_UnitDelay4 + 1.1767979999999998) -
                            0.040000000000000008) / (2.0 * rtb_Hover_Suresi);
        if (rtb_Hover_Suresi < 0.0) {
          /* '<S4>:1:264' */
          /* '<S4>:1:265' */
          rtb_Hover_Suresi = 0.0;
        }

        /* '<S4>:1:268' */
        rtb_Hover_Suresi = ((rtb_UnitDelay4 * 0.12 + 0.070607879999999984) +
                            rtb_Hover_Suresi) + 0.6;
      } else {
        /* '<S4>:1:277' */
        rtb_Hover_Suresi = rtb_UnitDelay2 + 0.5;
      }

      if (Ucus_Bilgisayari_DW.brake_phase == 0) {
        /* '<S4>:1:288' */
        if (((rtb_UnitDelay2 <= 0.65 * rtb_Hover_Suresi) || (rtb_UnitDelay4 >=
              6.5)) && (rtb_UnitDelay4 > 0.2)) {
          /* '<S4>:1:295' */
          /* '<S4>:1:296' */
          /* '<S4>:1:297' */
          /* '<S4>:1:299' */
          Ucus_Bilgisayari_DW.brake_phase = 1U;
        }
      } else if (Ucus_Bilgisayari_DW.brake_phase == 1) {
        /* '<S4>:1:303' */
        if (((rtb_UnitDelay2 <= 1.5) && (rtb_UnitDelay4 <= 0.4)) ||
            (Ucus_Bilgisayari_U.zvel > 0.05)) {
          /* '<S4>:1:310' */
          /* '<S4>:1:311' */
          /* '<S4>:1:312' */
          /* '<S4>:1:314' */
          Ucus_Bilgisayari_DW.brake_phase = 2U;
        }
      } else {
        /* '<S4>:1:322' */
        Ucus_Bilgisayari_DW.brake_phase = 2U;
      }

      /* '<S4>:1:328' */
      rtb_Hover_Suresi = rtb_UnitDelay4 / 0.35;
      if (rtb_Hover_Suresi < 0.0) {
        /* '<S4>:1:331' */
        /* '<S4>:1:333' */
        rtb_Hover_Suresi = 0.0;
      } else if (rtb_Hover_Suresi > 1.0) {
        /* '<S4>:1:335' */
        /* '<S4>:1:337' */
        rtb_Hover_Suresi = 1.0;
      }

      /* '<S4>:1:341' */
      /* '<S4>:1:348' */
      rtb_F_Hedef_N = rtb_UnitDelay2;
      if (rtb_UnitDelay2 < 0.15) {
        /* '<S4>:1:350' */
        /* '<S4>:1:351' */
        rtb_F_Hedef_N = 0.15;
      }

      /* '<S4>:1:354' */
      rtb_F_Hedef_N = (rtb_UnitDelay4 * rtb_UnitDelay4 - 0.040000000000000008) /
        (2.0 * rtb_F_Hedef_N);
      if (rtb_F_Hedef_N < 0.0) {
        /* '<S4>:1:359' */
        /* '<S4>:1:360' */
        rtb_F_Hedef_N = 0.0;
      }

      /* '<S4>:1:365' */
      rtb_F_Hedef_N = 0.8 * rtb_F_Hedef_N + 1.8 * rtb_UnitDelay6;

      /* '<S4>:1:370' */
      F_weight = F_available / rtb_Hover_Durum - 9.80665;
      if (rtb_F_Hedef_N > F_weight) {
        /* '<S4>:1:373' */
        /* '<S4>:1:375' */
        rtb_F_Hedef_N = F_weight;
      } else if (rtb_F_Hedef_N < -9.80665) {
        /* '<S4>:1:377' */
        /* '<S4>:1:379' */
        rtb_F_Hedef_N = -9.80665;
      }

      /* '<S4>:1:385' */
      rtb_F_Hedef_N = (rtb_F_Hedef_N + 9.80665) * rtb_Hover_Durum;
      if (rtb_F_Hedef_N < 0.0) {
        /* '<S4>:1:388' */
        /* '<S4>:1:390' */
        rtb_F_Hedef_N = 0.0;
      } else if (rtb_F_Hedef_N > F_available) {
        /* '<S4>:1:392' */
        /* '<S4>:1:394' */
        rtb_F_Hedef_N = F_available;
      }

      /* '<S4>:1:398' */
      /* '<S4>:1:403' */
      rtb_UnitDelay6 /= 0.35;
      if (rtb_UnitDelay6 < 0.0) {
        /* '<S4>:1:406' */
        /* '<S4>:1:408' */
        rtb_UnitDelay6 = 0.0;
      } else if (rtb_UnitDelay6 > 1.0) {
        /* '<S4>:1:410' */
        /* '<S4>:1:412' */
        rtb_UnitDelay6 = 1.0;
      }

      /* '<S4>:1:416' */
      /* '<S4>:1:421' */
      if (Ucus_Bilgisayari_DW.brake_phase == 0) {
        /* '<S4>:1:426' */
        /* '<S4>:1:429' */
        rtb_UnitDelay6 = 0.0;
      } else if (Ucus_Bilgisayari_DW.brake_phase == 1) {
        /* '<S4>:1:431' */
        /* '<S4>:1:435' */
        rtb_UnitDelay6 = 1.0;
      } else if (rtb_UnitDelay2 > 1.5) {
        /* '<S4>:1:441' */
        /* '<S4>:1:443' */
        rtb_UnitDelay6 = 0.0;
      } else {
        /* '<S4>:1:448' */
        vx_use = rtb_F_Hedef_N / F_available;
        rtb_F_Hedef_N = (3.0 - 2.0 * rtb_UnitDelay6) * (rtb_UnitDelay6 *
          rtb_UnitDelay6);
        if ((vx_use >= rtb_F_Hedef_N) || rtIsNaN(rtb_F_Hedef_N)) {
          rtb_F_Hedef_N = vx_use;
        }

        rtb_UnitDelay6 = (3.0 - 2.0 * rtb_Hover_Suresi) * (rtb_Hover_Suresi *
          rtb_Hover_Suresi) * rtb_F_Hedef_N;
      }

      /* '<S4>:1:455' */
      if (Ucus_Bilgisayari_DW.brake_phase == 2) {
        /* '<S4>:1:461' */
        /* '<S4>:1:463' */
        rtb_Hover_Suresi = (1.5 - rtb_UnitDelay2) / 1.5;
        if (rtb_Hover_Suresi < 0.0) {
          /* '<S4>:1:467' */
          /* '<S4>:1:469' */
          rtb_Hover_Suresi = 0.0;
        } else if (rtb_Hover_Suresi > 1.0) {
          /* '<S4>:1:471' */
          /* '<S4>:1:473' */
          rtb_Hover_Suresi = 1.0;
        }

        /* '<S4>:1:477' */
        /* '<S4>:1:489' */
        rtb_F_Hedef_N = (0.4 - rtb_UnitDelay4) / 0.2;
        if (rtb_F_Hedef_N < 0.0) {
          /* '<S4>:1:493' */
          /* '<S4>:1:495' */
          rtb_F_Hedef_N = 0.0;
        } else if (rtb_F_Hedef_N > 1.0) {
          /* '<S4>:1:497' */
          /* '<S4>:1:499' */
          rtb_F_Hedef_N = 1.0;
        }

        /* '<S4>:1:503' */
        /* '<S4>:1:508' */
        rtb_Hover_Suresi = (3.0 - 2.0 * rtb_Hover_Suresi) * (rtb_Hover_Suresi *
          rtb_Hover_Suresi) * ((3.0 - 2.0 * rtb_F_Hedef_N) * (rtb_F_Hedef_N *
          rtb_F_Hedef_N));

        /* '<S4>:1:514' */
        rtb_Hover_Durum *= 9.78665;
        if (rtb_Hover_Durum < 0.0) {
          /* '<S4>:1:517' */
          /* '<S4>:1:519' */
          rtb_Hover_Durum = 0.0;
        } else if (rtb_Hover_Durum > F_available) {
          /* '<S4>:1:521' */
          /* '<S4>:1:523' */
          rtb_Hover_Durum = F_available;
        }

        /* '<S4>:1:527' */
        /* '<S4>:1:530' */
        /* '<S4>:1:534' */
        rtb_F_Hedef_N = rtb_Hover_Durum / F_available;
        if ((rtb_UnitDelay6 <= rtb_F_Hedef_N) || rtIsNaN(rtb_F_Hedef_N)) {
          rtb_F_Hedef_N = rtb_UnitDelay6;
        }

        rtb_UnitDelay6 = (1.0 - rtb_Hover_Suresi) * rtb_UnitDelay6 +
          rtb_Hover_Suresi * rtb_F_Hedef_N;
      }

      /* '<S4>:1:544' */
      rtb_Hover_Durum = (0.01 - rtb_UnitDelay2) / 0.01;
      if (rtb_Hover_Durum < 0.0) {
        /* '<S4>:1:548' */
        /* '<S4>:1:550' */
        rtb_Hover_Durum = 0.0;
      } else if (rtb_Hover_Durum > 1.0) {
        /* '<S4>:1:552' */
        /* '<S4>:1:554' */
        rtb_Hover_Durum = 1.0;
      }

      /* '<S4>:1:558' */
      /* '<S4>:1:570' */
      rtb_UnitDelay4 = (0.06 - rtb_UnitDelay4) / 0.02;
      if (rtb_UnitDelay4 < 0.0) {
        /* '<S4>:1:574' */
        /* '<S4>:1:576' */
        rtb_UnitDelay4 = 0.0;
      } else if (rtb_UnitDelay4 > 1.0) {
        /* '<S4>:1:578' */
        /* '<S4>:1:580' */
        rtb_UnitDelay4 = 1.0;
      }

      /* '<S4>:1:584' */
      /* '<S4>:1:589' */
      /* '<S4>:1:593' */
      rtb_UnitDelay6 *= 1.0 - (3.0 - 2.0 * rtb_Hover_Durum) * (rtb_Hover_Durum *
        rtb_Hover_Durum) * ((3.0 - 2.0 * rtb_UnitDelay4) * (rtb_UnitDelay4 *
        rtb_UnitDelay4));
      if (rtb_UnitDelay2 <= 0.02) {
        /* '<S4>:1:599' */
        /* '<S4>:1:601' */
        rtb_UnitDelay2 = Ucus_Bilgisayari_U.zvel / 0.4;
        if (rtb_UnitDelay2 < 0.0) {
          /* '<S4>:1:604' */
          /* '<S4>:1:606' */
          rtb_UnitDelay2 = 0.0;
        } else if (rtb_UnitDelay2 > 1.0) {
          /* '<S4>:1:608' */
          /* '<S4>:1:610' */
          rtb_UnitDelay2 = 1.0;
        }

        /* '<S4>:1:614' */
        rtb_UnitDelay2 = (3.0 - 2.0 * rtb_UnitDelay2) * (rtb_UnitDelay2 *
          rtb_UnitDelay2);
      } else {
        /* '<S4>:1:621' */
        rtb_UnitDelay2 = 0.0;
      }

      /* '<S4>:1:625' */
      rtb_UnitDelay6 *= 1.0 - rtb_UnitDelay2;
      if (rtb_UnitDelay6 < 0.0) {
        /* '<S4>:1:631' */
        /* '<S4>:1:633' */
        rtb_UnitDelay6 = 0.0;
      } else if (rtb_UnitDelay6 > 1.0) {
        /* '<S4>:1:635' */
        /* '<S4>:1:637' */
        rtb_UnitDelay6 = 1.0;
      }

      /* '<S4>:1:643' */
      Ucus_Bilgisayari_DW.valve_filt += (rtb_UnitDelay6 -
        Ucus_Bilgisayari_DW.valve_filt) * 0.18;

      /* '<S4>:1:648' */
      rtb_UnitDelay2 = Ucus_Bilgisayari_DW.valve_filt -
        Ucus_Bilgisayari_DW.valve_prev;
      if (rtb_UnitDelay2 > 0.04) {
        /* '<S4>:1:651' */
        /* '<S4>:1:653' */
        rtb_UnitDelay2 = Ucus_Bilgisayari_DW.valve_prev + 0.04;
      } else if (rtb_UnitDelay2 < -0.06) {
        /* '<S4>:1:657' */
        /* '<S4>:1:659' */
        rtb_UnitDelay2 = Ucus_Bilgisayari_DW.valve_prev - 0.06;
      } else {
        /* '<S4>:1:665' */
        rtb_UnitDelay2 = Ucus_Bilgisayari_DW.valve_filt;
      }

      if (rtb_UnitDelay2 < 0.0) {
        /* '<S4>:1:671' */
        /* '<S4>:1:673' */
        rtb_UnitDelay2 = 0.0;
      } else if (rtb_UnitDelay2 > 1.0) {
        /* '<S4>:1:675' */
        /* '<S4>:1:677' */
        rtb_UnitDelay2 = 1.0;
      }

      /* '<S4>:1:681' */
      Ucus_Bilgisayari_DW.valve_prev = rtb_UnitDelay2;
    }
  } else {
    /* '<S4>:1:110' */
    /* '<S4>:1:112' */
    rtb_UnitDelay2 = 0.0;
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function2' */

  /* MATLAB Function: '<Root>/Otomatik_Gorev_Secimi' */
  /* MATLAB Function 'Otomatik_Gorev_Secimi': '<S5>:1' */
  /* '<S5>:1:15' */
  Ucus_Bilgisayari_B.Valve_Cmd = 0.0;

  /* '<S5>:1:16' */
  if (sensor_valid && ((!rtIsInf(Ucus_Bilgisayari_U.zvel)) && (!rtIsNaN
        (Ucus_Bilgisayari_U.zvel))) && (!rtIsNaN(rtb_UnitDelay2))) {
    /* '<S5>:1:18' */
    guard1 = false;
    if (!Ucus_Bilgisayari_DW.gorev_kilitli) {
      /* '<S5>:1:22' */
      if ((Ucus_Bilgisayari_U.zpos >= 1.0) && (Ucus_Bilgisayari_U.zpos <= 3.5) &&
          (Ucus_Bilgisayari_U.zvel > -2.0) && (Ucus_Bilgisayari_U.zvel < 2.0)) {
        /* '<S5>:1:23' */
        /* '<S5>:1:24' */
        Ucus_Bilgisayari_DW.gorev_p = 2U;

        /* '<S5>:1:25' */
        Ucus_Bilgisayari_DW.gorev_kilitli = true;
        guard1 = true;
      } else if (Ucus_Bilgisayari_U.zpos > 3.5) {
        /* '<S5>:1:26' */
        /* '<S5>:1:27' */
        Ucus_Bilgisayari_DW.gorev_p = 1U;

        /* '<S5>:1:28' */
        Ucus_Bilgisayari_DW.gorev_kilitli = true;
        guard1 = true;
      } else if (Ucus_Bilgisayari_U.zvel <= -2.0) {
        /* '<S5>:1:29' */
        /* '<S5>:1:30' */
        Ucus_Bilgisayari_DW.gorev_p = 1U;

        /* '<S5>:1:31' */
        Ucus_Bilgisayari_DW.gorev_kilitli = true;
        guard1 = true;
      } else {
        /* '<S5>:1:33' */
        Ucus_Bilgisayari_B.Valve_Cmd = 0.0;

        /* '<S5>:1:34' */
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (Ucus_Bilgisayari_DW.gorev_p == 1) {
        /* '<S5>:1:39' */
        /* '<S5>:1:40' */
        valve_ff = rtb_UnitDelay2;
      } else if (Ucus_Bilgisayari_DW.gorev_p != 2) {
        /* '<S5>:1:44' */
        valve_ff = 0.0;
      } else {
        /* '<S5>:1:41' */
        /* '<S5>:1:42' */
      }

      /* '<S5>:1:47' */
      Ucus_Bilgisayari_B.Valve_Cmd = valve_ff;

      /* '<S5>:1:48' */
    }
  } else {
    /* '<S5>:1:18' */
  }

  /* Gain: '<Root>/Gain2' incorporates:
   *  Inport: '<Root>/Yaw'
   */
  vx_use = Ucus_Bilgisayari_P.Gain2_Gain * Ucus_Bilgisayari_U.Yaw;

  /* SampleTimeMath: '<S1>/TSamp' incorporates:
   *  Inport: '<Root>/Yaw'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *   */
  Ucus_Bilgisayari_B.TSamp = Ucus_Bilgisayari_U.Yaw *
    Ucus_Bilgisayari_P.TSamp_WtEt;

  /* Sum: '<S1>/Diff' incorporates:
   *  UnitDelay: '<S1>/UD'
   */
  vx_use = Ucus_Bilgisayari_B.TSamp - Ucus_Bilgisayari_DW.UD_DSTATE;
}

/* Model update function */
void Ucus_Bilgisayari_update(void)
{
  /* Update for UnitDelay: '<Root>/Unit Delay' */
  Ucus_Bilgisayari_DW.UnitDelay_DSTATE = Ucus_Bilgisayari_B.V1;

  /* Update for UnitDelay: '<Root>/Unit Delay2' */
  Ucus_Bilgisayari_DW.UnitDelay2_DSTATE = Ucus_Bilgisayari_B.V3;

  /* Update for UnitDelay: '<Root>/Unit Delay4' */
  Ucus_Bilgisayari_DW.UnitDelay4_DSTATE = Ucus_Bilgisayari_B.V5;

  /* Update for UnitDelay: '<Root>/Unit Delay6' */
  Ucus_Bilgisayari_DW.UnitDelay6_DSTATE = Ucus_Bilgisayari_B.V7;

  /* Update for UnitDelay: '<Root>/Unit Delay8' */
  Ucus_Bilgisayari_DW.UnitDelay8_DSTATE = Ucus_Bilgisayari_B.Valve_Cmd;

  /* Update for UnitDelay: '<Root>/Unit Delay1' incorporates:
   *  Inport: '<Root>/P_main_bar'
   */
  Ucus_Bilgisayari_DW.UnitDelay1_DSTATE = Ucus_Bilgisayari_U.P_main_bar;

  /* Update for UnitDelay: '<Root>/Unit Delay3' incorporates:
   *  Inport: '<Root>/Gercek_Itki_N'
   */
  Ucus_Bilgisayari_DW.UnitDelay3_DSTATE = Ucus_Bilgisayari_U.Gercek_Itki_N;

  /* Update for UnitDelay: '<S1>/UD' */
  Ucus_Bilgisayari_DW.UD_DSTATE = Ucus_Bilgisayari_B.TSamp;

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.01s, 0.0s] */
    if ((rtmGetTFinal(Ucus_Bilgisayari_M)!=-1) &&
        !((rtmGetTFinal(Ucus_Bilgisayari_M)-Ucus_Bilgisayari_M->Timing.taskTime0)
          > Ucus_Bilgisayari_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Ucus_Bilgisayari_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++Ucus_Bilgisayari_M->Timing.clockTick0)) {
    ++Ucus_Bilgisayari_M->Timing.clockTickH0;
  }

  Ucus_Bilgisayari_M->Timing.taskTime0 = Ucus_Bilgisayari_M->Timing.clockTick0 *
    Ucus_Bilgisayari_M->Timing.stepSize0 +
    Ucus_Bilgisayari_M->Timing.clockTickH0 *
    Ucus_Bilgisayari_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void Ucus_Bilgisayari_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Ucus_Bilgisayari_M, 0,
                sizeof(RT_MODEL_Ucus_Bilgisayari_T));
  /* Embedded adaptation: run continuously; the original GRT build stopped
   * its desktop simulation after 7 seconds. */
  rtmSetTFinal(Ucus_Bilgisayari_M, -1.0);
  Ucus_Bilgisayari_M->Timing.stepSize0 = 0.01;

  /* block I/O */
  (void) memset(((void *) &Ucus_Bilgisayari_B), 0,
                sizeof(B_Ucus_Bilgisayari_T));

  /* states (dwork) */
  (void) memset((void *)&Ucus_Bilgisayari_DW, 0,
                sizeof(DW_Ucus_Bilgisayari_T));

  /* external inputs */
  (void)memset(&Ucus_Bilgisayari_U, 0, sizeof(ExtU_Ucus_Bilgisayari_T));

  /* external outputs */
  (void)memset(&Ucus_Bilgisayari_Y, 0, sizeof(ExtY_Ucus_Bilgisayari_T));

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
  Ucus_Bilgisayari_DW.UnitDelay_DSTATE =
    Ucus_Bilgisayari_P.UnitDelay_InitialCondition;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay2' */
  Ucus_Bilgisayari_DW.UnitDelay2_DSTATE =
    Ucus_Bilgisayari_P.UnitDelay2_InitialCondition;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay4' */
  Ucus_Bilgisayari_DW.UnitDelay4_DSTATE =
    Ucus_Bilgisayari_P.UnitDelay4_InitialCondition;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay6' */
  Ucus_Bilgisayari_DW.UnitDelay6_DSTATE =
    Ucus_Bilgisayari_P.UnitDelay6_InitialCondition;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay8' */
  Ucus_Bilgisayari_DW.UnitDelay8_DSTATE =
    Ucus_Bilgisayari_P.UnitDelay8_InitialCondition;

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter2' */
  Ucus_Bilgisayari_DW.PrevY = Ucus_Bilgisayari_P.RateLimiter2_IC;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay1' */
  Ucus_Bilgisayari_DW.UnitDelay1_DSTATE =
    Ucus_Bilgisayari_P.UnitDelay1_InitialCondition;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay3' */
  Ucus_Bilgisayari_DW.UnitDelay3_DSTATE =
    Ucus_Bilgisayari_P.UnitDelay3_InitialCondition;

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter' */
  Ucus_Bilgisayari_DW.PrevY_c = Ucus_Bilgisayari_P.RateLimiter_IC;

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter1' */
  Ucus_Bilgisayari_DW.PrevY_n = Ucus_Bilgisayari_P.RateLimiter1_IC;

  /* InitializeConditions for UnitDelay: '<S1>/UD' */
  Ucus_Bilgisayari_DW.UD_DSTATE =
    Ucus_Bilgisayari_P.DiscreteDerivative_ICPrevScaled;

  /* SystemInitialize for MATLAB Function: '<Root>/Hover_Kontrol' */
  Ucus_Bilgisayari_DW.state_p = 0U;
  Ucus_Bilgisayari_DW.arm_count = 0U;
  Ucus_Bilgisayari_DW.hover_streak_count = 0U;
  Ucus_Bilgisayari_DW.hover_best_count = 0U;
  Ucus_Bilgisayari_DW.F_meas_filt = 0.0;
  Ucus_Bilgisayari_DW.valve_previous = 0.0;
  Ucus_Bilgisayari_DW.shortage_count = 0U;
  Ucus_Bilgisayari_DW.shortage_latch = false;
  Ucus_Bilgisayari_DW.touchdown_latch = false;
  Ucus_Bilgisayari_DW.capture_brake_active = false;
  Ucus_Bilgisayari_DW.capture_brake_done = false;

  /* SystemInitialize for MATLAB Function: '<Root>/RCS_Denge_Kontrol' */
  Ucus_Bilgisayari_initAxis(8, &Ucus_Bilgisayari_DW.P);
  Ucus_Bilgisayari_initAxis(8, &Ucus_Bilgisayari_DW.Y);
  Ucus_Bilgisayari_DW.sample_count = 0U;
  Ucus_Bilgisayari_DW.hard_fault = false;
  Ucus_Bilgisayari_DW.landed_latch = false;

  /* SystemInitialize for MATLAB Function: '<Root>/MATLAB Function2' */
  Ucus_Bilgisayari_DW.valve_prev = 0.0;
  Ucus_Bilgisayari_DW.valve_filt = 0.0;
  Ucus_Bilgisayari_DW.brake_phase = 0U;

  /* SystemInitialize for MATLAB Function: '<Root>/Otomatik_Gorev_Secimi' */
  Ucus_Bilgisayari_DW.gorev_kilitli = false;
  Ucus_Bilgisayari_DW.gorev_p = 0U;
}

/* Model terminate function */
void Ucus_Bilgisayari_terminate(void)
{
  /* (no terminate code required) */
}
