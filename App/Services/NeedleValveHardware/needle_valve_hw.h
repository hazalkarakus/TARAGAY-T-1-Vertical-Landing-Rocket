#ifndef NEEDLE_VALVE_HW_H
#define NEEDLE_VALVE_HW_H

#include <stdint.h>

void NeedleValveHW_Init(void);
uint16_t NeedleValveHW_ReadPotADC10(void);
uint16_t NeedleValveHW_ReadPotADC10Burst3(void);

/* P82 diagnostic-only independent/cross-check readers. */
uint16_t NeedleValveHW_ReadPotADC2ADC10(void);
uint16_t NeedleValveHW_ReadPotADC2ADC10Burst3(void);
uint16_t NeedleValveHW_ReadPotADC1ShortADC10(void);
uint16_t NeedleValveHW_ReadVrefRaw12(void);

void NeedleValveHW_SetEnabled(uint8_t enabled);
void NeedleValveHW_SetBenchJogArm(uint8_t armed);
void NeedleValveHW_Stop(void);
void NeedleValveHW_Brake(void);
void NeedleValveHW_DriveOpen(uint8_t pwm_0_255);
void NeedleValveHW_DriveClose(uint8_t pwm_0_255);

uint8_t NeedleValveHW_IsInitialized(void);

/* ADC1 production/P79-compatible diagnostics. */
extern volatile uint16_t needle_valve_hw_adc_raw12;
extern volatile uint16_t needle_valve_hw_adc_mv;
extern volatile uint32_t needle_valve_hw_adc_ok_count;
extern volatile uint32_t needle_valve_hw_adc_timeout_count;

/* P82 additional diagnostic channels/counters. */
extern volatile uint16_t needle_valve_hw_adc2_raw12;
extern volatile uint32_t needle_valve_hw_adc2_ok_count;
extern volatile uint32_t needle_valve_hw_adc2_timeout_count;
extern volatile uint16_t needle_valve_hw_adc1_short_raw12;
extern volatile uint32_t needle_valve_hw_adc1_short_ok_count;
extern volatile uint32_t needle_valve_hw_adc1_short_timeout_count;
extern volatile uint16_t needle_valve_hw_vref_raw12;
extern volatile uint16_t needle_valve_hw_vdda_mv_est;
extern volatile uint32_t needle_valve_hw_vref_ok_count;
extern volatile uint32_t needle_valve_hw_vref_timeout_count;

#endif
