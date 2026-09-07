#ifndef APP_SERVICES_SERVO_OUTPUT_H
#define APP_SERVICES_SERVO_OUTPUT_H

#include <stdint.h>

/*
 * Tahliye servosu:
 *   nRF komutu 0 -> CLOSED (kapali)
 *   nRF komutu 1 -> OPEN   (acik)
 *
 * Donanim PWM cikisi: PB6 / TIM4_CH1 / AF2
 */
void ServoOutput_Init(void);
void ServoOutput_Update(uint8_t remote_command, uint8_t link_active);
void ServoOutput_ForceClosed(void);

/* CubeIDE Live Expressions icin durum degiskenleri. */
extern volatile uint8_t vent_servo_init_ok;
extern volatile uint8_t vent_servo_remote_command;
extern volatile uint8_t vent_servo_link_active;
extern volatile uint8_t vent_servo_target_open;
extern volatile uint16_t vent_servo_pulse_us;
extern volatile uint32_t vent_servo_update_count;
extern volatile uint32_t vent_servo_command_change_count;


/* V8.14 servo anti-chatter diagnostics. */
extern volatile uint8_t vent_servo_transition_reason;
extern volatile uint32_t vent_servo_link_loss_grace_count;
extern volatile uint32_t vent_servo_link_glitch_suppressed_count;
extern volatile uint8_t vent_servo_link_loss_grace_active;
extern volatile uint32_t vent_servo_link_loss_grace_age_ms;

#endif
