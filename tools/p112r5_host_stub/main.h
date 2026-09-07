#ifndef P112R5_HOST_MAIN_H
#define P112R5_HOST_MAIN_H
#include <stdint.h>
extern uint32_t p112r5_host_tick_ms;
extern uint8_t p112r11_host_button;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef int GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef*)0)
#define GPIO_PIN_0 1U
#define GPIO_MODE_INPUT 0U
#define GPIO_PULLDOWN 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define __HAL_RCC_GPIOA_CLK_ENABLE() do{}while(0)
static inline void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g){(void)p;(void)g;}
static inline int HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint32_t pin){(void)p;(void)pin;return p112r11_host_button?GPIO_PIN_SET:GPIO_PIN_RESET;}
static inline uint32_t HAL_GetTick(void) { return p112r5_host_tick_ms; }
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) { }
static inline void __enable_irq(void) { }
#endif
