#ifndef STM32F405_LINK_H
#define STM32F405_LINK_H

#include "stm32f4xx_hal.h"

#define FLASH_SECTOR_ADDR  ((uint32_t)0x080E0000)
#define FLASH_SECTOR_NUM   FLASH_SECTOR_11

void link_set_pwm_freq(TIM_HandleTypeDef *htim, uint32_t freq);
uint32_t link_get_max_pwm(TIM_HandleTypeDef *htim);
int link_enable_pwm(TIM_HandleTypeDef *htim);
int link_disable_pwm(TIM_HandleTypeDef *htim);
int link_enable_adc(ADC_HandleTypeDef *hadc);
int link_disable_adc(ADC_HandleTypeDef *hadc);
int link_write_flash(void *data, uint32_t len);

#endif // STM32F405_LINK_H