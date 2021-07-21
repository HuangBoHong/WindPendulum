//
// Created by coco24 on 2021/7/19.
//

#ifndef U8G2TEST_STM32F4XX_HAL_U8G2_H
#define U8G2TEST_STM32F4XX_HAL_U8G2_H

#include "u8g2/u8g2.h"

extern uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
extern uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif //U8G2TEST_STM32F4XX_HAL_U8G2_H
