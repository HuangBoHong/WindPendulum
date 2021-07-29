//
// Created by coco24 on 2021/7/28.
//

#ifndef ANOTC_HOST_H
#define ANOTC_HOST_H

#include "stm32f4xx_hal.h"

void anotcSendData_int8_t(UART_HandleTypeDef *huart, uint8_t fun, const int8_t *data, size_t size);
void anotcSendData_int16_t(UART_HandleTypeDef *huart, uint8_t fun, const int16_t *data, size_t size);
void anotcSendData_int32_t(UART_HandleTypeDef *huart, uint8_t fun, const int32_t *data, size_t size);

#endif //ANOTC_HOST_H
