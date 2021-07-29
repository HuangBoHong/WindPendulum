//
// Created by coco24 on 2021/7/28.
//
#include "anotc_host.h"

void anotcSendData(UART_HandleTypeDef *huart, uint8_t fun, uint8_t *data, uint8_t size) {
  uint8_t txBuffer[size + 4];
  uint8_t *txPtr = txBuffer;
  *txPtr++ = 0x88;
  *txPtr++ = fun;
  *txPtr++ = size;
  for(uint8_t *dataPtr = data; dataPtr < data + size; dataPtr ++)
    *txPtr++ = *dataPtr;
  *txPtr = 0;
  for(uint8_t *sumPtr = txBuffer; sumPtr < txPtr; sumPtr ++)
    *txPtr += *sumPtr;
  HAL_UART_Transmit(huart, txBuffer, size, 10);
}

#define anotcSendDataGenericInt(type) void anotcSendData_##type(UART_HandleTypeDef *huart, uint8_t fun, const type *data, size_t size) { \
  uint8_t u8Size = size * sizeof(type); \
  uint8_t u8Buffer[u8Size]; \
  for(size_t i = 0; i < size; i ++) { \
    for(size_t j = 0; j < sizeof(type); j ++) { \
      u8Buffer[i * sizeof(type) + j] = (data[i] >> ((sizeof(type) - 1 - j) * 8)) & 0xff; \
    } \
  } \
  anotcSendData(huart, fun, u8Buffer, u8Size); \
} \

anotcSendDataGenericInt(int8_t)

anotcSendDataGenericInt(uint16_t)

anotcSendDataGenericInt(int16_t)

anotcSendDataGenericInt(int32_t)
