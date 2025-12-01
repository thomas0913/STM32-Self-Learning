#ifndef _RETARGET_H__
#define _RETARGET_H__

#include "stm32f1xx_hal.h"

typedef enum {
    UART_POLLING = 0,    // 同步傳輸
    UART_INTERRUPT,  // 低速非同步傳輸
    UART_DMA         // 高速非同步傳輸
} UART_SYNC_MODE;

void RetargetInit(UART_HandleTypeDef *huart);
int _write(int file, char* ptr, int len);
int _read(int file, char* ptr, int len);

#endif