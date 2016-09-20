#ifndef __RC_H
#define __RC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "usart.h"

// for native middle position and -134% to +134%
#define LOW_OFFS 3952
#define MIDDLE_POS 2048.0f

extern volatile uint8_t uart_data[35];
extern volatile uint8_t HAL_UART_ERROR;
extern volatile uint16_t channels[17];
extern volatile uint16_t back_channels[17];
extern volatile uint16_t RC_ERROR;
extern volatile uint8_t RC_RECEIVED;

extern void (*UART_RxCpltCallback)(UART_HandleTypeDef*);
extern void (*UART_ErrorCallback)(UART_HandleTypeDef*);

void HAL_UART_RxCpltCallback_SBUS(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback_SBUS(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback_SRXL(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback_SRXL(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __RC_H*/
