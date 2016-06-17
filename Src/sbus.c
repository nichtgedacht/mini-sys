#include "sbus.h"

volatile uint8_t uart_data[26] =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
volatile uint8_t HAL_UART_ERROR = 0;
volatile uint16_t channels[16] =
{ 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
volatile uint16_t SBUS_ERROR = 0;
volatile uint8_t SBUS_RECEIVED = 0;

/**
 * @brief  Rx frame transfer completed callbacks, get channel values
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t i;

    if (uart_data[0] == 0x0F && uart_data[24] == 0x00)
    {
        // stolen from https://github.com/zendes/SBUS
        channels[0] = ((uart_data[1] | uart_data[2] << 8) & 0x07FF);
        channels[1] = ((uart_data[2] >> 3 | uart_data[3] << 5) & 0x07FF);
        channels[2] = ((uart_data[3] >> 6 | uart_data[4] << 2 | uart_data[5] << 10) & 0x07FF);
        channels[3] = ((uart_data[5] >> 1 | uart_data[6] << 7) & 0x07FF);
        channels[4] = ((uart_data[6] >> 4 | uart_data[7] << 4) & 0x07FF);
        channels[5] = ((uart_data[7] >> 7 | uart_data[8] << 1 | uart_data[9] << 9) & 0x07FF);
        channels[6] = ((uart_data[9] >> 2 | uart_data[10] << 6) & 0x07FF);
        channels[7] = ((uart_data[10] >> 5 | uart_data[11] << 3) & 0x07FF);
        channels[8] = ((uart_data[12] | uart_data[13] << 8) & 0x07FF);
        channels[9] = ((uart_data[13] >> 3 | uart_data[14] << 5) & 0x07FF);
        channels[10] = ((uart_data[14] >> 6 | uart_data[15] << 2 | uart_data[16] << 10) & 0x07FF);
        channels[11] = ((uart_data[16] >> 1 | uart_data[17] << 7) & 0x07FF);
        channels[12] = ((uart_data[17] >> 4 | uart_data[18] << 4) & 0x07FF);
        channels[13] = ((uart_data[18] >> 7 | uart_data[19] << 1 | uart_data[20] << 9) & 0x07FF);
        channels[14] = ((uart_data[20] >> 2 | uart_data[21] << 6) & 0x07FF);
        channels[15] = ((uart_data[21] >> 5 | uart_data[22] << 3) & 0x07FF);

        SBUS_RECEIVED = 1;
    }
    else
    {
        // if TX is started first the first interrupt read could hit
        // any point within the s-bus frame
        // delay start of next read until we hit the gap between s-bus frames
        // and get in sync
        // HAL_Delay do not work here
        for (i = 0; i < 8000; i++) // 5ms
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        }
        SBUS_ERROR++;

    }
    HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 25);
}

/**
 * @brief  UART error callbacks.
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

    // there is always an overrun error if transmitter started first
    // recover until we are in sync
    HAL_UART_ERROR = huart->ErrorCode;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    // restart receiving
    HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 25);
}
