#include "rc.h"
#include "config.h"

volatile uint8_t uart_data[35] =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
volatile uint8_t HAL_UART_ERROR = 0;
volatile uint16_t channels[17] =
{ 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000 };
volatile uint16_t back_channels[17] =
{ 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000 };
volatile uint16_t RC_ERROR = 0;
volatile uint8_t RC_RECEIVED = 0;

// function pointers to implementation specific callback functions
void (*UART_RxCpltCallback)(UART_HandleTypeDef*) = NULL;
void (*UART_ErrorCallback)(UART_HandleTypeDef*) = NULL;

/**
 * @brief  Wrapper to Rx frame transfer completed callbacks, get channel values
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_RxCpltCallback(huart);
}

/**
 * @brief  Wrapper to UART error callbacks.
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_ErrorCallback(huart);
}

/**
 * @brief  Rx frame transfer completed callbacks, get channel values
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxCpltCallback_SRXL(UART_HandleTypeDef *huart)
{
    uint16_t i;
    // uint16_t j;
    // uint16_t checksum;
    // uint16_t crc = 0;

    if (uart_data[0] == 0xA2)   // minimum check whether the start of frame was catched
    {

        /* Its a pity but checksum calculation is to slow
         for (i = 0; i < 33; i++)
         {
         crc = crc ^ (int16_t) uart_data[i] << 8;
         for (j = 0; j < 8; j++)
         {
         if (crc & 0x8000)
         {
         crc = crc << 1 ^ 0x1021;
         }
         else
         {
         crc = crc << 1;
         }
         }
         }

         checksum = uart_data[33] << 8 | uart_data[34];

         if (checksum == crc)
         {
         */
        channels[0] = uart_data[1] << 8 | uart_data[2];
        channels[1] = uart_data[3] << 8 | uart_data[4];
        channels[2] = uart_data[5] << 8 | uart_data[6];
        channels[3] = uart_data[7] << 8 | uart_data[8];
        channels[4] = uart_data[9] << 8 | uart_data[10];
        channels[5] = uart_data[11] << 8 | uart_data[12];
        channels[6] = uart_data[13] << 8 | uart_data[14];
        channels[7] = uart_data[15] << 8 | uart_data[16];
        channels[8] = uart_data[17] << 8 | uart_data[18];
        channels[9] = uart_data[19] << 8 | uart_data[20];
        channels[10] = uart_data[21] << 8 | uart_data[22];
        channels[11] = uart_data[23] << 8 | uart_data[24];
        channels[12] = uart_data[25] << 8 | uart_data[26];
        channels[13] = uart_data[27] << 8 | uart_data[28];
        channels[14] = uart_data[29] << 8 | uart_data[30];
        channels[15] = uart_data[31] << 8 | uart_data[32];

        for( i = 0; i < 12; i++ )
        {
            if ( rc_rev[i] == 1 )
            {
                channels[i] = 4095 - channels[i];
            }
        }

        RC_RECEIVED = 1;

        /*
         }

         else
         {
         RC_ERROR++;
         }
         */
    }
    else
    {
        // if TX is started first the first interrupt read could hit
        // any point within the srxl frame
        // delay start of next read until we hit the gap between s-bus frames
        // and get in sync
        // HAL_Delay do not work here
        for (i = 0; i < 5000; i++) // ~3.2ms
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        }
        RC_ERROR++;
    }

    HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 35);
}

/**
 * @brief  Rx frame transfer completed callbacks, get channel values
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxCpltCallback_SBUS(UART_HandleTypeDef *huart)
{
    uint16_t i;

    if (uart_data[0] == 0x0F && uart_data[24] == 0x00)
    {
        // stolen from https://github.com/zendes/SBUS
        channels[0] = (((uart_data[1] | uart_data[2] << 8) & 0x07FF) << 1);
        channels[1] = (((uart_data[2] >> 3 | uart_data[3] << 5) & 0x07FF) << 1);
        channels[2] = (((uart_data[3] >> 6 | uart_data[4] << 2 | uart_data[5] << 10) & 0x07FF) << 1);
        channels[3] = (((uart_data[5] >> 1 | uart_data[6] << 7) & 0x07FF) << 1);
        channels[4] = (((uart_data[6] >> 4 | uart_data[7] << 4) & 0x07FF) << 1);
        channels[5] = (((uart_data[7] >> 7 | uart_data[8] << 1 | uart_data[9] << 9) & 0x07FF) << 1);
        channels[6] = (((uart_data[9] >> 2 | uart_data[10] << 6) & 0x07FF) << 1);
        channels[7] = (((uart_data[10] >> 5 | uart_data[11] << 3) & 0x07FF) << 1);
        channels[8] = (((uart_data[12] | uart_data[13] << 8) & 0x07FF) << 1);
        channels[9] = (((uart_data[13] >> 3 | uart_data[14] << 5) & 0x07FF) << 1);
        channels[10] = (((uart_data[14] >> 6 | uart_data[15] << 2 | uart_data[16] << 10) & 0x07FF) << 1);
        channels[11] = (((uart_data[16] >> 1 | uart_data[17] << 7) & 0x07FF) << 1);
        channels[12] = (((uart_data[17] >> 4 | uart_data[18] << 4) & 0x07FF) << 1);
        channels[13] = (((uart_data[18] >> 7 | uart_data[19] << 1 | uart_data[20] << 9) & 0x07FF) << 1);
        channels[14] = (((uart_data[20] >> 2 | uart_data[21] << 6) & 0x07FF) << 1);
        channels[15] = (((uart_data[21] >> 5 | uart_data[22] << 3) & 0x07FF) << 1);

        for( i = 0; i < 12; i++ )
        {
            if ( rc_rev[i] == 1 )
            {
                channels[i] = 4095 - channels[i];
            }
        }

        RC_RECEIVED = 1;
    }
    else
    {
        // if TX is started first the first interrupt read could hit
        // any point within the s-bus frame
        // delay start of next read until we hit the gap between s-bus frames
        // and get in sync
        // HAL_Delay do not work here
        for (i = 0; i < 5000; i++) // ~3.2ms
        {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        }
        RC_ERROR++;

    }
    HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 25);
}

/**
 * @brief  UART error callbacks.
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_ErrorCallback_SBUS(UART_HandleTypeDef *huart)
{
// should never happen
    HAL_UART_ERROR = huart->ErrorCode;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

// restart receiving
    HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 25);
}

/**
 * @brief  UART error callbacks.
 * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_ErrorCallback_SRXL(UART_HandleTypeDef *huart)
{
// should never happen
    HAL_UART_ERROR = huart->ErrorCode;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

// restart receiving
    HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 35);
}

