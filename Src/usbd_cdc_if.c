/**
 ******************************************************************************
 * @file           : usbd_cdc_if.c
 * @brief          :
 ******************************************************************************
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_CDC 
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_CDC_Private_TypesDefinitions
 * @{
 */
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */
/**
 * @}
 */

/** @defgroup USBD_CDC_Private_Defines
 * @{
 */
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  64
// in header now
//#define APP_RX_DATA_SIZE  16
//#define APP_TX_DATA_SIZE  4
/* USER CODE END PRIVATE_DEFINES */
/**
 * @}
 */

/** @defgroup USBD_CDC_Private_Macros
 * @{
 */
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_CDC_Private_Variables
 * @{
 */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
//uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
/* USER CODE BEGIN PRIVATE_VARIABLES */

USBD_CDC_LineCodingTypeDef LineCoding =
{ 115200, /* baud rate*/
0x00, /* stop bits-1*/
0x00, /* parity - none*/
0x08 /* nb. of bits 8*/
};

//USBD_HandleTypeDef *hUsbDevice_0;

/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Exported_Variables
 * @{
 */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

volatile uint8_t cdc_received = 0;
uint8_t received_data[1024];
volatile uint16_t cdc_received_tot = 0;

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_Private_FunctionPrototypes
 * @{
 */
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{ CDC_Init_FS, CDC_DeInit_FS, CDC_Control_FS, CDC_Receive_FS };

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  CDC_Init_FS
 *         Initializes the CDC media low layer over the FS USB IP
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Init_FS(void)
{
    /* USER CODE BEGIN 3 */
  //  hUsbDevice_0 = &hUsbDeviceFS;

    /* Set Application Buffers */
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    return (USBD_OK);
    /* USER CODE END 3 */
}

/**
 * @brief  CDC_DeInit_FS
 *         DeInitializes the CDC media low layer
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit_FS(void)
{
    /* USER CODE BEGIN 4 */
    return (USBD_OK);
    /* USER CODE END 4 */
}

/**
 * @brief  CDC_Control_FS
 *         Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    /* USER CODE BEGIN 5 */
    switch (cmd)
    {
    case CDC_SEND_ENCAPSULATED_COMMAND:

        break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

        break;

    case CDC_SET_COMM_FEATURE:

        break;

    case CDC_GET_COMM_FEATURE:

        break;

    case CDC_CLEAR_COMM_FEATURE:

        break;

        /*******************************************************************************/
        /* Line Coding Structure                                                       */
        /*-----------------------------------------------------------------------------*/
        /* Offset | Field       | Size | Value  | Description                          */
        /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
        /* 4      | bCharFormat |   1  | Number | Stop bits                            */
        /*                                        0 - 1 Stop bit                       */
        /*                                        1 - 1.5 Stop bits                    */
        /*                                        2 - 2 Stop bits                      */
        /* 5      | bParityType |  1   | Number | Parity                               */
        /*                                        0 - None                             */
        /*                                        1 - Odd                              */
        /*                                        2 - Even                             */
        /*                                        3 - Mark                             */
        /*                                        4 - Space                            */
        /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
        /*******************************************************************************/
    case CDC_SET_LINE_CODING:

        memcpy( &LineCoding, pbuf, sizeof(LineCoding) );

        /*
        LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
        LineCoding.format     = pbuf[4];
        LineCoding.paritytype = pbuf[5];
        LineCoding.datatype   = pbuf[6];
        */

        break;

    case CDC_GET_LINE_CODING:

        memcpy( pbuf, &LineCoding, sizeof(LineCoding) );

        /*
        pbuf[0] = (uint8_t) (LineCoding.bitrate);
        pbuf[1] = (uint8_t) (LineCoding.bitrate >> 8);
        pbuf[2] = (uint8_t) (LineCoding.bitrate >> 16);
        pbuf[3] = (uint8_t) (LineCoding.bitrate >> 24);
        pbuf[4] = LineCoding.format;
        pbuf[5] = LineCoding.paritytype;
        pbuf[6] = LineCoding.datatype;
        */

        break;

    case CDC_SET_CONTROL_LINE_STATE:

        break;

    case CDC_SEND_BREAK:

        break;

    default:
        break;
    }

    return (USBD_OK);
    /* USER CODE END 5 */
}

/**
 * @brief  CDC_Receive_FS
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         untill exiting this function. If you exit this function before transfer
 *         is complete on CDC interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
    /* USER CODE BEGIN 6 */
    // store string in UserRxBufferFS at next received_data index
    if ( cdc_received_tot + *Len > 1024 )
    {
        cdc_received_tot = 0;
    }
    memcpy(&received_data[cdc_received_tot], Buf, *Len);

    cdc_received_tot += *Len;
    cdc_received = 1;

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);

    return (USBD_OK);

    /* USER CODE END 6 */
}

/**
 * @brief  CDC_Transmit_FS
 *         Data send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be send
 * @param  Len: Number of data to be send (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
    uint8_t result = USBD_OK;

    /* USER CODE BEGIN 7 */
    uint16_t bytes_written = 0;
    uint16_t bytes_to_write;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;

    if (hcdc->TxState != 0)
    {
        return USBD_BUSY;
    }

    //if ( hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hUsbDeviceFS.ep0_state == USBD_EP0_DATA_OUT)
    if ( hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED )
    {
        // No physical connection
        result = USBD_FAIL;
    }
    else
    {

        // send in parts until success or error
        while (bytes_written < Len && result != USBD_FAIL)
        {

            /* commented out because it works without delay with socat and with qt5 serial
            // if faster the data don't get drained safely. Why?
            // delay only if previous chunk was written
            if ( bytes_written > 0 )
            {
                HAL_Delay(20);
            }
            */

            //HAL_Delay(20);

            // size of part is APP_TX_DATA_SIZE or remainder
            bytes_to_write = (Len - bytes_written) > APP_TX_DATA_SIZE ? APP_TX_DATA_SIZE : ( Len - bytes_written );

            memcpy(UserTxBufferFS, &Buf[bytes_written], bytes_to_write );

            USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, bytes_to_write);

            // Check if USB disconnected while retrying
            //if ( hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hUsbDeviceFS.ep0_state == USBD_EP0_DATA_IN )
            //if ( hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hUsbDeviceFS.ep0_state == USBD_EP0_STATUS_IN )
            if ( hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED )
            {
                result = USBD_FAIL;
                break;
            }

            // Try send
            result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
            if (result == USBD_OK)
            {
                bytes_written += bytes_to_write;
            }
            else if (result != USBD_BUSY) // other error
            {
                result = USBD_FAIL;
                break;
            }
        }
    }

    //HAL_PCD_EP_Flush( hUsbDeviceFS.pData, CDC_IN_EP);

    /* USER CODE END 7 */
    return result;
}


/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

