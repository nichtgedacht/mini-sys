/**
  ******************************************************************************
  * @file    stm32f1xx_minisys.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-December-2015
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32F1XX-MINISYS Kit
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
#include "../STM32F1xx_MiniSys/stm32f1xx_minisys.h"
#include "spi.h"

/** @addtogroup BSP
  * @{
  */ 

/** @defgroup STM32F1XX_MINISYS STM32F103RB-MINISYS
  * @brief This file provides LCD and uSD functions to communicate with
  *        Adafruit 1.8" TFT LCD shield (reference ID 802) or similar.
  * @{
  */ 


/** @defgroup STM32F1XX_MINISYS_Private_Defines Private Defines
  * @{
  */

/**
  * @brief LINK SD Card
  */
#define SD_DUMMY_BYTE            0xFF
#define SD_NO_RESPONSE_EXPECTED  0x80

/**
  * @}
  */ 
  

/** @defgroup STM32F1XX_MINISYS_Private_Variables Private Variables
  * @{
  */ 

/**
 * @brief BUS variables
 */

#ifdef HAL_SPI_MODULE_ENABLED
uint32_t SpixTimeout = MINISYS_SPIx_TIMEOUT_MAX;        /*<! Value of Timeout when SPI communication fails */
// using hspi2 from CubeMX generated spi.c/h now
//static SPI_HandleTypeDef hMINISYS_Spi;
#endif /* HAL_SPI_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32F1XX_MINISYS_Private_Functions Private Functions
  * @{
  */ 
#ifdef HAL_SPI_MODULE_ENABLED
static void               SPIx_Write(uint8_t Value);
static void               SPIx_Error (void);
/* new functions */
static void               SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
/* MPU IO functions */
void                      MPU_IO_Init(void);
uint8_t                   MPU_IO_WriteReadReg(uint8_t MPUReg, uint8_t arg);
uint8_t                   MPU_IO_WriteByte(uint8_t Data);
void                      MPU_IO_CSState(uint8_t val);

/* SD IO functions */
void                      SD_IO_Init(void);
//HAL_StatusTypeDef         SD_IO_WriteCmd(uint8_t Cmd, uint32_t Arg, uint8_t Crc, uint8_t Response);
//HAL_StatusTypeDef         SD_IO_WaitResponse(uint8_t Response);
void                      SD_IO_WriteDummy(void);
//void                      SD_IO_WriteByte(uint8_t Data);
/* exchanged function from F4 firmware */
uint8_t                   SD_IO_WriteByte(uint8_t Data);
//uint8_t                   SD_IO_ReadByte(void);
/* new functions */
void                      SD_IO_CSState(uint8_t state);
void                      SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);

/* LCD IO functions */
void                      LCD_IO_Init(void);
void                      LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void                      LCD_IO_WriteColorData(uint16_t *pData, uint32_t Size);
void                      LCD_IO_WriteReg(uint8_t LCDReg);
void                      LCD_Delay(uint32_t delay);
#endif /* HAL_SPI_MODULE_ENABLED */
/**
  * @}
  */ 

/** @defgroup STM32F1XX_MINISYS_Exported_Functions Exported Functions
  * @{
  */ 


/** @addtogroup STM32F1XX_MINISYS_Private_Functions
  * @{
  */ 
  
#ifdef HAL_SPI_MODULE_ENABLED
/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/**
  * @brief  SPI Write a byte to device
  * @param  DataIn: value to be written
  * @param  DataOut: value to be read
  * @param  DataLegnth: length of data
  */
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) DataIn, DataOut, DataLegnth, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}
/* end new functions */

/**
  * @brief  SPI Write a byte to device
  * @param  Value: value to be written
  * @retval None
  */
static void SPIx_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&hspi2, (uint8_t*) &Value, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI error treatment function
  * @retval None
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hspi2);

  /* Re-Initiaize the SPI communication BUS */
  // SPIx_Init();
  MX_SPI2_Init();
}

/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/**************************** LINK MPU ****************************************/

/**
  * @brief Set the MPU_CS pin.
  * @param val: pin value.
  */
void MPU_IO_CSState(uint8_t val)
{
  if(val == 1)
  {
    MPU_CS_HIGH();
  }
  else
  {
    MPU_CS_LOW();
  }
}

/**
  * @brief  Initializes the MPU
  * @retval None
  */
void MPU_IO_Init(void)
{
	  GPIO_InitTypeDef  gpioinitstruct = {0};

	  /* MPU_CS_GPIO Periph clock enable */
	  MPU_CS_GPIO_CLK_ENABLE();

	  /* Configure MPU_CS_PIN pin */
	  gpioinitstruct.Pin    = MPU_CS_PIN;
	  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
	  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(MPU_CS_GPIO_PORT, &gpioinitstruct);

	  MPU_CS_HIGH();
}

/**
  * @brief  Writes command and arg to MPU.
  * @param  MPUReg: Address of the selected register.
  * @param  arg: setting value
  * @retval None
  */
uint8_t MPU_IO_WriteReadReg(uint8_t MPUReg, uint8_t arg)
{

  uint8_t resp;

  /* Reset LCD control line CS */
  MPU_CS_LOW();

  /* Send Command */
  SPIx_Write(MPUReg);

  /* Send the byte */
  SPIx_WriteReadData(&arg,&resp,1);

  /* Deselect : Chip Select high */
  MPU_CS_HIGH();

  return resp;

}

uint8_t MPU_IO_WriteByte(uint8_t Data)
{
  uint8_t tmp;

  /* Send the byte */
  SPIx_WriteReadData(&Data,&tmp,1);

  return tmp;
}


/********************************* LINK SD ************************************/
/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for
  *         data transfer).
  * @retval None
  */
void SD_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  uint8_t counter = 0;

  /* SD_CS_GPIO Periph clock enable */
  SD_CS_GPIO_CLK_ENABLE();

  /* Configure SD_CS_PIN pin: SD Card CS pin */
  gpioinitstruct.Pin    = SD_CS_PIN;
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull   = GPIO_PULLUP;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_PORT, &gpioinitstruct);

  /*------------Put SD in SPI mode--------------*/
  /* SD SPI Config */
  /* already done by MX */
  //SPIx_Init();

  /* SD chip select high */
  SD_CS_HIGH();

  /* Send dummy byte 0xFF, 10 times with CS high */
  /* Rise CS and MOSI for 80 clocks cycles */
  for (counter = 0; counter <= 9; counter++)
  {
    /* Send dummy byte 0xFF */
    SD_IO_WriteByte(SD_DUMMY_BYTE);
  }
}

/* exchanged function from F4 firmaware */
/**
  * @brief  Writes a byte on the SD.
  * @param  Data: byte to send.
  */
uint8_t SD_IO_WriteByte(uint8_t Data)
{
  uint8_t tmp;
  /* Send the byte */
  SPIx_WriteReadData(&Data,&tmp,1);
  return tmp;
}

/**
  * @brief Set the SD_CS pin.
  * @param val: pin value.
  */
void SD_IO_CSState(uint8_t val)
{
  if(val == 1)
  {
    SD_CS_HIGH();
  }
  else
  {
    SD_CS_LOW();
  }
}

/* end new functions */

/**
  * @brief Write a byte on the SD.
  * @param  DataIn: value to be written
  * @param  DataOut: value to be read
  * @param  DataLength: length of data
  */
void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  /* Send the byte */
  SPIx_WriteReadData(DataIn, DataOut, DataLength);
}

/**
  * @brief  Sends dummy byte with CS High
  * @retval None
  */
void SD_IO_WriteDummy(void)
{
  /* SD chip select high */
  SD_CS_HIGH();

  /* Send Dummy byte 0xFF */
  SD_IO_WriteByte(SD_DUMMY_BYTE);
}

/********************************* LINK LCD ***********************************/
/**
  * @brief  Initializes the LCD
  * @retval None
  */
void LCD_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};

  /* LCD_CS_GPIO and LCD_DC_GPIO Periph clock enable */
  LCD_CS_GPIO_CLK_ENABLE();
  LCD_DC_GPIO_CLK_ENABLE();
  LCD_RST_GPIO_CLK_ENABLE();
  
  /* Configure LCD_CS_PIN pin */
  gpioinitstruct.Pin    = LCD_CS_PIN;
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
      
  /* Configure LCD_DC_PIN pin */
  gpioinitstruct.Pin    = LCD_DC_PIN;
  HAL_GPIO_Init(LCD_DC_GPIO_PORT, &gpioinitstruct);

  /* Configure LCD Card RST pin */
  gpioinitstruct.Pin    = LCD_RST_PIN;
  HAL_GPIO_Init(LCD_RST_GPIO_PORT, &gpioinitstruct);

  /* Reset LCD */
  LCD_RST_HIGH();
  HAL_Delay(500);
  LCD_RST_LOW();
  HAL_Delay(500);
  LCD_RST_HIGH();
//  HAL_Delay(50);

  /* LCD chip select high */
  LCD_CS_HIGH();
  
  HAL_Delay(5);

  /* LCD SPI Config */
  /* already done by MX */
  //SPIx_Init();
}

/**
  * @brief  Writes command to select the LCD register.
  * @param  LCDReg: Address of the selected register.
  * @retval None
  */
void LCD_IO_WriteReg(uint8_t LCDReg)
{
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to Low */
  LCD_DC_LOW();
    
  /* Send Command */
  SPIx_Write(LCDReg);
  
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
* @brief  Write register value.
* @param  pData Pointer on the register value
* @param  Size Size of byte to transmit to the register
* @retval None
*/
void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size)
{
  uint32_t counter = 0;
  
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  if (Size == 1)
  {
    /* Only 1 byte to be sent to LCD - general interface can be used */
    /* Send Data */
    SPIx_Write(*pData);
  }
  else
  {
    /* Several data should be sent in a raw */
    /* Direct SPI accesses for optimization */
    for (counter = Size; counter != 0; counter--)
    {

      while(((hspi2.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }

      /* Need to invert bytes for LCD*/
      *((__IO uint8_t*)&hspi2.Instance->DR) = *(pData+1);

      while(((hspi2.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }

      *((__IO uint8_t*)&hspi2.Instance->DR) = *pData;
      counter--;
      pData += 2;
    }
  
    /* Wait until the bus is ready before releasing Chip select */ 
    while(((hspi2.Instance->SR) & SPI_FLAG_BSY) != RESET)
    {
    }

  } 
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
* @brief  Write color data to LCD memory.
* @param  pData Pointer on the color data word
* @param  Size Number of pxels to transmit
* @retval None
*/
void LCD_IO_WriteColorData(uint16_t *pData, uint32_t Size)
{
  uint32_t counter = 0;

  /* Reset LCD control line CS */
  LCD_CS_LOW();

  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();


  /* Several data should be sent in a raw */
  /* Direct SPI accesses for optimization */
    for (counter = Size; counter != 0; counter--)
    {

     while(((hspi2.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
     {
     }

    /* Need to invert bytes for LCD*/
     *((__IO uint8_t*)&hspi2.Instance->DR) = *pData >> 8;

    while(((hspi2.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
     {
     }

    *((__IO uint8_t*)&hspi2.Instance->DR) = *pData;

   }

  /* Wait until the bus is ready before releasing Chip select */
   while(((hspi2.Instance->SR) & SPI_FLAG_BSY) != RESET)
   {
   }

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}


/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

#endif /* HAL_SPI_MODULE_ENABLED */

  
/**
  * @}
  */    

/**
  * @}
  */ 
    
/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
