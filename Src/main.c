/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "fatfs.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "stm32_adafruit_lcd.h"
#include "stm32_adafruit_sd.h"
#include "usbd_cdc_if.h"
#include "fatfs_storage.h"
#include "unistd.h"
#include "mpu9250.h"
#include "math.h"
#include "sbus.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t counter = 0;
char* pDirectoryFiles[MAX_BMP_FILES];
uint8_t res;
FRESULT fres;
DIR directory;
FATFS SD_FatFs;  /* File system object for SD card logical drive */
UINT BytesWritten, BytesRead;
uint8_t str[20];
uint32_t size = 0;
uint32_t nbline;
uint8_t *ptr = NULL;
float volt1, volt2;
uint32_t free_ram;
uint8_t red=1;
uint32_t width;
uint32_t height;
char buf[20]={0};
uint16_t color=LCD_COLOR_WHITE;
const uint8_t flash_top=255;
uint32_t free_flash;
uint8_t whoami;
uint8_t mpu_res;
uint32_t tick, prev_tick, dt, dtx;
float roll, pitch, yaw;
float bla;
int xp, yp;
float vx=0.0f, vy=0.0f;
HAL_StatusTypeDef hal_res;
//uint32_t alloc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static uint8_t TFT_DisplayImages(uint8_t x, uint16_t y, const char* fname);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  MX_FATFS_Init();
  MX_SPI2_Init();

  //############### MPU Test init ########################################
  BSP_MPU_Init(99, 0x06, 0x06);
  BSP_MPU_GyroCalibration();
  //############ end MPU Test init #######################################

  BSP_LCD_Init();
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_SetFont(&Font20);
  BSP_LCD_SetRotation(0);
  color=LCD_COLOR_WHITE;

  //for moving circle by gravity start position
  xp = BSP_LCD_GetXSize()/2;
  yp = BSP_LCD_GetYSize()/2;

  // enable USB on maple mine clone or use reset as default state
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  //############ init SD-card, signal errors by LED ######################
  res = BSP_SD_Init();

  if ( res != BSP_SD_OK)
  {
      for (counter = 0; counter < 6; counter++)
          {
              HAL_GPIO_TogglePin(GPIOB,  GPIO_PIN_1);
              HAL_GPIO_TogglePin(GPIOC,  GPIO_PIN_13);
              HAL_Delay(100);
          }
  }
  else
  {
      fres = f_mount(&SD_FatFs, (TCHAR const*)"/", 0);
          sprintf(buf, "f_mount: %d", fres);
  }

  if ( fres != FR_OK)
  {
      for (counter = 0; counter < 6; counter++)
          {
              HAL_GPIO_TogglePin(GPIOB,  GPIO_PIN_1);
              HAL_GPIO_TogglePin(GPIOC,  GPIO_PIN_13);
              HAL_Delay(100);
          }
  }
  else
  {
          for (counter = 0; counter < MAX_BMP_FILES; counter++)
          {
              pDirectoryFiles[counter] = malloc(11);
          }

  }
  //############ end init SD-card, signal errors by LED ##################


  res = BSP_SD_Init();
  if ( res == BSP_SD_OK )
  {
      TFT_DisplayImages(0, 0, "PICT1.BMP");
  }

  // Request 25 bytes frame from uart, uart_data becomes filled per interrupts
  // Get callback if ready
  HAL_UART_Receive_IT(&huart1, (uint8_t*)uart_data, 25);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

      HAL_GPIO_TogglePin(GPIOB,  GPIO_PIN_1);
      HAL_GPIO_TogglePin(GPIOC,  GPIO_PIN_13);


      //################### ADC test #####################################

      HAL_ADCEx_Calibration_Start(&hadc1);
      if ( HAL_ADC_Start(&hadc1) == HAL_OK )
      {
          if ( HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK )
          {
              volt1 =  (3.3 * HAL_ADC_GetValue(&hadc1)) / 4090; // calibrate
          }

          HAL_ADC_Stop(&hadc1);
      }

      HAL_ADCEx_Calibration_Start(&hadc2);
      if ( HAL_ADC_Start(&hadc2) == HAL_OK )
      {
          if ( HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK )
          {
              volt2 =  (3.3 * HAL_ADC_GetValue(&hadc2)) / 4090;  // calibrate
          }

          HAL_ADC_Stop(&hadc2);
      }

      //###################### end ADC test ##############################


      //############ s-bus test ##########################################
      //########### set rotation to 0 or 2 above #########################
      if ( SBUS_RECEIVED == 1 )
      {

          SBUS_RECEIVED = 0;

      	  // show channels 1-12
      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 0 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_1: %d", channels[0]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(0, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 1 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_2: %d", channels[1]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(1, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 2 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_3: %d", channels[2]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(2, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 3 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_4: %d", channels[3]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(3, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 4 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_5: %d", channels[4]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(4, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 5 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_6: %d", channels[5]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(5, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 6 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_7: %d", channels[6]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(6, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 7 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_8: %d", channels[7]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(7, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 8 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_9: %d", channels[8]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(8, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 9 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_10: %d", channels[9]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(9, (uint8_t *)buf);

      	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      	  BSP_LCD_FillRect(0, 10 * 12, BSP_LCD_GetXSize() - 50, 12);
      	  sprintf(buf, "ch_11: %d", channels[10]);
      	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      	  BSP_LCD_DisplayStringAtLine(10, (uint8_t *)buf);

   	      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
   	      BSP_LCD_FillRect(0, 11 * 12, BSP_LCD_GetXSize() - 50, 12);
   		  sprintf(buf, "ch_12: %d", channels[11]);
   		  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
   		  BSP_LCD_DisplayStringAtLine(11, (uint8_t *)buf);

   	      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
   	      BSP_LCD_FillRect(0, 12 * 12, BSP_LCD_GetXSize() - 50, 12);
   	  	  sprintf(buf, "ERROR: %d", HAL_UART_ERROR );
   	  	  BSP_LCD_SetTextColor(LCD_COLOR_RED);
   	  	  BSP_LCD_DisplayStringAtLine(12, (uint8_t *)buf);

   	  }

      if ( HAL_UART_ERROR != 0 )
      {
          HAL_UART_ERROR = 0;
      }

     // HAL_Delay(10);

      //############ end s-bus test ######################################



      //############### MPU Test #########################################

      BSP_LCD_SetFont(&Font12);
      BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

      BSP_MPU_read_rot();
      BSP_MPU_read_acc();

      tick = HAL_GetTick();
      dt = tick - prev_tick;
      prev_tick = tick;

      BSP_MPU_updateIMU(ac[x], ac[y], ac[z], gy[x], gy[y], gy[z], dt);

      BSP_MPU_getEuler(&roll, &pitch, &yaw);

/*
      //########### moving circle by gravity #############################
      //########### set rotation above according to IMU orientation ######

	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_DrawCircle(xp, yp , 5);

      BSP_LCD_SetTextColor(LCD_COLOR_RED);
      BSP_LCD_DrawRect(74, 58, 13, 13);

	  vx += sinf(pitch)*10;
	  vy += sinf(roll)*10;
	  xp += roundf(vx);
	  yp += roundf(vy);

	  if ( xp < 5 ) {
		  xp = 5;
		  vx = 0;
	  }
	  if ( yp < 5 ) {
		  yp = 5;
		  vy = 0;
	  }
	  if ( yp > 122) {
		  yp = 122;
		  vy = 0;
	  }
	  if ( xp > 154) {
		  xp = 154;
		  vx = 0;
	  }

	  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	  BSP_LCD_DrawCircle(xp, yp , 5);

      if ( xp == 80 && yp == 64 )
      {

          res = BSP_SD_Init();
          if ( res == BSP_SD_OK )
          {
              TFT_DisplayImages(0, 0, "PICT2.BMP");
          }
      }

	  sprintf(buf, "%3.3f,%3.3f,%3.3f\n", yaw, pitch, roll);
 	  CDC_Transmit_FS((uint8_t*) buf, strlen(buf));

	  HAL_Delay(10);

      //############ end moving circle by gravity ########################
*/

/*
      //############ show Euler angles and quaternion ####################

      roll  *= 180 / M_PI;
	  pitch *= 180 / M_PI;
	  yaw   *= 180 / M_PI;

	  sprintf(buf, "roll: %f", roll);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 4 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(4, (uint8_t *)buf);

      sprintf(buf, "pitch: %f", pitch);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 5 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(5, (uint8_t *)buf);

      sprintf(buf, "yaw: %f", yaw);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 6 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(6, (uint8_t *)buf);


      sprintf(buf, "q0: %f", q0);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 0 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(0, (uint8_t *)buf);

      sprintf(buf, "q1: %f", q1);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 1 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(1, (uint8_t *)buf);

      sprintf(buf, "q2: %f", q2);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 2 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(2, (uint8_t *)buf);

      sprintf(buf, "q3: %f", q3);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 3 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(3, (uint8_t *)buf);

      //############ end show Euler angles and quaternion ################

*/

/*
      //############ show values from sensors ############################

      sprintf(buf, "gx: %f", gy[x]);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 0 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(0, (uint8_t *)buf);

      sprintf(buf, "gy: %f", gy[y]);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 1 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(1, (uint8_t *)buf);

      sprintf(buf, "gz: %f", gy[z]);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 2 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(2, (uint8_t *)buf);

      sprintf(buf, "ax: %f", ac[x]);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 3 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(3, (uint8_t *)buf);

      sprintf(buf, "ay: %f", ac[y]);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 4 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(4, (uint8_t *)buf);

      sprintf(buf, "az: %f", ac[z]);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_FillRect(80, 5 * 12, BSP_LCD_GetXSize() - 80, 12);
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(5, (uint8_t *)buf);

      //############ end show values from sensors ########################

      //############ IMU SPI test ########################################
	  //whoami = BSP_MPU_Whoami();
	  //sprintf(buf, "whoami: %d", whoami);
	  //BSP_LCD_DisplayStringAtLine(0, (uint8_t *)buf);
      //BSP_MPU_ReadRegs(MPUREG_WHOAMI, &mpu_res, 1 );
      //sprintf(buf, "mpu_res: %d", mpu_res);
      //BSP_LCD_DisplayStringAtLine(4, (uint8_t *)buf);
	  //############ end IMU SPI test ####################################

      //############# show loop speed #####################################
	  dtx = dt + 1;
	  if ( dtx > dt)
	  {
	      dtx = dt;
	  }
	  sprintf(buf, "dt: %ld", dtx);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_FillRect(35, 9 * 12, 35, 12);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	  BSP_LCD_DisplayStringAtLine(9, (uint8_t *)buf);
	  //############# end show loop speed #################################


      //#################### end MPU Test ################################
*/


/*
      //##################### Display Test ###############################

      //BSP_LCD_DisplayChar(20, 20, 'H');
      //BSP_LCD_DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Line_ModeTypdef Mode)

      BSP_LCD_DisplayStringAt(8, 8, (uint8_t *) "Hello", LEFT_MODE);
      BSP_LCD_DisplayStringAt(8, 8 + BSP_LCD_GetFontHeight(), (uint8_t *) "World", LEFT_MODE);

      //BSP_LCD_DisplayStringAtLine(0, (uint8_t *) "Hello");
      //BSP_LCD_DisplayStringAtLine(1, (uint8_t *) "World");

      BSP_LCD_DrawRect(7, 7, 72, 42);
      BSP_LCD_DrawCircle(BSP_LCD_GetXSize() - 28, BSP_LCD_GetYSize() - 28, 21);
      BSP_LCD_DrawRect(BSP_LCD_GetXSize() - 49, BSP_LCD_GetYSize() - 49, 43, 43);

      BSP_LCD_DrawRect(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
      BSP_LCD_DrawRect(1, 1, BSP_LCD_GetXSize()-2, BSP_LCD_GetYSize()-2);

      BSP_LCD_SetTextColor(color);
      BSP_LCD_FillRect( 6, BSP_LCD_GetYSize() / 2 - 9, BSP_LCD_GetXSize() - 12, 19 );
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
      BSP_LCD_SetFont(&Font12);

      //ptr = malloc(sizeof(uint8_t)*12);
      //if ( ptr != NULL)
      //  {
      //      memset(ptr, 0xFF, sizeof(uint8_t)*12);
      //  }
      //alloc = (uint32_t) sbrk((int)0);


      // ######################## free_ram check  ########################
      //free_ram = (0x20000000 + 1024 * 20) - (uint32_t) sbrk((int)0);
      //sprintf(buf, "free: %ld", free_ram);
      // #################################################################
      // ######################## free_flash check #######################
      //free_flash = (0x8000000 + 1024 * 64) - (uint32_t) &flash_top;
      //sprintf(buf, "free: %ld", free_flash);
      // #################################################################

      //displays last sprintf buf value on center bar
      BSP_LCD_SetBackColor(color);
      BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 - 6, (uint8_t *) buf, CENTER_MODE);

      //######### write to /dev/ACM0 (USB) ###############################
	  //UsbDeviceFS comes magically from usbd_cdc_if.h (extern declaration)
	  //this function is not well implemented
	  if ( USBD_LL_DevConnected(&hUsbDeviceFS) == USBD_OK)
	  {
	      sprintf(buf, "U1: %3.3f V\n", volt1);
		  CDC_Transmit_FS((uint8_t*) buf, strlen(buf));

	  }
	  //######### end write to /dev/ACM0 (USB) ###########################

      HAL_Delay(100);
      HAL_GPIO_TogglePin(GPIOB,  GPIO_PIN_1);   // Maple mini clone LED
      HAL_GPIO_TogglePin(GPIOC,  GPIO_PIN_13);  // STM32 minimum system LED
      HAL_Delay(2500);

      if ( red == 1 )
      {
          red = 0;
          sprintf(buf, "V1: %3.3f V", volt1);

          //color = ( ( sizeof(uint16_t) * rand() + 1 ) / sizeof(int));
          color = LCD_COLOR_YELLOW;

          BSP_LCD_SetFont(&Font20);
          BSP_LCD_SetRotation( ( BSP_LCD_GetRotation() + 1 ) % 4  );
          BSP_LCD_SetBackColor(LCD_COLOR_RED);
          //BSP_LCD_Clear(LCD_COLOR_BLACK);

          if ( BSP_LCD_GetRotation() == 0 || BSP_LCD_GetRotation() == 2)
          {
              res = BSP_SD_Init();
              if ( res == BSP_SD_OK )
              {
                  TFT_DisplayImages(0, 0, "PICT1.BMP");

                  //if(f_open(&bmpfile, "testfile", FA_WRITE | FA_CREATE_NEW) == FR_OK)
                  //{
                  //    sprintf(buf, "%s\n", "Hallo Welt" );
                  //    f_write(&bmpfile, buf, strlen(buf), &BytesWritten);
                  //    f_close(&bmpfile);
                  //
                  //}


                  //res = BSP_SD_GetStatus();
                  //sprintf(buf, "SD_Status: %d", res );
              }
          }
          else
          {
              res = BSP_SD_Init();
        	  if ( res == BSP_SD_OK )
        	  {
        	      TFT_DisplayImages(0, 0, "PICT2.BMP");
              }
          }

          BSP_LCD_SetTextColor(LCD_COLOR_RED);
          BSP_LCD_FillCircle(BSP_LCD_GetXSize() - 28, BSP_LCD_GetYSize() - 28, 20);
          BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      }
      else
      {
          red = 1;
          sprintf(buf, "V2: %3.3f V", volt2);

          //color =  ( ( sizeof(uint16_t) * rand() + 1 ) / sizeof(int));
          color = LCD_COLOR_WHITE;

          BSP_LCD_SetFont(&Font20);
          BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
          BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
          //BSP_LCD_Clear(LCD_COLOR_BLUE);

          if ( BSP_LCD_GetRotation() == 0 || BSP_LCD_GetRotation() == 2)
          {
              res = BSP_SD_Init();
        	  if ( res == BSP_SD_OK )
        	  {
        	      TFT_DisplayImages(BSP_LCD_GetXSize()-50, BSP_LCD_GetYSize()-50, "PICT1-P.BMP");
              }
          }
          else
          {
              res = BSP_SD_Init();
        	  if ( res == BSP_SD_OK )
        	  {
        	      TFT_DisplayImages(BSP_LCD_GetXSize()-50, BSP_LCD_GetYSize()-50, "PICT2-P.BMP");
              }
          }
      }

      //###################### end Display Test ##########################

*/

      }

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */


static uint8_t TFT_DisplayImages(uint8_t x, uint16_t y, const char* fname)
{
  uint32_t bmplen = 0x00;
  uint32_t nfiles=0x00;
  uint32_t checkstatus = 0x00;
  DIR directory;
  FRESULT res;

  /* Open directory */
  res = f_opendir(&directory, "/");
  if((res != FR_OK))
  {
    if(res == FR_NO_FILESYSTEM)
    {
      sprintf(buf, "SD_CARD_NOT_FORMATTED" );
      return 1;
    }
    else
    {
      sprintf(buf, "SD_CARD_OPEN_FAIL" );
      return 1;
    }
  }

  /* Get number of bitmap files */
  nfiles = Storage_GetDirectoryBitmapFiles ("/", pDirectoryFiles);

  //sprintf(buf, "nfiles: %ld", number_of_files);

  //sprintf((char*)str, "%-11.11s", pDirectoryFiles[bmpcounter -1]);

  // sprintf((char*)str, "%s", pDirectoryFiles[bmpcounter -1]);

  sprintf((char*)str, "%s", fname);

  checkstatus = Storage_CheckBitmapFile((const char*)str, &bmplen);

  //sprintf(buf, "%ld: %s", bmpcounter, str );

  if(checkstatus == 0)
  {
      checkstatus = Storage_OpenReadFile(x, y, (const char*)str);
  }

  if (checkstatus == 1)
  {
      //sprintf(buf, "SD_CARD_FILE_NOT_SUPPORTED" );
	  return 1;
  }

  return 0;

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
