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
#include "servo.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t counter = 0;
uint32_t failsafe_counter = 0;
char* pDirectoryFiles[MAX_BMP_FILES];
uint8_t res;
FRESULT fres;
DIR directory;
FATFS SD_FatFs; /* File system object for SD card logical drive */
UINT BytesWritten, BytesRead;
uint32_t size = 0;
uint32_t nbline;
uint8_t *ptr = NULL;
float volt1 = 0.0f, volt2 = 0.0f;
uint32_t free_ram;
uint8_t red = 1;
uint32_t width;
uint32_t height;
char buf[50] =
{ 0 };
uint16_t color = LCD_COLOR_WHITE;
const uint8_t flash_top = 255;
uint32_t free_flash;
uint8_t whoami;
uint8_t mpu_res;
uint32_t tick, prev_tick, dt, dtx;
float roll, pitch, yaw;
float bla;
int xp, yp;
float vx = 0.0f, vy = 0.0f;
HAL_StatusTypeDef hal_res;

float diffroll;
float diffnick;
float diffgier;

int16_t thrust_set = 0;
int16_t roll_set = 0;
int16_t nick_set = 0;
int16_t gier_set = 0;

//int32_t i_diffroll;

float last_derivative[3];
float last_error[3];
float integrator[3];

const float RKp = 0.24f;
const float RKi = 1.5f;
const float RKd = 0.002f;
//const float RKd = 0.0070f;
//const float RKd = 0.00144f;

const float NKp = 0.24f;
const float NKi = 1.5f;
const float NKd = 0.002f;
//const float NKd = 0.0070f;
//const float NKd = 0.00144f;

const float GKp = 1.5f;
const float GKi = 1.5f;
const float GKd = 0.001f;

const float RC = 0.007958;  // 1/(2*PI*_fCut fcut = 20 from Ardupilot

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int16_t pid(uint8_t axis, float error, float Kp, float Ki, float Kd, float dt);
void control(int16_t thrust_set, int16_t roll_set, int16_t nick_set, int16_t gier_set);

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
    MX_USB_DEVICE_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */

    MX_FATFS_Init();
    MX_SPI2_Init();

    // Request first 25 bytes s-bus frame from uart, uart_data becomes filled per interrupts
    // Get callback if ready Callback restarts request
    HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 25);

    //############### MPU Test init ########################################
    // no sample rate divider, accel: lowpass filter bandwidth 460 Hz, Rate 1kHz, gyro:  lowpass filter bandwidth 250 Hz
    BSP_MPU_Init(0, 2, 0);
    HAL_Delay(2000);
    BSP_MPU_GyroCalibration();
    //############ end MPU Test init #######################################

    BSP_LCD_Init();
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    BSP_LCD_SetFont(&Font20);
    BSP_LCD_SetRotation(3);
    color = LCD_COLOR_WHITE;

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);

    //not to be enabled until BSP_MPU_GyroCalibration
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

    //for moving circle by gravity start position
    xp = BSP_LCD_GetXSize() / 2 - 1;
    yp = BSP_LCD_GetYSize() / 2 + 1;

    // enable USB on maple mine clone or use reset as default state
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    //############ init SD-card, signal errors by LED ######################
    res = BSP_SD_Init();

    if (res != BSP_SD_OK)
    {
        Error_Handler();
    }
    else
    {
        fres = f_mount(&SD_FatFs, (TCHAR const*) "/", 0);
        sprintf(buf, "f_mount: %d", fres);
    }

    if (fres != FR_OK)
    {
        Error_Handler();
    }
    else
    {
        for (counter = 0; counter < MAX_BMP_FILES; counter++)
        {
            pDirectoryFiles[counter] = malloc(11);
        }
    }

    //############ end init SD-card, signal errors by LED ##################

    /*
     res = BSP_SD_Init();
     if ( res == BSP_SD_OK )
     {
     TFT_DisplayImages(0, 0, "PICT2.BMP", buf);
     }
     */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        /*
         //############ s-bus test ##########################################
         //########### set rotation to 0 or 2 above #########################
         BSP_LCD_SetRotation(0);
         BSP_LCD_SetFont(&Font12);

         // show channels 1-12
         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 0 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_1: %d", channels[0]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(0, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 1 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_2: %d", channels[1]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(1, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 2 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_3: %d", channels[2]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(2, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 3 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_4: %d", channels[3]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(3, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 4 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_5: %d", channels[4]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(4, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 5 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_6: %d", channels[5]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(5, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 6 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_7: %d", channels[6]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(6, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 7 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_8: %d", channels[7]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(7, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 8 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_9: %d", channels[8]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(8, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 9 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_10: %d", channels[9]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(9, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 10 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_11: %d", channels[10]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(10, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 11 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ch_12: %d", channels[11]);
         BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
         BSP_LCD_DisplayStringAtLine(11, (uint8_t *) buf);

         BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
         BSP_LCD_FillRect(0, 12 * 12, BSP_LCD_GetXSize() - 50, 12);
         sprintf(buf, "ERROR: %d", SBUS_ERROR);
         BSP_LCD_SetTextColor(LCD_COLOR_RED);
         BSP_LCD_DisplayStringAtLine(12, (uint8_t *) buf);

         //############ end s-bus test ######################################
         */

        //############### MPU Test #########################################
        if (PeriodElapsed == 1) // back to 200 Hz otherwise water bubble is to slow to get around
        {
            PeriodElapsed = 0;
            counter++;
            failsafe_counter++;

            if (SBUS_RECEIVED == 1)
            {
                SBUS_RECEIVED = 0;
                failsafe_counter = 0;
            }

            BSP_MPU_read_rot();
            BSP_MPU_read_acc();

            if (channels[4] < 1200 || failsafe_counter > 40)
            {
                servos[0] = 2000;
                servos[1] = 2000;
                servos[2] = 2000;
                servos[3] = 2000;
                last_derivative[x] = 0.0f;
                last_derivative[y] = 0.0f;
                last_derivative[z] = 0.0f;
                last_error[x] = 0.0f;
                last_error[y] = 0.0f;
                last_error[z] = 0.0f;
                integrator[x] = 0.0f;
                integrator[y] = 0.0f;
                integrator[z] = 0.0f;
            }
            else
            {
                // just attitude hold mode
                diffroll = gy[x] * 4.0f - (float) channels[1] + 1000.0f;
                diffnick = gy[y] * 4.0f - (float) channels[2] + 1000.0f;
                diffgier = gy[z] * 4.0f + (float) channels[3] - 1000.0f; // control reversed, gy right direction

                thrust_set = (int16_t) channels[0] + 2000;
                roll_set = pid(x, diffroll, RKp, RKi, RKd, 5.0f);
                nick_set = pid(y, diffnick, NKp, NKi, NKd, 5.0f);
                gier_set = pid(z, diffgier, GKp, GKi, GKd, 5.0f);

                control(thrust_set, roll_set, nick_set, gier_set);
                // assured finished before first servo update by HAL_TIM_PWM_PulseFinishedCallback
            }

            tick = HAL_GetTick();
            dt = tick - prev_tick;
            prev_tick = tick;

            BSP_MPU_updateIMU(ac[x], ac[y], ac[z], gy[x], gy[y], gy[z], 5.0f);
            BSP_MPU_getEuler(&roll, &pitch, &yaw);

            //free_ram = (0x20000000 + 1024 * 20) - (uint32_t) sbrk((int)0);
            //sprintf(buf, "free: %ld\n", free_ram);

            //sprintf(buf, "dt: %ld\n", dt);
            //sprintf(buf, "%3.3f,%3.3f,%3.3f\n", yaw, pitch, roll);
            //sprintf(buf, "%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f\n", ac[x], ac[y], ac[z], gy[x], gy[y], gy[z]);

            //########### water bubble #########################################

            if (counter > 5)
            {
                counter = 0;

                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                BSP_LCD_DrawCircle(xp, yp, 5);

                BSP_LCD_SetTextColor(LCD_COLOR_RED);
                BSP_LCD_DrawHLine(75, 64, 10);
                BSP_LCD_DrawVLine(80, 59, 10);

                vx = sinf(pitch) * 300.0f;
                vy = sinf(roll) * 300.0f;

                xp = roundf(vx) + 80;
                yp = roundf(vy) + 64;

                if (xp < 5)
                {
                    xp = 5;
                    vx = 0;
                }
                if (yp < 5)
                {
                    yp = 5;
                    vy = 0;
                }
                if (yp > 122)
                {
                    yp = 122;
                    vy = 0;
                }
                if (xp > 154)
                {
                    xp = 154;
                    vx = 0;
                }

                BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
                BSP_LCD_DrawCircle(xp, yp, 5);

            }

            //############ end water bubble ####################################

            HAL_ADCEx_Calibration_Start(&hadc1);
            if (HAL_ADC_Start(&hadc1) == HAL_OK)
            {
                if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
                {
                    volt1 = (HAL_ADC_GetValue(&hadc1)) / 219.84f; // calibrate
                }

                HAL_ADC_Stop(&hadc1);
            }

            if (volt1 < 10.0f || channels[6] > 1000) // beeper
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
            }

            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

            if (HAL_UART_ERROR != 0)
            {
                HAL_UART_ERROR = 0;
            }

            //CDC_Transmit_FS((uint8_t*) buf, strlen(buf));

        }

    } //while(1)

    /* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

int16_t pid(uint8_t axis, float error, float Kp, float Ki, float Kd, float dt)
{
    float derivative = 0;
    float output_f = 0;
    int16_t output = 0;

    dt /= 1000.0f;

// P
    output_f += error * Kp;

    integrator[axis] += (error * Ki) * dt;
    if (integrator[axis] < -400)
    {
        integrator[axis] = -400;
    }
    else if (integrator[axis] > 400)
    {
        integrator[axis] = 400;
    }

// I
    output_f += integrator[axis];

    derivative = (error - last_error[axis]) / dt;
// DT1
    derivative = last_derivative[axis] + ((dt / (RC + dt)) * (derivative - last_derivative[axis]));

    last_error[axis] = error;
    last_derivative[axis] = derivative;

// DT1
    output_f += Kd * derivative;

    output = roundf(output_f);

    return output;
}

void control(int16_t thrust_set, int16_t roll_set, int16_t nick_set, int16_t gier_set)
{
    servos[3] = thrust_set - roll_set + nick_set + gier_set;  // Motor front left  CCW
    servos[1] = thrust_set + roll_set + nick_set - gier_set;  // Motor front right CW
    servos[0] = thrust_set - roll_set - nick_set - gier_set;  // Motor rear left   CW
    servos[2] = thrust_set + roll_set - nick_set + gier_set;  // Motor rear right  CCW

    /*
     Mapping:

     normal:
     thrust  roll-right  nick-down  gier-right (gier control reversed in software)
     servos[0] Motor front left  CCW     +         +          -           -
     servos[1] Motor front right CW      +         -          -           +
     servos[2] Motor rear left   CW      +         +          +           -
     servos[3] Motor rear right  CCW     +         -          +           +

     connected:
     servos[0] Motor rear left   CW
     servos[2] Motor rear right  CCW
     servos[1] Motor front right CW
     servos[3] Motor front left  CCW
     */

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
    uint8_t counter;
    for (counter = 0; counter < 6; counter++)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
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
