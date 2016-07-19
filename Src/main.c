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
#include "rc.h"
#include "servo.h"
#include "controller.h"
#include "flash.h"

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
float volt1 = 0.0f;
uint32_t free_ram;
char buf[50] =
{ 0 };
const uint8_t flash_top = 255;
uint32_t free_flash;
uint32_t tick, prev_tick, dt;
float roll, pitch, yaw;
int xp, yp;
float vx = 0.0f, vy = 0.0f;
HAL_StatusTypeDef hal_res;
uint32_t idle_counter;
float cp_pid_vars[9];
uint8_t i;
uint8_t indexer = 0;
uint32_t systick_val1, systick_val2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void draw_program_pid_values(uint8_t line, float value, char* format, uint8_t index, uint8_t offset);

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
    MX_SPI2_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */

    MX_FATFS_Init();

    /*
     // alternative SBUS
     MX_SBUS_USART1_UART_Init();
     UART_RxCpltCallback = &HAL_UART_RxCpltCallback_SBUS;
     UART_ErrorCallback = &HAL_UART_ErrorCallback_SBUS;
     // Request first 25 bytes s-bus frame from uart, uart_data becomes filled per interrupts
     // Get callback if ready. Callback function starts next request
     HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 25);
     */

    // alternative SRXL16 (Multiplex) aka UDI16 (Jeti)
    MX_USART1_UART_Init();
    UART_RxCpltCallback = &HAL_UART_RxCpltCallback_SRXL;
    UART_ErrorCallback = &HAL_UART_ErrorCallback_SRXL;
    // Request first 35 bytes srxl frame from uart, uart_data becomes filled per interrupts
    // Get callback if ready. Callback function starts next request
    HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 35);

    // no sample rate divider (0+1), accel: lowpass filter bandwidth 460 Hz, Rate 1kHz, gyro: lowpass filter bandwidth 250 Hz
    // gyro lpf, 3. parameter:
    // 0  250 Hz
    // 1  184 Hz
    // 2   92 Hz
    // 3   41 Hz
    BSP_MPU_Init(0, 2, 0);
    HAL_Delay(2000);
    BSP_MPU_GyroCalibration();

    BSP_LCD_Init();
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetRotation(0);
    BSP_LCD_SetFont(&Font12);

    // Start servo pulse generation
    // pulse finish callback updates length of next pulse
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);

    // not to be enabled until BSP_MPU_GyroCalibration
    // Period elapsed callback sets flag PeriodElapsed
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

    //for water bubble
    xp = BSP_LCD_GetXSize() / 2;
    yp = BSP_LCD_GetYSize() / 2;

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

    // if program switch is on ( channels[7] low value ) and WriteUse switch
    // ( channels[9] ) is in the upper position copy pid_vars from upper flash page to ram
    if (channels[9] > 2400 && channels[7] < 1600)
    {
        // pid_vars from upper flash page to ram
        read_flash_vars(pid_vars, 9);
    }
    else
    {
        // pid_vars from constant flash values to ram
        for (i = 0; i < 9; i++)
        {
            pid_vars[i] = const_pid_vars[i];
        }
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        //systick_val1 = SysTick->VAL;
        if (PeriodElapsed == 1) // 200 Hz from servo timer
        {
            PeriodElapsed = 0;
            counter++;
            failsafe_counter++;

            if (RC_RECEIVED == 1)
            {
                RC_RECEIVED = 0;
                failsafe_counter = 0;
            }

            BSP_MPU_read_rot();
            BSP_MPU_read_acc();

            if (channels[4] < 2400 || failsafe_counter > 40) // motor stop
            {
                halt_reset();
            }
            else // armed, flight mode
            {
                // just attitude hold mode
                // full stick equals 250 degrees per second
                diffroll = gy[x] * 8.0f - (float) channels[1] + MIDDLE_POS; // native middle positions
                diffnick = gy[y] * 8.0f - (float) channels[2] + MIDDLE_POS;
                diffgier = gy[z] * 8.0f + (float) channels[3] - MIDDLE_POS; // control reversed, gy right direction

                thrust_set = (int16_t) channels[0] + LOW_OFFS; // native middle position and 134 % are set
                roll_set = pid(x, diffroll, pid_vars[RKp], pid_vars[RKi], pid_vars[RKd], 5.0f);
                nick_set = pid(y, diffnick, pid_vars[NKp], pid_vars[NKi], pid_vars[NKd], 5.0f);
                gier_set = pid(z, diffgier, pid_vars[GKp], pid_vars[GKi], pid_vars[GKd], 5.0f);

                control(thrust_set, roll_set, nick_set, gier_set);
                // assured finished before first servo update by HAL_TIM_PWM_PulseFinishedCallback

                //systick_val2 = SysTick->VAL;
                //sprintf(buf, "elapsed: %ld usec\n", (systick_val1 - systick_val2) / 72);
            }

            //tick = HAL_GetTick();
            //dt = tick - prev_tick;
            //prev_tick = tick;

            BSP_MPU_updateIMU(ac[x], ac[y], ac[z], gy[x], gy[y], gy[z], 5.0f);
            BSP_MPU_getEuler(&roll, &pitch, &yaw);

            //free_ram = (0x20000000 + 1024 * 20) - (uint32_t) sbrk((int)0);
            //sprintf(buf, "free: %ld\n", free_ram);
            //free_flash = (0x8000000 + 1024 * 128) - (uint32_t) &flash_top;
            //sprintf(buf, "free: %ld bytes\n", free_flash);

            //sprintf(buf, "%d %d %d %d %d %d\n", channels[0], channels[1], channels[2], channels[3] , channels[4], channels[5]);
            //sprintf(buf, "dt: %ld\n", dt);
            //sprintf(buf, "%3.3f,%3.3f,%3.3f\n", yaw, pitch, roll);
            //sprintf(buf, "%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f\n", ac[x], ac[y], ac[z], gy[x], gy[y], gy[z]);
            //sprintf(buf, "%d %d %d %d\n", servos[0], servos[1], servos[2], servos[3]);
            //sprintf(buf, "%3.3f %3.3f %3.3f %ld %ld\n", gy[x], gy[y], gy[z], dt, idle_counter);
            //sprintf(buf, "HAL_UART_ERROR: %d\n", HAL_UART_ERROR);
            //sprintf(buf, "roll_set: %d\n", roll_set);

            // do it in time pieces
            if (counter == 4)
            {
                counter = 0;

                if (channels[4] < 2400) // motor stop screen
                {
                    // transition to motor stop clear screen
                    if (back_channels[4] - channels[4] > 1000)
                    {
                        BSP_LCD_SetRotation(0);
                        BSP_LCD_Clear(LCD_COLOR_BLACK);
                        back_channels[4] = channels[4];

                    }

                    // transition of beeper momentary switch (channels[6]) detect
                    if (channels[6] - back_channels[6] > 500)
                    {
                        // if program switch (channels[7]) is off
                        // increment indexer
                        if (channels[7] > 2400)
                        {
                            if (indexer < 8)
                            {
                                indexer++;
                            }
                            else
                            {
                                indexer = 0;
                            }

                        }
                        // else if program switch (channels[7]) is on
                        // and WriteUse switch channels[9] is in upper position
                        // copy pid_vars from ram to upper flash page
                        else if (channels[7] < 800 && channels[9] > 2400)
                        {
                            if (erase_flash_page() != HAL_OK)
                            {
                                Error_Handler();
                            }
                            else
                            {
                                if (write_flash_vars(pid_vars, 9) != HAL_OK)
                                {
                                    Error_Handler();
                                }
                            }
                        }
                    }

                    back_channels[6] = channels[6];

                    // show and program by RC the current PID values
                    draw_program_pid_values(2, pid_vars[RKp], "Roll Kp: %3.5f", indexer, 2);
                    draw_program_pid_values(3, pid_vars[RKi], "Roll Ki: %3.5f", indexer, 2);
                    draw_program_pid_values(4, pid_vars[RKd], "Roll Kd: %3.5f", indexer, 2);
                    draw_program_pid_values(5, pid_vars[NKp], "Nick Kp: %3.5f", indexer, 2);
                    draw_program_pid_values(6, pid_vars[NKi], "Nick Ki: %3.5f", indexer, 2);
                    draw_program_pid_values(7, pid_vars[NKd], "Nick Kd: %3.5f", indexer, 2);
                    draw_program_pid_values(8, pid_vars[GKp], "Gier Kp: %3.5f", indexer, 2);
                    draw_program_pid_values(9, pid_vars[GKi], "Gier Ki: %3.5f", indexer, 2);
                    draw_program_pid_values(10, pid_vars[GKd], "Gier Kd: %3.5f", indexer, 2);

                }
                else // armed, flight mode screen
                {
                    // transition to armed, flight mode, clear screen
                    if (channels[4] - back_channels[4] > 1000)
                    {
                        BSP_LCD_SetRotation(3);
                        BSP_LCD_Clear(LCD_COLOR_BLACK);
                        back_channels[4] = channels[4];
                    }

                    //########### water bubble ################################

                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_DrawCircle(xp, yp, 5);

                    BSP_LCD_SetTextColor(LCD_COLOR_RED);
                    BSP_LCD_DrawHLine(75, 64, 11);
                    BSP_LCD_DrawVLine(80, 59, 11);

                    vx = sinf(pitch) * 300.0f;
                    vy = sinf(roll) * 300.0f;

                    xp = rintf(vx) + 80;
                    yp = rintf(vy) + 64;

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

                    //############ end water bubble ###########################
                }
            }

            if (counter == 3)
            {
                HAL_ADCEx_Calibration_Start(&hadc1);
                if (HAL_ADC_Start(&hadc1) == HAL_OK)
                {
                    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
                    {
                        volt1 = (HAL_ADC_GetValue(&hadc1)) / 219.84f; // calibrate, resistors 8.2k 1.8k
                    }
                    HAL_ADC_Stop(&hadc1);
                }

                // beeper not enabled in program mode (program switch channels[7] low value)
                // let default value of 2000 included for beeping
                if ((volt1 < 10.2f || channels[6] > 2000) && (channels[7] > 1800))
                {
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                }
            }

            if (counter == 2)
            {
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // CDC_Transmit_FS((uint8_t*) buf, strlen(buf));
            idle_counter = 0;

        }
        else
        {
            idle_counter++; //in flight mode minimum value before reset is > 450 with sbus running
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

/**
 *  @brief Shows PID value, highlight selected line, writes new value on selected line according RC
 *  @param Number of line, PID value of this line, format string, select index, offset line to index
 *  @retval none
 */
void draw_program_pid_values(uint8_t line, float value, char* format, uint8_t index, uint8_t offset)
{
    /*
     Currently used functions/names to channel mapping on my DC16:
     1 (channels[0]) f4 (Thrust)
     2 (channels[1]) f1 (Roll)
     3 (channels[2]) f2 (Nick) // I prefer the german word since I fly helicopters back in the eighties
     4 (channels[3]) f3 (Gier) // I prefer the german word ...
     5 (channels[4]) sd (Arm)
     6 (channels[5]  sj (Atti/Hori/Baro) # for other copters
     7 (channels[6]  sa (Beeper)         # momentary switch
     8 (channels[7]  sc (Program)
     9 (channels[8]  f8 (Variable)       # knob
     10 (channels[9] sb (WriteUse)       # three positions
     */

    // if indexer points to current line and we are in program mode ( channels[7] low )
    // and WriteUse switch is not in upper position, then
    // the new value adjustable by variable knob (channels[8]) is shown
    if ((indexer == line - offset) && (channels[7] < 1600) && (channels[9] < 2400))
    {
        switch (indexer)
        {
        case 0: //RKp
            value = (float) channels[8] / 8000.0f;
            break;

        case 1: //RKi
            value = (float) channels[8] / 2000.0f;
            break;

        case 2: //RKd
            value = (float) channels[8] / 200000.0f;
            break;

        case 3:  //NKp
            value = (float) channels[8] / 8000.0f;
            break;

        case 4:  //NKi
            value = (float) channels[8] / 2000.0f;
            break;

        case 5:  //NKd
            value = (float) channels[8] / 200000.0f;
            break;

        case 6:  //GKp
            value = (float) channels[8] / 2000.0f;
            break;

        case 7:  //GKi
            value = (float) channels[8] / 2000.0f;
            break;

        case 8:  //GKd
            value = (float) channels[8] / 200000.0f;
            break;
        }

        // if beeper switch is switched to lower position the new value will be written immediately
        // and continuously to ram (pid_vars[x]) while adjusting the value with the knob
        if (channels[6] > 2400)
        {
            pid_vars[indexer] = value;
        }
    }

    BSP_LCD_SetTextColor(indexer == line - offset ? LCD_COLOR_BLUE : LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(indexer == line - offset ? LCD_COLOR_BLUE : LCD_COLOR_BLACK);
    BSP_LCD_FillRect(60, line * 12, BSP_LCD_GetXSize() - 80, 12);
    sprintf(buf, format, value);
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
    BSP_LCD_DisplayStringAtLine(line, (uint8_t *) buf);
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
