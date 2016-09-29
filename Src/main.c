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
#include "config.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define BOOTLOADER_ADDRESS 0x08000000

typedef void (*pFunction)(void);

uint8_t counter = 0;
uint32_t failsafe_counter = 100;
char* pDirectoryFiles[MAX_BMP_FILES];
uint8_t res;
uint16_t usb_res;
FRESULT fres;
DIR directory;
FATFS SD_FatFs; /* File system object for SD card logical drive */
float volt1 = 0.0f;
uint32_t free_ram;
char buf[50] =
{ 0 };
char buf2[70] =
{ 0 };
const uint8_t flash_top = 255;
uint32_t free_flash;
uint32_t tick, prev_tick, dt;
float e_roll, e_nick, e_gier;
int xp, yp;
float vx = 0.0f, vy = 0.0f;
HAL_StatusTypeDef hal_res;
uint32_t idle_counter;
float cp_pid_vars[9];
uint8_t i;
uint8_t indexer = 0;
uint32_t systick_val1, systick_val2;
uint8_t rcv_settings = 0;
uint8_t snd_settings = 0;
uint8_t snd_channels = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef HAVE_DISPLAY
void draw_program_pid_values(uint8_t line, float value, char* format, uint8_t index, uint8_t offset);
#endif

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

#ifdef HAVE_SD_CARD
    MX_FATFS_Init();
#endif

    // Reset USB on maple mine clone to let the PC host enumerate the device
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    check_settings();
    analyze_settings();

    if (p_settings->receiver == SBUS)
    {
        // alternative SBUS (Futaba) aka DBUS (Jeti)
        MX_SBUS_USART1_UART_Init();
        UART_RxCpltCallback = &HAL_UART_RxCpltCallback_SBUS;
        UART_ErrorCallback = &HAL_UART_ErrorCallback_SBUS;
        // Request first 25 bytes s-bus frame from uart, uart_data becomes filled per interrupts
        // Get callback if ready. Callback function starts next request
        HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 25);
    }
    else if (p_settings->receiver == SRXL)
    {
        // alternative SRXL16 (Multiplex) aka UDI16 (Jeti)
        MX_USART1_UART_Init();
        UART_RxCpltCallback = &HAL_UART_RxCpltCallback_SRXL;
        UART_ErrorCallback = &HAL_UART_ErrorCallback_SRXL;
        // Request first 35 bytes srxl frame from uart, uart_data becomes filled per interrupts
        // Get callback if ready. Callback function starts next request
        HAL_UART_Receive_IT(&huart1, (uint8_t*) uart_data, 35);
    }

    // no sample rate divider (0+1), accel: lowpass filter bandwidth 460 Hz, Rate 1kHz, gyro: lowpass filter bandwidth 250 Hz
    // gyro lpf, 3. parameter:
    // 0  250 Hz
    // 1  184 Hz
    // 2   92 Hz
    // 3   41 Hz
    BSP_MPU_Init(0, 2, 0);
    HAL_Delay(2000); // wait for silence after batteries plug in
    BSP_MPU_GyroCalibration();

#ifdef HAVE_DISPLAY
    BSP_LCD_Init();
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetRotation(0);
    BSP_LCD_SetFont(&Font12);

    //for water bubble
    xp = BSP_LCD_GetXSize() / 2;
    yp = BSP_LCD_GetYSize() / 2;
#endif

#ifdef HAVE_SD_CARD
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

        if (fres != FR_OK)
        {
            Error_Handler();
        }
        else
        {
            for (i = 0; i < MAX_BMP_FILES; i++)
            {
                pDirectoryFiles[i] = malloc(11);
            }
        }
    }
#endif

    // Start servo pulse generation
    // pulse finish callback updates length of next pulse
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);

    // not to be enabled until BSP_MPU_GyroCalibration
    // Period elapsed callback sets flag PeriodElapsed
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //systick_val1 = SysTick->VAL; // SysTick->VAL reference probe
        if (PeriodElapsed == 1) // 200 Hz from servo timer
        {

            //systick_val2 = SysTick->VAL;  // max 8 us

            PeriodElapsed = 0;
            counter++;
            failsafe_counter++;

            //systick_val2 = SysTick->VAL; // max 9 us

            if (RC_RECEIVED == 1)
            {
                RC_RECEIVED = 0;
                failsafe_counter = 0;
            }

            //systick_val2 = SysTick->VAL; // max 9 us

            BSP_MPU_read_rot();
            BSP_MPU_read_acc();

            //systick_val2 = SysTick->VAL; // max 122 us

            // use int values se_roll, se_nick, se_gier as index to map different orientations of the sensor
            BSP_MPU_updateIMU(ac[se_roll] * se_roll_sign, ac[se_nick] * se_nick_sign, ac[se_gier] * se_gier_sign, gy[se_roll] * se_roll_sign,
                    gy[se_nick] * se_nick_sign, gy[se_gier] * se_gier_sign, 5.0f); // dt 5ms

            // then it comes out here properly mapped because Quaternions already changed axises
            BSP_MPU_getEuler(&e_roll, &e_nick, &e_gier);

            //systick_val2 = SysTick->VAL;  // max 325 us

            // armed only if arm switch on + not failsafe + not usb connected
            if (channels[rc_arm] > 1600 && failsafe_counter < 40 && hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) // armed, flight mode
            //if (channels[rc_arm] > 1600 && failsafe_counter < 40 ) // test performance while usb connected
            {
                if (channels[rc_mode] < 1200)
                {
                    // attitude hold mode
                    // full stick equals ~250 degrees per second with rate of 8 (2000 / 250)
                    // gy range goes from 0 - 1000 [DPS]
                    diffroll = gy[se_roll] * se_roll_sign * rate[se_roll] - (float) channels[rc_roll] + MIDDLE_POS; // native middle positions
                    diffnick = gy[se_nick] * se_nick_sign * rate[se_nick] - (float) channels[rc_nick] + MIDDLE_POS;

                    // systick_val2 = SysTick->VAL; // max 334 us

                    roll_set = pid(x, scale_roll, diffroll, pid_vars[RKp], pid_vars[RKi], pid_vars[RKd], 5.0f);
                    nick_set = pid(y, scale_nick, diffnick, pid_vars[NKp], pid_vars[NKi], pid_vars[NKd], 5.0f);

                    //systick_val2 = SysTick->VAL; // max 396 us
                }
                else
                {
                    // level hold mode
                    // full stick 45 degrees
                    diffroll = e_roll * 2000.0f / (M_PI / 4.0f) - (float) channels[rc_roll] + MIDDLE_POS;
                    diffnick = e_nick * 2000.0f / (M_PI / 4.0f) - (float) channels[rc_nick] + MIDDLE_POS;

                    roll_set = pid(x, scale_roll, diffroll, l_pid_vars[RKp], l_pid_vars[RKi], l_pid_vars[RKd], 5.0f);
                    nick_set = pid(y, scale_nick, diffnick, l_pid_vars[NKp], l_pid_vars[NKi], l_pid_vars[NKd], 5.0f);
                }

                // full stick equals ~250 degrees per second with rate of 8 (2000 / 250)
                // gy range goes from 0 - 1000 [DPS]
                diffgier = gy[se_gier] * se_gier_sign * rate[se_gier] + (float) channels[rc_gier] - MIDDLE_POS; // control reversed, gy right direction
                gier_set = pid(z, 1.0f, diffgier, pid_vars[GKp], pid_vars[GKi], pid_vars[GKd], 5.0f);

                // systick_val2 = SysTick->VAL; // max 434 us

                // scale thrust channel to have space for governor if max thrust is set
                thrust_set = rintf((float) channels[rc_thrust] * 0.85f) + LOW_OFFS; // native middle position and 134 % are set
                //thrust_set = (int16_t) channels[rc_thrust] + LOW_OFFS; // native middle position and 134 % are set

                //systick_val2 = SysTick->VAL; // max 442 us

                control(thrust_set, roll_set, nick_set, gier_set);
                // assured finished before first servo update by HAL_TIM_PWM_PulseFinishedCallback (1000 us)

                //systick_val2 = SysTick->VAL; // max 488 us if all channels are inverted and receiver repeat rate 5ms

            }
            else // not armed or fail save, motor stop
            {
                halt_reset();

                // prevent overflow after many hours
                failsafe_counter = 100;

                // message over VCP arrived. From config tool?
                if (cdc_received == 1)
                {
                    // receiving 1k data for settings flash page
                    if (rcv_settings == 1)
                    {
                        if (cdc_received_tot < 1024)
                        {
                            cdc_received = 0;
                            // rearm receiving
                            USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                        }
                        else // 1k received
                        {
                            sprintf(buf, "%d", cdc_received_tot);
                            BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                            BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                            BSP_LCD_FillRect(0, 1 * 12, BSP_LCD_GetXSize(), 12);
                            BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                            BSP_LCD_DisplayStringAtLine(1, (uint8_t *) buf);

                            cdc_received = 0;
                            // reset index
                            cdc_received_tot = 0;
                            USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                            rcv_settings = 0;

                            if (erase_flash_page() != HAL_OK)
                            {
                                Error_Handler();
                            }
                            else
                            {
                                if (write_flash_vars((uint32_t*) received_data, 256, 0) != HAL_OK)
                                {
                                    Error_Handler();
                                }
                            }
                        }
                    }                                                 // request bootloader received
                    else if (strcmp((const char *) received_data, (const char *) "bootloader") == 0)
                    {
                        //setting flag in flash
                        //so reborn as bootloader we can know that we should not start this live immediately again
                        // write string "DFU" (4 bytes incl. trailing \0) to last 32 bit wide space of flash page
                        write_flash_vars((uint32_t*) "DFU", 1, 1020);

                        // is this really needed? I think it is not
                        HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
                        HAL_NVIC_DisableIRQ(USART1_IRQn);
                        HAL_NVIC_DisableIRQ(TIM2_IRQn);

                        // set stackpointer pointing to bootloader startup and reset system
                        __set_MSP(*(__IO uint32_t*) BOOTLOADER_ADDRESS);
                        HAL_NVIC_SystemReset();
                    }
                    else if (strcmp((const char *) received_data, (const char *) "reboot") == 0)
                    {
                        HAL_NVIC_SystemReset();
                    }                                        // request for receiving settings received
                    else if (strcmp((const char *) received_data, (const char *) "push_settings") == 0)
                    {
                        cdc_received_tot = 0;
                        cdc_received = 0;
                        rcv_settings = 1;
                        USBD_CDC_ReceivePacket(&hUsbDeviceFS);

                        sprintf(buf, "<%s>", "XXXX");
                        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                        BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                        BSP_LCD_FillRect(0, 1 * 12, BSP_LCD_GetXSize(), 12);
                        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                        BSP_LCD_DisplayStringAtLine(1, (uint8_t *) buf);

                    }                                        // request for sending settings received
                    else if (strcmp((const char *) received_data, (const char *) "pull_settings") == 0)
                    {
                        cdc_received_tot = 0;
                        cdc_received = 0;
                        snd_settings = 1;
                        //snd_channels = 0;
                        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                        read_flash_vars((uint32_t *) flash_buf, 256, 0);

                        sprintf(buf, "<%s>", "XXXX");
                        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                        BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                        BSP_LCD_FillRect(0, 0 * 12, BSP_LCD_GetXSize(), 12);
                        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                        BSP_LCD_DisplayStringAtLine(0, (uint8_t *) buf);

                        //HAL_Delay(300);
                    }
                    else if (strcmp((const char *) received_data, (const char *) "load_defaults") == 0)
                    {
                        load_default_settings();
                        cdc_received_tot = 0;
                        cdc_received = 0;
                        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                        //HAL_NVIC_SystemReset();
                    }
                    else if (strcmp((const char *) received_data, (const char *) "send_channels") == 0)
                    {
                        snd_channels = 1;
                        cdc_received_tot = 0;
                        cdc_received = 0;
                        USBD_CDC_ReceivePacket(&hUsbDeviceFS);

                        sprintf(buf, "send_channels");
                        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                        BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                        BSP_LCD_FillRect(0, 11 * 12, BSP_LCD_GetXSize(), 12);
                        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                        BSP_LCD_DisplayStringAtLine(11, (uint8_t *) buf);
                    }
                    else if (strcmp((const char *) received_data, (const char *) "stop_channels") == 0)
                    {
                        snd_channels = 0;
                        cdc_received_tot = 0;
                        cdc_received = 0;
                        USBD_CDC_ReceivePacket(&hUsbDeviceFS);

                        sprintf(buf, "stop_channels");
                        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                        BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                        BSP_LCD_FillRect(0, 11 * 12, BSP_LCD_GetXSize(), 12);
                        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                        BSP_LCD_DisplayStringAtLine(11, (uint8_t *) buf);
                    }
                    else // recover from broken input
                    {
                        cdc_received_tot = 0;
                        cdc_received = 0;
                        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                    }

                } //cdc_received

                // sending 1k data of settings flash page
                if (snd_settings == 1)
                {
                    HAL_Delay(300); // wait for live data on wire gets swallowed before send
                    usb_res = CDC_Transmit_FS((uint8_t*) flash_buf, 1024);

                    sprintf(buf, "%d", usb_res);
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                    BSP_LCD_FillRect(0, 0 * 12, BSP_LCD_GetXSize(), 12);
                    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                    BSP_LCD_DisplayStringAtLine(0, (uint8_t *) buf);

                    if (usb_res != USBD_BUSY)
                    {
                        snd_settings = 0;
                    }
                }
                else if (snd_channels == 1)  // sending channels
                {
                    sprintf(buf2, "%d %d %d %d %d %d %d %d %d %d %d %d\n", channels[rc_thrust], channels[rc_roll],
                            channels[rc_nick], channels[rc_gier], channels[rc_arm], channels[rc_mode],
                            channels[rc_beep], channels[rc_prog], channels[rc_var], channels[rc_aux1],
                            channels[rc_aux2], channels[rc_aux3]);

                    CDC_Transmit_FS((uint8_t*) buf2, strlen(buf2));

                    //for display in processing
                    //while not in flight mode must use dt instead 5.0f ms
                    //sprintf(buf2, "%3.3f,%3.3f,%3.3f\n", e_gier,
                    //                                     e_nick,
                    //                                     e_roll);

                }

                free_ram = (0x20000000 + 1024 * 20) - (uint32_t) sbrk((int) 0);
                sprintf(buf, "free: %ld", free_ram);
                //sprintf(buf, "ch6: %ld", channels[6]);
                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
                BSP_LCD_FillRect(0, 12 * 12, BSP_LCD_GetXSize(), 12);
                BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
                BSP_LCD_DisplayStringAtLine(12, (uint8_t *) buf);

            }

            //tick = HAL_GetTick();
            //dt = tick - prev_tick;
            //prev_tick = tick;
            //free_ram = (0x20000000 + 1024 * 20) - (uint32_t) sbrk((int)0);
            //sprintf(buf, "free: %ld", free_ram);
            //free_flash = (0x8000000 + 1024 * 128) - (uint32_t) &flash_top;
            //sprintf(buf, "free: %ld bytes\n", free_flash);

            //sprintf(buf, "%d %d %d %d %d %d\n", channels[rc_thrust], channels[rc_roll], channels[rc_nick], channels[rc_gier] , channels[rc_arm], channels[rc_mode]);
            //sprintf(buf, "dt: %ld\n", dt);
            //sprintf(buf2, "%3.3f,%3.3f,%3.3f\n", yaw, pitch, roll);
            //sprintf(buf, "%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f\n", ac[x], ac[y], ac[z], gy[x], gy[y], gy[z]);
            //sprintf(buf, "%d %d %d %d\n", servos[0], servos[1], servos[2], servos[3]);
            //sprintf(buf, "%3.3f %3.3f %3.3f %ld %ld\n", gy[x], gy[y], gy[z], dt, idle_counter);
            //sprintf(buf, "HAL_UART_ERROR: %d\n", HAL_UART_ERROR);
            //sprintf(buf, "thrust_set: %d channels[rc_thrust]: %d LOW_OFFS: %d\n", thrust_set, channels[rc_thrust], LOW_OFFS);
            //sprintf(buf, "%ld\r\n", idle_counter);

            // do it in time pieces
            if (counter >= 4)
            {
                counter = 0;
#ifdef HAVE_DISPLAY
                // stay in motor stop screen in any case if usb connected
                if (channels[rc_arm] < 2400 || hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) // motor stop screen
                //if (channels[rc_arm] < 2400) // test performance while usb connected
                {
                    // transition to motor stop clear screen
                    if (back_channels[rc_arm] - channels[rc_arm] > 1000)
                    {
                        BSP_LCD_SetRotation(0);
                        BSP_LCD_Clear(LCD_COLOR_BLACK);
                        back_channels[rc_arm] = channels[rc_arm];

                    }

                    // transition of beeper momentary switch (channels[rc_beep]) detect
                    if (channels[rc_beep] - back_channels[rc_beep] > 500)
                    {
                        // if program switch (channels[rc_prog]) is off
                        // increment indexer
                        if (channels[rc_prog] < 1200)
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
                        // else if program switch (channels[rc_prog]) full on
                        // copy pid_vars from ram to upper flash page
                        else if (channels[rc_prog] > 2800)
                        {
                            p_settings = (settings *) flash_buf;
                            read_flash_vars((uint32_t *) flash_buf, 256, 0);

                            for (i = 0; i < 9; i++)
                            {
                                // write ram vars to flash buffer
                                p_settings->pidvars[i] = pid_vars[i];
                                p_settings->l_pidvars[i] = l_pid_vars[i];
                            }

                            if (erase_flash_page() != HAL_OK)
                            {
                                Error_Handler();
                            }
                            else
                            {
                                if (write_flash_vars((uint32_t*) flash_buf, 256, 0) != HAL_OK)
                                {
                                    Error_Handler();
                                }
                            }
                        }
                    }

                    back_channels[rc_beep] = channels[rc_beep];

                    if (channels[rc_mode] < 1200)
                    {
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
                    else
                    {
                        // show and program by RC the current level flight PID values
                        draw_program_pid_values(2, l_pid_vars[RKp], "Roll Kp: %3.5f", indexer, 2);
                        draw_program_pid_values(3, l_pid_vars[RKi], "Roll Ki: %3.5f", indexer, 2);
                        draw_program_pid_values(4, l_pid_vars[RKd], "Roll Kd: %3.5f", indexer, 2);
                        draw_program_pid_values(5, l_pid_vars[NKp], "Nick Kp: %3.5f", indexer, 2);
                        draw_program_pid_values(6, l_pid_vars[NKi], "Nick Ki: %3.5f", indexer, 2);
                        draw_program_pid_values(7, l_pid_vars[NKd], "Nick Kd: %3.5f", indexer, 2);
                        draw_program_pid_values(8, l_pid_vars[GKp], "Gier Kp: %3.5f", indexer, 2);
                        draw_program_pid_values(9, l_pid_vars[GKi], "Gier Ki: %3.5f", indexer, 2);
                        draw_program_pid_values(10, l_pid_vars[GKd], "Gier Kd: %3.5f", indexer, 2);
                    }
                }
                else // armed, flight mode screen
                {
                    // transition to armed, flight mode, clear screen
                    if (channels[rc_arm] - back_channels[rc_arm] > 1000)
                    {
                        BSP_LCD_SetRotation(3);
                        BSP_LCD_Clear(LCD_COLOR_BLACK);
                        back_channels[rc_arm] = channels[rc_arm];
                    }

                    //########### water bubble ################################

                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                    BSP_LCD_DrawCircle(xp, yp, 5);

                    BSP_LCD_SetTextColor(LCD_COLOR_RED);
                    BSP_LCD_DrawHLine(75, 64, 11);
                    BSP_LCD_DrawVLine(80, 59, 11);

                    vx = sinf(e_nick) * 300.0f;
                    vy = sinf(e_roll) * 300.0f;

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
#endif
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

                // beeper not enabled if USB is connected
                // let default value 2000 of beeper channel included for beeping
                if ( ( volt1 < 10.5f || channels[rc_beep] > 2000 ) && hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED )
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

            if (counter == 1)
            {

            }

            //sprintf(buf2, "elapsed: %ld usec\n", (systick_val1 - systick_val2) / 72 );
            //sprintf(buf2, "%ld\n", idle_counter);
            //sprintf(buf2, "roll_set: %d nick_set: %d gier_set: %d\n", roll_set, nick_set, gier_set);

            //CDC_Transmit_FS((uint8_t*) buf2, strlen(buf2));

            idle_counter = 0;

        }
        else
        {
            idle_counter++;
            // min 723 now with running SRXL receiving
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

#ifdef HAVE_DISPLAY
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
     5 (channels[4]) sd (Arm)            # two position switch
     6 (channels[5]  sj (Mode)           # three position switch  Attitude Hold / Level Hold
     7 (channels[6]  sa (Beeper)         # two position momentary switch
     8 (channels[7]  sc (Program)        # two position switch
     9 (channels[8]  f8 (Variable)       # knob proportinal
     10 (channels[9] sb (Write)          # two position switch
     */

    // if indexer points to current line and we are in program mode ( channels[rc_prog] middle )
    // then the new value adjustable by variable knob (channels[rc_var]) is shown
    if ((indexer == line - offset) && (channels[rc_prog] > 1200) && (channels[rc_prog] < 2800))
    {
        switch (indexer)
        {
        case 0: //RKp
            value = (float) channels[rc_var] / 4000.0f;
            //value = (float) channels[rc_var] / 8000.0f;
            break;

        case 1: //RKi
            value = (float) channels[rc_var] / 4000.0f;
            //value = (float) channels[rc_var] / 2000.0f;
            break;

        case 2: //RKd
            value = (float) channels[rc_var] / 50000.0f;
            //value = (float) channels[rc_var] / 200000.0f;
            break;

        case 3:  //NKp
            value = (float) channels[rc_var] / 4000.0f;
            //value = (float) channels[rc_var] / 8000.0f;
            break;

        case 4:  //NKi
            value = (float) channels[rc_var] / 4000.0f;
            //value = (float) channels[rc_var] / 2000.0f;
            break;

        case 5:  //NKd
            value = (float) channels[rc_var] / 50000.0f;
            //value = (float) channels[rc_var] / 200000.0f;
            break;

        case 6:  //GKp
            value = (float) channels[rc_var] / 2000.0f;
            break;

        case 7:  //GKi
            value = (float) channels[rc_var] / 2000.0f;
            break;

        case 8:  //GKd
            value = (float) channels[rc_var] / 200000.0f;
            break;
        }

        // if beeper switch is switched to lower position the new value will be written immediately
        // and continuously to ram (pid_vars[x]) while adjusting the value with the knob
        if (channels[rc_beep] > 2800)
        {
            if (channels[rc_mode] < 1200)
            {
                pid_vars[indexer] = value;
            }
            else
            {
                l_pid_vars[indexer] = value;
            }
        }
    }

    BSP_LCD_SetTextColor(indexer == line - offset ? LCD_COLOR_BLUE : LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(indexer == line - offset ? LCD_COLOR_BLUE : LCD_COLOR_BLACK);
    BSP_LCD_FillRect(60, line * 12, BSP_LCD_GetXSize() - 80, 12);
    sprintf(buf, format, value);
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
    BSP_LCD_DisplayStringAtLine(line, (uint8_t *) buf);
}
#endif

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
    uint8_t i;
    for (i = 0; i < 6; i++)
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
