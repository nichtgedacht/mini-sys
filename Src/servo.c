#include "servo.h"

volatile uint16_t servos[4] =
{ 2000, 2000, 2000, 2000 }; //servo pulse width 1000 to 2000 us step 0.5 us (2 MHz clock)
volatile uint8_t PeriodElapsed = 0;

/**
 * @brief  one servo pulse finished callbacks, update CCRx asap
 * @param  htim: Pointer to a TIM_HandleTypeDef structure that contains
 *                the configuration information for the specified Timer module.
 * @retval None
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        switch (htim->Channel)
        {
        case HAL_TIM_ACTIVE_CHANNEL_1:    // Pin 11 (maple) Motor rear left
            if (servos[0] > 4000)
            {
                servos[0] = 4000;
            }
            else if (servos[0] < 2000)
            {
                servos[0] = 2000;
            }
            TIM2->CCR1 = servos[0];
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:    // Pin 10 (maple) Motor front right
            if (servos[1] > 4000)
            {
                servos[1] = 4000;
            }
            else if (servos[1] < 2000)
            {
                servos[1] = 2000;
            }
            TIM2->CCR2 = servos[1];
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:    // Pin 9 (maple) Motor rear right
            if (servos[2] > 4000)
            {
                servos[2] = 4000;
            }
            else if (servos[2] < 2000)
            {
                servos[2] = 2000;
            }
            TIM2->CCR3 = servos[2];
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:    // Pin 8 (maple) Motor front left
            if (servos[3] > 4000)
            {
                servos[3] = 4000;
            }
            else if (servos[3] < 2000)
            {
                servos[3] = 2000;
            }
            TIM2->CCR4 = servos[3];
            break;
        case HAL_TIM_ACTIVE_CHANNEL_CLEARED:
            break;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //200 Hz, period 5 ms
{
    if (htim == &htim2)
    {
        PeriodElapsed = 1;
    }
}

