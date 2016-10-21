#include "servo.h"

volatile uint16_t servos[4] =
{ 4000, 4000, 4000, 4000 }; //servo pulse width 1000 to 2000 us step is 0.25 us (4 MHz clock)
                            //So values here going from 4000 to 8000
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
        case HAL_TIM_ACTIVE_CHANNEL_1:    // Pin 11 (maple)
            TIM2->CCR1 = servos[0]; //alt use __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, servos[0]);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:    // Pin 10 (maple)
            TIM2->CCR2 = servos[1];
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:    // Pin 9 (maple)
            TIM2->CCR3 = servos[2];
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:    // Pin 8 (maple)
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

