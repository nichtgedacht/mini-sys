#include "servo.h"
#include "tim.h"

volatile uint16_t servos[4]={1500}; //servo pulse width 1000 to 2000 us

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
            case HAL_TIM_ACTIVE_CHANNEL_1:
                TIM2->CCR1 = servos[0];
            break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
        	    TIM2->CCR2 = servos[1];
            break;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                TIM2->CCR3 = servos[2];
            break;
            case HAL_TIM_ACTIVE_CHANNEL_4:
                TIM2->CCR4 = servos[3];
            break;
            case HAL_TIM_ACTIVE_CHANNEL_CLEARED:
            break;
        }
	}
}
