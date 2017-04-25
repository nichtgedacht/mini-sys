#include "servo.h"

volatile uint16_t servos[4] =
{ 4000, 4000, 4000, 4000 }; //servo pulse width 1000 to 2000 us step is 0.25 us (4 MHz clock i.e 72 / 18)
                            //So values here going from 4000 to 8000
volatile uint8_t PeriodElapsed = 0;
volatile uint8_t ServoPeriodElapsed = 0;

// timer4 is synchronized to servo timer and has n times its period frequency
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim4)
    {
        PeriodElapsed = 1;
    }
    else if (htim == &htim2)
    {
        ServoPeriodElapsed = 1;
    }
}

