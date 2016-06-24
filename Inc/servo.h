#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "tim.h"

volatile uint16_t servos[4];
volatile uint8_t PeriodElapsed;

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H*/
