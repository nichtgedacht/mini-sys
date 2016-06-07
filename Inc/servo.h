#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "tim.h"

extern volatile uint16_t servos[4];
extern volatile uint8_t PeriodElapsed;

#endif /* __SERVO_H*/
