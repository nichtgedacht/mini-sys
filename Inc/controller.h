#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "servo.h"
#include "math.h"

extern float diffroll;
extern float diffnick;
extern float diffgier;

extern int16_t thrust_set;
extern int16_t roll_set;
extern int16_t nick_set;
extern int16_t gier_set;

extern float last_derivative[3];
extern float last_error[3];
extern float integrator[3];

/*
extern const float RKp;
extern const float RKi;
extern const float RKd;

extern const float NKp;
extern const float NKi;
extern const float NKd;

extern const float GKp;
extern const float GKi;
extern const float GKd;
*/

//extern float pid_vars[9];
//extern float l_pid_vars[9];
//extern const float const_pid_vars[9];
//extern const float const_l_pid_vars[9];

//enum { RKp, RKi, RKd, NKp, NKi, NKd, GKp, GKi, GKd };

int16_t pid(uint8_t axis, float scale, float error, float Kp, float Ki, float Kd, float dt);
void control(int16_t thrust_set, int16_t roll_set, int16_t nick_set, int16_t gier_set);
void halt_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROLLER_H*/
