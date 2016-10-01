#include "controller.h"
#include "mpu9250.h"
#include "config.h"

float diffroll = 0.0f;
float diffnick = 0.0f;
float diffgier = 0.0f;

int16_t thrust_set = 0;
int16_t roll_set = 0;
int16_t nick_set = 0;
int16_t gier_set = 0;

float last_derivative[3];
float last_error[3];
float integrator[3];

const float RC = 0.007958;  // 1/(2*PI*_fCut fcut = 20 from Ardupilot

int16_t pid(uint8_t axis, float scale, float error, float Kp, float Ki, float Kd, float dt)
{
    float derivative = 0;
    float output_f = 0;
    int16_t output = 0;

    dt /= 1000.0f;

// P
    output_f += error * Kp;

    integrator[axis] += (error * Ki) * dt;
    //limit is about 90 degrees of rotation
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

    // Scale with value from aspect_ratio (nick only);
    output_f *= scale;

    //output = roundf(output_f);
    //output = lroundf(output_f);
    output = rintf(output_f);

    return output;
}

void control(int16_t thrust_set, int16_t roll_set, int16_t nick_set, int16_t gier_set)
{

    // prevents motor stop, hopefully
    gier_set = gier_set < -400 ? -400 : (gier_set > 400 ? 400 : gier_set);

    /* old original
    servos[0] = thrust_set - roll_set - nick_set - gier_set;  // Motor rear left   CCW
    servos[1] = thrust_set - roll_set + nick_set + gier_set;  // Motor front left  CW
    servos[2] = thrust_set + roll_set - nick_set + gier_set;  // Motor rear right  CW
    servos[3] = thrust_set + roll_set + nick_set - gier_set;  // Motor front right CCW
    */

    // change roll, nick, gier to x, y, z. We need 0, 1, 2 as fixed index
    servos[motor1_tim_ch] = thrust_set - roll_set - nick_set + gier_set * rot_dir[M1];  // Motor 1 rear left   CCW
    servos[motor2_tim_ch] = thrust_set - roll_set + nick_set + gier_set * rot_dir[M2];  // Motor 2 front left  CW
    servos[motor3_tim_ch] = thrust_set + roll_set - nick_set + gier_set * rot_dir[M3];  // Motor 3 rear right  CW
    servos[motor4_tim_ch] = thrust_set + roll_set + nick_set + gier_set * rot_dir[M1];  // Motor 4 front right CCW


    // It is essential that these statements are completed within the current period before
    // the values are taken by HAL_TIM_PWM_PulseFinishedCallback (in servo.c).
    // That is < 1ms since PeriodElapsed flag was set.
    servos[motor1_tim_ch] = servos[motor1_tim_ch] < 4000 ? 4000 : (servos[motor1_tim_ch] > 8000 ? 8000 : servos[motor1_tim_ch]);
    servos[motor2_tim_ch] = servos[motor2_tim_ch] < 4000 ? 4000 : (servos[motor2_tim_ch] > 8000 ? 8000 : servos[motor2_tim_ch]);
    servos[motor3_tim_ch] = servos[motor3_tim_ch] < 4000 ? 4000 : (servos[motor3_tim_ch] > 8000 ? 8000 : servos[motor3_tim_ch]);
    servos[motor4_tim_ch] = servos[motor4_tim_ch] < 4000 ? 4000 : (servos[motor4_tim_ch] > 8000 ? 8000 : servos[motor4_tim_ch]);

    /*
     Mapping:

     normal:
     #############################    thrust  roll-right  nick-down  gier-right
     servos[0] Motor front left  CCW     +         +          -           +
     servos[1] Motor front right CW      +         -          -           -
     servos[2] Motor rear left   CW      +         +          +           +
     servos[3] Motor rear right  CCW     +         -          +           -

     connected:
     servos[0] Motor rear left   CW
     servos[2] Motor rear right  CCW
     servos[1] Motor front right CW
     servos[3] Motor front left  CCW
     */

}

void halt_reset(void)
{
    servos[0] = 4000;
    servos[1] = 4000;
    servos[2] = 4000;
    servos[3] = 4000;
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
