#include "controller.h"

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

const float RKp = 0.24f;
const float RKi = 1.5f;
const float RKd = 0.002f;

const float NKp = 0.24f;
const float NKi = 1.5f;
const float NKd = 0.002f;

const float GKp = 1.5f;
const float GKi = 1.5f;
const float GKd = 0.001f;

const float RC = 0.007958;  // 1/(2*PI*_fCut fcut = 20 from Ardupilot

int16_t pid(uint8_t axis, float error, float Kp, float Ki, float Kd, float dt)
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

    output = roundf(output_f);

    return output;
}

void control(int16_t thrust_set, int16_t roll_set, int16_t nick_set, int16_t gier_set)
{
    servos[3] = thrust_set - roll_set + nick_set + gier_set;  // Motor front left  CCW
    servos[1] = thrust_set + roll_set + nick_set - gier_set;  // Motor front right CW
    servos[0] = thrust_set - roll_set - nick_set - gier_set;  // Motor rear left   CW
    servos[2] = thrust_set + roll_set - nick_set + gier_set;  // Motor rear right  CCW

    if (servos[0] > 4000)
    {
        servos[0] = 4000;
    }
    else if (servos[0] < 2000)
    {
        servos[0] = 2000;
    }

    if (servos[1] > 4000)
    {
        servos[1] = 4000;
    }
    else if (servos[1] < 2000)
    {
        servos[1] = 2000;
    }

    if (servos[2] > 4000)
    {
        servos[2] = 4000;
    }
    else if (servos[2] < 2000)
    {
        servos[2] = 2000;
    }
    if (servos[3] > 4000)
    {
        servos[3] = 4000;
    }
    else if (servos[3] < 2000)
    {
        servos[3] = 2000;
    }

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
