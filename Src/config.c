#include "config.h"


/*
const float default_pid_vars[9] = { 0.24f, 1.5f, 0.004f, 0.24f, 1.5f, 0.004f, 0.5f, 1.5f, 0.001f };
const float default_l_pid_vars[9] = { 0.1f, 0.03f, 0.02f, 0.1f, 0.03f, 0.02f, 1.0f, 1.5f, 0.001f };
const float default_rate[] = {250.0f, 250.0f, 250.0f };

const float default_r_scale = 1.0f;
const float default_n_scale = 1.0f;
const float default_g_scale = 1.0f;

// ############################## roll   nick   gier  tim_ch
//const motor default_motor_1 = { {-1.0f, -1.0f, -1.0f }, 0 };
//const motor default_motor_2 = { {-1.0f,  1.0f,  1.0f }, 1 };
//const motor default_motor_3 = { { 1.0f, -1.0f,  1.0f }, 2 };
//const motor default_motor_4 = { { 1.0f,  1.0f, -1.0f }, 3 };

//int8_t rotational_direction;
//uint8_t tim_ch;
const motor default_motor_1 = { CCW, 0 };
const motor default_motor_2 = { CW, 1 };
const motor default_motor_3 = { CW, 2 };
const motor default_motor_4 = { CCW, 3 };

const float default_aspect_ratio = 1.0f;

//normal orientation
const matrix default_sensor_orient = { // Front   Left   Top
                                     {     1,      0,     0  }, // x
                                     {     0,      1,     0  }, // y
                                     {     0,      0,     1  }  // z
                                     };

const uint8_t default_receiver = SRXL;
*/

/*
    uint8_t magic;
    uint8_t pad1;
    uint8_t pad2;
    uint8_t pad3;
    float pidvars[9];
    float l_pidvars[9];
    float rate[3];
    motor motor_1;
    motor motor_2;
    motor motor_3;
    motor motor_4;
    matrix sensor_orient;
    uint8_t pad4;
    uint8_t pad5;
    uint8_t pad6;
    float aspect_ratio;
    uint8_t rc_thrust;
    uint8_t rc_roll;
    uint8_t rc_nick;
    uint8_t rc_gier;
    uint8_t rc_arm;
    uint8_t rc_mode;
    uint8_t rc_beep;
    uint8_t rc_prog;
    uint8_t rc_var;
    uint8_t rc_write;
    uint8_t rc_aux1;
    uint8_t rc_aux2;
    uint8_t receiver;
*/

// see structure above
const settings default_settings = {
        0xdb,   // magic
        0xff,   // padding
        0xff,   // padding
        0xff,   // padding
        { 0.24f, 1.5f, 0.004f, 0.24f, 1.5f, 0.004f, 0.5f, 1.5f, 0.001f },  // pid_vars
        { 0.1f, 0.03f, 0.02f, 0.1f, 0.03f, 0.02f, 1.0f, 1.5f, 0.001f },    // l_pid_vars
        {250.0f, 250.0f, 250.0f },  // rate
        { CCW, 0 },  // motor_1
        { CW, 1 },   // motor_2
        { CW, 2 },   // motor_3
        { CCW, 3 },  // motor_4
        { // Front   Left   Top           //sensor_orient
        {     1,      0,     0  }, // x
        {     0,      1,     0  }, // y
        {     0,      0,     1  }  // z
        },
        0xff,  // padding
        0xff,  // padding
        0xff,  // padding
        1.0f,  // aspect_ratio
        1,     // Thrust Channel
        2,     // Roll Channel
        3,     // Nick Channel
        4,     // Gier Channel
        5,     // Arm Channel
        6,     // Mode Attitude/Level Channel
        7,     // Beeper Channel
        8,     // Program Channel
        9,     // Variable Channel
        10,    // Write Channel
        11,    // aux1 Channel
        12,    // aux2 Channel
        SRXL,  // Receiver type
};

float pid_vars[9];
float l_pid_vars[9];
float rate[3];

motor motor_1;
motor motor_2;
motor motor_3;
motor motor_4;

float aspect_ratio;
//matrix sensor_orient;
uint8_t roll, nick, gier;
float roll_sign, nick_sign, gier_sign;

uint8_t tim_ch_1;
uint8_t tim_ch_2;
uint8_t tim_ch_3;
uint8_t tim_ch_4;

uint8_t rc_thrust;
uint8_t rc_roll;
uint8_t rc_nick;
uint8_t rc_gier;
uint8_t rc_arm;
uint8_t rc_mode;
uint8_t rc_beep;
uint8_t rc_prog;
uint8_t rc_var;
uint8_t rc_write;
uint8_t rc_aux1;
uint8_t rc_aux2;

int8_t rot_dir[4];

float scale_nick;
float scale_roll;

uint8_t receiver;
