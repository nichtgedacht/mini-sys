#include "config.h"
#include "flash.h"

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
    rc_channel rc_func[13];
    uint8_t pad7;
    rc_channel rc_ch[13];
    uint8_t pad8;
    uint8_t receiver;

 */

// see structure above
const settings default_settings =
{       0xdb,   // magic
        0xff,   // padding
        0xff,   // padding
        0xff,   // padding
        { 0.24f, 1.5f, 0.004f, 0.24f, 1.5f, 0.004f, 0.5f, 1.5f, 0.001f },  // pid_vars
        { 0.1f, 0.03f, 0.02f, 0.1f, 0.03f, 0.02f, 1.0f, 1.5f, 0.001f },    // l_pid_vars
        { 250.0f, 250.0f, 250.0f },  // rate
        { CCW, 1 },  // motor_1
        { CW, 2 },   // motor_2
        { CW, 3 },   // motor_3
        { CCW, 4 },  // motor_4
        { // Front   Left   Top   //sensor_orient
            { 1,      0,     0 }, // x
            { 0,      1,     0 }, // y
            { 0,      0,     1 }  // z
        },
        0xff,  // padding
        0xff,  // padding
        0xff,  // padding
        1.0f,  // aspect_ratio
        { {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}, {9, 0}, {10, 0}, {11, 0}, {12, 0} },
        0xff,  // padding
        { {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}, {9, 0}, {10, 0}, {11, 0}, {12, 0} },
        0xff,  // padding
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
uint8_t se_roll, se_nick, se_gier;
float se_roll_sign, se_nick_sign, se_gier_sign;

uint8_t motor1_tim_ch;
uint8_t motor2_tim_ch;
uint8_t motor3_tim_ch;
uint8_t motor4_tim_ch;

uint8_t rc_thrust;
uint8_t rc_roll;
uint8_t rc_nick;
uint8_t rc_gier;
uint8_t rc_arm;
uint8_t rc_mode;
uint8_t rc_beep;
uint8_t rc_prog;
uint8_t rc_var;
uint8_t rc_aux1;
uint8_t rc_aux2;
uint8_t rc_aux3;

uint8_t rc_rev[12];

int8_t rot_dir[4];

float scale_nick;
float scale_roll;

settings *p_settings;

uint8_t receiver;

void check_settings(void)
{
    p_settings = (settings *) flash_buf;
    read_flash_vars((uint32_t *) flash_buf, 256, 0);

    // if flash page is not valid
    // regenerate from default values
    if (p_settings->magic != 0xdb)
    {
        load_default_settings();
    }
}

void analyze_settings(void)
{
    uint8_t i, j;

    for (i = 0; i < 9; i++)
    {
        pid_vars[i] = p_settings->pidvars[i];
        l_pid_vars[i] = p_settings->l_pidvars[i];
    }

    for (i = 0; i < 3; i++)
    {
        // translate rate deg/s to appropriate factor
        rate[i] = 2000.0f / p_settings->rate[i];
    }

    if (p_settings->aspect_ratio > 1)
    {
        scale_nick = 1.0f / p_settings->aspect_ratio;
        scale_roll = 1.0f;
    }
    else if (p_settings->aspect_ratio < 1)
    {
        scale_roll = 1.0f / p_settings->aspect_ratio;
        scale_nick = 1.0f;
    }
    else
    {
        scale_nick = 1.0f;
        scale_roll = 1.0f;
    }

    for (j = 0; j < 3; ++j)
    {
        for (i = 0; i < 3; ++i)
        {
            if (p_settings->sensor_orient[i][j] != 0)
            {
                if (j == 0) // X vector of Sensor
                {
                    se_roll = i; // direction the vector points to or opposite direction if sign is negative
                    se_roll_sign = p_settings->sensor_orient[i][j];
                }
                else if (j == 1) // Y vector of sensor
                {
                    se_nick = i; // direction the vector points to or opposite direction if sign is negative
                    se_nick_sign = p_settings->sensor_orient[i][j];
                }
                else if (j == 2) // Z vector of sensor
                {
                    se_gier = i; // direction the vector points to or opposite direction if sign is negative
                    se_gier_sign = p_settings->sensor_orient[i][j];
                }

                break;
            }
        }
    }

    motor1_tim_ch = p_settings->motor_1.tim_ch - 1;
    motor2_tim_ch = p_settings->motor_2.tim_ch - 1;
    motor3_tim_ch = p_settings->motor_3.tim_ch - 1;
    motor4_tim_ch = p_settings->motor_4.tim_ch - 1;

    rot_dir[M1] = p_settings->motor_1.rotational_direction;
    rot_dir[M2] = p_settings->motor_2.rotational_direction;
    rot_dir[M3] = p_settings->motor_3.rotational_direction;
    rot_dir[M4] = p_settings->motor_4.rotational_direction;

    // assign to pseudo channels[16] (value fixed to 2000) if channel number is 0
    // real channels start from index 0
    rc_thrust = p_settings->rc_func[r_thrust].number > 0 ? p_settings->rc_func[r_thrust].number - 1 : 16;
    rc_roll = p_settings->rc_func[r_roll].number > 0 ? p_settings->rc_func[r_roll].number - 1 : 16;
    rc_nick = p_settings->rc_func[r_nick].number > 0 ? p_settings->rc_func[r_nick].number - 1 : 16;
    rc_gier = p_settings->rc_func[r_gier].number > 0 ? p_settings->rc_func[r_gier].number - 1 : 16;
    rc_arm = p_settings->rc_func[r_arm].number > 0 ? p_settings->rc_func[r_arm].number - 1 : 16;
    rc_mode = p_settings->rc_func[r_mode].number > 0 ? p_settings->rc_func[r_mode].number - 1 : 16;
    rc_beep = p_settings->rc_func[r_beep].number > 0 ? p_settings->rc_func[r_beep].number - 1 : 16;
    rc_prog = p_settings->rc_func[r_prog].number > 0 ? p_settings->rc_func[r_prog].number - 1 : 16;
    rc_var = p_settings->rc_func[r_var].number > 0 ? p_settings->rc_func[r_var].number - 1 : 16;
    rc_aux1 = p_settings->rc_func[r_aux1].number > 0 ? p_settings->rc_func[r_aux1].number - 1 : 16;
    rc_aux2 = p_settings->rc_func[r_aux2].number > 0 ? p_settings->rc_func[r_aux2].number - 1 : 16;
    rc_aux3 = p_settings->rc_func[r_aux3].number > 0 ? p_settings->rc_func[r_aux3].number - 1 : 16;

    for (i = 0; i < 12; ++i)
    {
        rc_rev[i] = p_settings->rc_ch[i + 1].rev;
    }
}

void load_default_settings(void)
{
    memset(flash_buf, 0xff, 1024);

    *p_settings = default_settings;

    if (erase_flash_page() != HAL_OK)
    {
        Error_Handler();
    }
    else
    {
        // write back flash buffer
        write_flash_vars((uint32_t*) flash_buf, 256, 0);
    }
}

