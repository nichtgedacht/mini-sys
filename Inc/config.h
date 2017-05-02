#ifndef __config_H
#define __config_H

#include "stm32f1xx_hal.h"
#include "string.h"
#include "stm32_adafruit_lcd.h"
#include "servo.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "rc.h"
#include "mpu9250.h"

//#define HAVE_DISPLAY
//#define HAVE_SD_CARD

#define L_TRSH 1400
#define H_TRSH 2700
#define TRANS_OFFS 1000

enum { roll, nick, gier }; // all axis index
enum { SBUS, SRXL }; // receiver index
enum { RKp, RKi, RKd, NKp, NKi, NKd, GKp, GKi, GKd }; // pid index
enum { CW = 1, CCW = -1};
enum { M1, M2, M3, M4 };
enum { r_thrust = 1,
       r_roll = 2,
       r_nick = 3,
       r_gier = 4,
       r_arm = 5,
       r_mode = 6,
       r_beep = 7,
       r_prog = 8,
       r_var = 9,
       r_aux1 = 10,
       r_aux2 = 11,
       r_aux3 = 12
     };

typedef struct
{
    int8_t rotational_direction;
    uint8_t tim_ch;
} motor;

typedef struct
{
    uint8_t number;
    uint8_t rev;
} rc_channel;

typedef int8_t matrix[3][3];

/*
typedef struct {
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
    uint8_t pad9;
    uint8_t pad10;
    uint8_t pad11;
    float low_voltage;
} settings;
*/

// new 8 bytes alignment
typedef struct {
    uint8_t magic;
    uint8_t pad1[7];    // 1 + 7 = 8

    float pidvars[9];
    uint8_t pad2[4];    // 9 * 4 + 4 = 40

    float l_pidvars[9];
    uint8_t pad3[4];    // 9 * 4 + 4 = 40

    float rate[3];      // 3 * 4 + 4 = 16
    uint8_t pad4[4];

    motor motor_1;
    motor motor_2;
    motor motor_3;
    motor motor_4;      // 4 * 2 = 8

    matrix sensor_orient;
    uint8_t pad5[7];    // 3 * 3 + 7 = 16

    float aspect_ratio;
    uint8_t pad6[4];    // 4 + 4 = 8

    rc_channel rc_func[13];
    uint8_t pad7[6];    // 2 * 13 + 6 = 32

    rc_channel rc_ch[13];
    int8_t pad8[6];     // 2 * 13 + 6 = 32

    uint8_t receiver;
    int8_t pad9[7];     // 1 + 7 = 8

    float low_voltage;
    int8_t pad10[4];    // 4 + 4 = 8

    int32_t acc_offset[3];
    int8_t pad11[4];    // 3 * 4 + 4 = 16

} settings;

extern const settings default_settings;

extern float pid_vars[];
extern float l_pid_vars[];
extern float rate[];
extern uint8_t receiver;

extern motor motor_1;
extern motor motor_2;
extern motor motor_3;
extern motor motor_4;

extern float aspect_ratio;
extern uint8_t se_roll, se_nick, se_gier;
extern float se_roll_sign, se_nick_sign, se_gier_sign;

extern uint8_t motor1_tim_ch;
extern uint8_t motor2_tim_ch;
extern uint8_t motor3_tim_ch;
extern uint8_t motor4_tim_ch;

extern uint8_t rc_thrust;
extern uint8_t rc_roll;
extern uint8_t rc_nick;
extern uint8_t rc_gier;
extern uint8_t rc_arm;
extern uint8_t rc_mode;
extern uint8_t rc_beep;
extern uint8_t rc_prog;
extern uint8_t rc_var;
extern uint8_t rc_aux1;
extern uint8_t rc_aux2;
extern uint8_t rc_aux3;

extern uint8_t rc_rev[12];

extern int8_t rot_dir[4];

extern float scale_nick;
extern float scale_roll;

extern settings *p_settings;

extern uint8_t rcv_settings;
extern uint8_t snd_settings;
extern uint8_t snd_channels;
extern uint8_t snd_live;
extern uint8_t rcv_motors;
extern uint8_t live_receipt;
extern uint8_t channels_receipt;

extern float low_voltage;

// example:
// cannels[rc_thrust];

extern void Error_Handler(void);

void check_settings_page(void);
void analyze_settings(void);
void load_default_settings(void);
void start_bootloader (void);
void config_state_switch(const char *cmd);
void receive_settings(void);
void send_settings(uint8_t* flash);
void send_channels(void);
void send_live(void);
#ifdef HAVE_DISPLAY
void draw_program_pid_values(uint8_t line, float value, char* format, uint8_t index, uint8_t offset);
#endif

#endif
