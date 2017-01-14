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
const settings default_settings = {
        0xdb,   // magic
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
        {     1,      0,     0 }, // x
        {     0,      1,     0 }, // y
        {     0,      0,     1 }  // z
        },
        0xff,  // padding
        0xff,  // padding
        0xff,  // padding
        1.0f,  // aspect_ratio
        { {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}, {9, 0}, {10, 0}, {11, 0}, {12, 0} }, // rc_func
        0xff,  // padding
        { {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}, {9, 0}, {10, 0}, {11, 0}, {12, 0} }, // rc_ch
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

uint8_t rcv_settings = 0;
uint8_t snd_settings = 0;
uint8_t snd_channels = 0;
uint8_t snd_live = 0;
uint8_t rcv_motors = 0;
uint8_t live_receipt = 0;
uint8_t channels_receipt = 0;

void check_settings_page(void)
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
    p_settings = (settings *) flash_buf;

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

void start_bootloader(void)
{
    HAL_NVIC_SystemReset();
}

void config_state_switch(const char *cmd)
{
    char buf[20];

    if (strcmp(cmd, "reboot") == 0)
    {
        start_bootloader();
    }
    else if (strcmp(cmd, "bootloader") == 0)
    {
        //setting flag in flash
        //so reborn as bootloader we can know that we should not start this live immediately again
        // write string "DFU" (4 bytes incl. trailing \0) to last 32 bit wide space of flash page
        write_flash_vars((uint32_t*) "DFU", 1, 1020);
        start_bootloader();
    }
    else if (strcmp(cmd, "push_settings") == 0)
    {
        rcv_settings = 1;

        snd_live = 0;
        snd_channels = 0;
        rcv_motors = 0;

        CDC_Reset_Receive();

        HAL_Delay(100);

        sprintf(buf, "ok_push\n");
        CDC_Transmit_FS((uint8_t*) buf, strlen(buf));

#ifdef HAVE_DISPLAY
        sprintf(buf, "XXXXXXXXXX");
        BSP_LCD_DisplayCLRStringAtLine(10, (uint8_t *) buf, LEFT_MODE, LCD_COLOR_RED);
#endif

    }                                        // request for sending settings received
    else if (strcmp(cmd, "pull_settings") == 0)
    {
        snd_settings = 1;

        snd_live = 0;
        snd_channels = 0;
        rcv_motors = 0;

        CDC_Reset_Receive();

        read_flash_vars((uint32_t *) flash_buf, 256, 0);
#ifdef HAVE_DISPLAY
        sprintf(buf, "XXXXXXXXXX");
        BSP_LCD_DisplayCLRStringAtLine(11, (uint8_t *) buf, LEFT_MODE, LCD_COLOR_RED);
#endif

    }
    else if (strcmp(cmd, "load_defaults") == 0)
    {
        load_default_settings();

        CDC_Reset_Receive();

        //HAL_NVIC_SystemReset(); // shall we reboot right now?
    }
    /*
     else if (strcmp(cmd, "suspend") == 0)
     {
     snd_live = 0;
     snd_channels = 0;
     rcv_motors = 0;

     sprintf(buf2, "ok_suspend\n");
     CDC_Transmit_FS((uint8_t*) buf2, strlen(buf2));

     CDC_Reset_Receive();

     }
     */
    else if (strcmp(cmd, "fw_tab") == 0)
    {
        snd_live = 0;
        snd_channels = 0;
        rcv_motors = 0;

        CDC_Reset_Receive();

#ifdef HAVE_DISPLAY
        sprintf(buf, "Firmware");
        BSP_LCD_DisplayCLRStringAtLine(0, (uint8_t *) buf, CENTER_MODE, LCD_COLOR_RED);
#endif
    }
    else if (strcmp(cmd, "config_tab") == 0)
    {
        snd_live = 0;
        snd_channels = 1;
        channels_receipt = 1;
        rcv_motors = 0;

        CDC_Reset_Receive();

#ifdef HAVE_DISPLAY
        sprintf(buf, "Configuration");
        BSP_LCD_DisplayCLRStringAtLine(0, (uint8_t *) buf, CENTER_MODE, LCD_COLOR_RED);
#endif
    }
    else if (strcmp(cmd, "motors_tab") == 0)
    {
        snd_live = 0;
        snd_channels = 0;
        rcv_motors = 1;

        CDC_Reset_Receive();

#ifdef HAVE_DISPLAY
        sprintf(buf, "Motor Test");
        BSP_LCD_DisplayCLRStringAtLine(0, (uint8_t *) buf, CENTER_MODE, LCD_COLOR_RED);
#endif
    }
    else if (strcmp(cmd, "flight_tab") == 0)
    {
        snd_live = 0;
        snd_channels = 0;
        rcv_motors = 0;

        CDC_Reset_Receive();

#ifdef HAVE_DISPLAY
        sprintf(buf, "Flight Setup");
        BSP_LCD_DisplayCLRStringAtLine(0, (uint8_t *) buf, CENTER_MODE, LCD_COLOR_RED);
#endif
    }
    else if (strcmp(cmd, "live_tab") == 0)
    {
        snd_live = 1;
        snd_channels = 0;
        rcv_motors = 0;
        live_receipt = 1;

        CDC_Reset_Receive();

#ifdef HAVE_DISPLAY
        sprintf(buf, "Live Plots");
        BSP_LCD_Clear(LCD_COLOR_BLACK);
        BSP_LCD_DisplayCLRStringAtLine(0, (uint8_t *) buf, CENTER_MODE, LCD_COLOR_RED);
#endif
    }
    else if (strcmp(cmd, "live_receipt") == 0)
    {
        live_receipt = 1;

        CDC_Reset_Receive();
    }
    else if (strcmp(cmd, "channels_receipt") == 0)
    {
        channels_receipt = 1;

        CDC_Reset_Receive();
    }
    else if (rcv_motors == 1)
    {
        servos[motor1_tim_ch] = atoi(strtok((char *) cmd, ","));
        servos[motor2_tim_ch] = atoi(strtok(NULL, ","));
        servos[motor3_tim_ch] = atoi(strtok(NULL, ","));
        servos[motor4_tim_ch] = atoi(strtok(NULL, ","));

        sprintf(buf, "motors_receipt\n");
        CDC_Transmit_FS((uint8_t*) buf, strlen(buf));

        CDC_Reset_Receive();
    }
    else // recover from broken input
    {
        CDC_Reset_Receive();
    }
}

void receive_settings(void)
{
    char buf[20];

    if (cdc_received_tot < 1024)
    {
        cdc_received = 0;
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
    else // 1k received
    {
#ifdef HAVE_DISPLAY
        sprintf(buf, "%d", cdc_received_tot);
        BSP_LCD_DisplayCLRStringAtLine(10, (uint8_t *) buf, LEFT_MODE, LCD_COLOR_GREEN);
#endif

        rcv_settings = 0;

        CDC_Reset_Receive();

        p_settings = (settings *) received_data;

        if (erase_flash_page() != HAL_OK)
        {
            Error_Handler();
        }
        else
        {
            if (p_settings->magic == 0xdb)
            {
                if (write_flash_vars((uint32_t*) received_data, 256, 0) != HAL_OK)
                {
                    Error_Handler();
                }
                else
                {
                    sprintf(buf, "settings_rcvd\n");
                    CDC_Transmit_FS((uint8_t*) buf, strlen(buf));
                }
            }
            else
            {
                Error_Handler();
            }
        }
    }
}

void send_settings(uint8_t* flash)
{
    char buf[10];
    uint8_t res;

    res = CDC_Transmit_FS(flash, 1024);

#ifdef HAVE_DISPLAY
    sprintf(buf, "%d", res);
    BSP_LCD_DisplayCLRStringAtLine(11, (uint8_t *) buf, LEFT_MODE, LCD_COLOR_GREEN);
#endif

    if (res != USBD_BUSY)
    {
        snd_settings = 0;
    }
}

void send_channels(void)
{
    char buf[100];

    sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d\n", channels[rc_thrust], channels[rc_roll], channels[rc_nick],
            channels[rc_gier], channels[rc_arm], channels[rc_mode], channels[rc_beep], channels[rc_prog],
            channels[rc_var], channels[rc_aux1], channels[rc_aux2], channels[rc_aux3]);

    channels_receipt = 0;

    CDC_Transmit_FS((uint8_t*) buf, strlen((const char*) buf));
}

void send_live(void)
{
    char buf[100];

    sprintf(buf, "%1.6f %1.6f %1.6f %4.3f %4.3f %4.3f %1.6f %1.6f %1.6f\n", ac[se_roll], ac[se_nick], ac[se_gier],
            gy[se_roll], gy[se_nick], gy[se_gier], ang[roll], ang[nick], ang[gier]);

    CDC_Transmit_FS((uint8_t*) buf, strlen(buf));
    live_receipt = 0;
}

#ifdef HAVE_DISPLAY
/**
 *  @brief Shows PID value, highlight selected line, writes new value on selected line according RC
 *  @param Number of line, PID value of this line, format string, select index, offset line to index
 *  @retval none
 */
void draw_program_pid_values(uint8_t line, float value, char* format, uint8_t index, uint8_t offset)
{
    char buf[20];

    /*
     Currently used functions/names to channel mapping on my DC16:
     1 (channels[0]) f4 (Thrust)
     2 (channels[1]) f1 (Roll)
     3 (channels[2]) f2 (Nick) // I prefer the german word since I fly helicopters back in the eighties
     4 (channels[3]) f3 (Gier) // I prefer the german word ...
     5 (channels[4]) sd (Arm)            # two position switch
     6 (channels[5]  sj (Mode)           # three position switch  Attitude Hold / Level Hold
     7 (channels[6]  sa (Beeper)         # two position momentary switch
     8 (channels[7]  sc (Program)        # three position switch
     9 (channels[8]  f8 (Variable)       # knob proportinal
     */

    // if indexer points to current line and we are in program mode ( channels[rc_prog] middle )
    // then the new value adjustable by variable knob (channels[rc_var]) is shown
    if ((index == line - offset) && (channels[rc_prog] > 1400) && (channels[rc_prog] < 2700))
    {
        switch (index)
        {
        case 0: //RKp
            value = (float) channels[rc_var] / 4000.0f;
            //value = (float) channels[rc_var] / 8000.0f;
            break;

        case 1: //RKi
            value = (float) channels[rc_var] / 4000.0f;
            //value = (float) channels[rc_var] / 2000.0f;
            break;

        case 2: //RKd
            value = (float) channels[rc_var] / 50000.0f;
            //value = (float) channels[rc_var] / 200000.0f;
            break;

        case 3: //NKp
            value = (float) channels[rc_var] / 4000.0f;
            //value = (float) channels[rc_var] / 8000.0f;
            break;

        case 4: //NKi
            value = (float) channels[rc_var] / 4000.0f;
            //value = (float) channels[rc_var] / 2000.0f;
            break;

        case 5: //NKd
            value = (float) channels[rc_var] / 50000.0f;
            //value = (float) channels[rc_var] / 200000.0f;
            break;

        case 6: //GKp
            value = (float) channels[rc_var] / 2000.0f;
            break;

        case 7: //GKi
            value = (float) channels[rc_var] / 2000.0f;
            break;

        case 8: //GKd
            value = (float) channels[rc_var] / 200000.0f;
            break;
        }

        // if beeper switch is switched to lower position the new value will be written immediately
        // and continuously to ram (pid_vars[x]) while adjusting the value with the knob
        if (channels[rc_beep] > 2700)
        {
            if (channels[rc_mode] < 1400)
            {
                pid_vars[index] = value;
            }
            else
            {
                l_pid_vars[index] = value;
            }
        }
    }

    BSP_LCD_SetTextColor(index == line - offset ? LCD_COLOR_BLUE : LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(index == line - offset ? LCD_COLOR_BLUE : LCD_COLOR_BLACK);
    BSP_LCD_FillRect(80, line * 12, BSP_LCD_GetXSize() - 80, 12);
    sprintf(buf, format, value);
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
    BSP_LCD_DisplayStringAtLine(line, (uint8_t *) buf);
}
#endif

