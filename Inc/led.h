#ifndef __LED_H
#define __LED_H

#include "stm32f1xx_hal.h"
#include "math.h"

#define NR_LEDS 12
#define NR_COLORS 25

enum
{
    G, R, B
};

extern uint8_t aCCValue_Buffer[];
extern uint8_t led_val[][3];
extern uint8_t led_tmp_val[3];

void led_trans_vals(void);
void led_rotate_right(uint8_t from, uint8_t to);
void led_rotate_left(uint8_t from, uint8_t to);
void led_set_rainbow(uint8_t from, uint8_t to, uint8_t brightness);
void led_set_armed_acro(uint8_t brightness);
void led_set_armed_level_hold(uint8_t brightness);

#endif
