#include "led.h"

uint8_t aCCValue_Buffer[NR_LEDS * 24 + 8];
uint8_t led_val[NR_COLORS][3];
uint8_t led_tmp_val[3];

void led_trans_vals(void)
{
    uint8_t i, j;
    uint16_t k;

    k = 0;
    for (i = 0; i < NR_LEDS; i++) // each LED
    {
        for (j = 0; j < 3; j++) // 3 colors
        {
            aCCValue_Buffer[k++] = led_val[i][j] & 1 << 7 ? 60 : 30; // each bit
            aCCValue_Buffer[k++] = led_val[i][j] & 1 << 6 ? 60 : 30;
            aCCValue_Buffer[k++] = led_val[i][j] & 1 << 5 ? 60 : 30;
            aCCValue_Buffer[k++] = led_val[i][j] & 1 << 4 ? 60 : 30;
            aCCValue_Buffer[k++] = led_val[i][j] & 1 << 3 ? 60 : 30;
            aCCValue_Buffer[k++] = led_val[i][j] & 1 << 2 ? 60 : 30;
            aCCValue_Buffer[k++] = led_val[i][j] & 1 << 1 ? 60 : 30;
            aCCValue_Buffer[k++] = led_val[i][j] & 1 ? 60 : 30;
        }
    }
    for (i = 0; i < 8; i++)
    {
        aCCValue_Buffer[k++] = 0; // pulse pause
    }
}

void led_rotate_right(uint8_t from, uint8_t to)
{
    uint8_t i;

    led_tmp_val[G] = led_val[to][G];
    led_tmp_val[R] = led_val[to][R];
    led_tmp_val[B] = led_val[to][B];

    for (i = to; i > from; i--)
    {
        led_val[i][G] = led_val[i - 1][G];
        led_val[i][R] = led_val[i - 1][R];
        led_val[i][B] = led_val[i - 1][B];
    }

    led_val[from][G] = led_tmp_val[G];
    led_val[from][R] = led_tmp_val[R];
    led_val[from][B] = led_tmp_val[B];

    led_trans_vals();
}

void led_rotate_left(uint8_t from, uint8_t to)
{
    uint8_t i;

    led_tmp_val[G] = led_val[from][G];
    led_tmp_val[R] = led_val[from][R];
    led_tmp_val[B] = led_val[from][B];

    for (i = from; i < to; i++)
    {
        led_val[i][G] = led_val[i + 1][G];
        led_val[i][R] = led_val[i + 1][R];
        led_val[i][B] = led_val[i + 1][B];
    }
    led_val[to][G] = led_tmp_val[G];
    led_val[to][R] = led_tmp_val[R];
    led_val[to][B] = led_tmp_val[B];

    led_trans_vals();
}

void led_set_rainbow(uint8_t from, uint8_t to, uint8_t brightness)
{
    uint8_t index;
    float ratio, r, g, b;

    for (index = from; index < to; index++)
    {
        ratio = (float) index / (float) NR_COLORS;

        if (ratio < 1.0f / 3.0f)
        {
            r = (2.0f - ratio * 6.0f) * (float) brightness;
            g = ratio * 6.0f * (float) brightness;
            b = 0;
        }
        else if (ratio < 2.0f / 3.0f)
        {
            r = 0;
            g = (4.0f - ratio * 6.0f) * (float) brightness;
            b = (ratio * 6.0f - 2.0f) * (float) brightness;
        }
        else
        {
            r = (ratio * 6 - 4) * (float) brightness;
            g = 0;
            b = ((1 - ratio) * 6) * (float) brightness;
        }

        if (r > brightness)
        {
            r = brightness;
        }
        if (g > brightness)
        {
            g = brightness;
        }
        if (b > brightness)
        {
            b = brightness;
        }

        led_val[index][R] = rintf(r);
        led_val[index][G] = rintf(g);
        led_val[index][B] = rintf(b);
    }

    led_trans_vals();
}

void led_set_armed_acro(uint8_t brightness)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        led_val[i][G] = 0;
        led_val[i][R] = brightness;
        led_val[i][B] = 0;
    }
    for (i = 8; i < 12; i++)
    {
        led_val[i][G] = brightness;
        led_val[i][R] = brightness;
        led_val[i][B] = brightness;
    }

    led_trans_vals();
}

void led_set_armed_level_hold(uint8_t brightness)
{
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        led_val[i][G] = brightness;
        led_val[i][R] = brightness;
        led_val[i][B] = 0;
    }
    for (i = 4; i < 8; i++)
    {
        led_val[i][G] = 0;
        led_val[i][R] = brightness;
        led_val[i][B] = 0;
    }

    led_val[8][G] = brightness;
    led_val[8][R] = 0;
    led_val[8][B] = brightness;

    for (i = 9; i < 12; i++)
    {
        led_val[i][G] = brightness;
        led_val[i][R] = brightness;
        led_val[i][B] = brightness;
    }

    led_trans_vals();
}
