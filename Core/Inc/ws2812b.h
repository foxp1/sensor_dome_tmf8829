#pragma once
#include <stdint.h>

#define WS_NUM_LEDS 8

void ws2812b_set_pixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
void ws2812b_clear(void);
void ws2812b_show(void);         // blocking
void ws2812b_show_dma(void);     // non-blocking, use TxCplt callback
