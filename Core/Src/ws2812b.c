#include "ws2812b.h"
#include "main.h"
#include <string.h>

#define BYTES_PER_LED  12
#define RESET_BYTES    80

// Layout: [pre-reset][LED data][post-reset]
// Pre-reset absorbs any MOSI glitch at SPI transfer start
static uint8_t buf[RESET_BYTES + WS_NUM_LEDS * BYTES_PER_LED + RESET_BYTES];
#define LED_OFFSET  RESET_BYTES

extern SPI_HandleTypeDef hspi2;

static const uint8_t lut[4] = {0x88, 0x8E, 0xE8, 0xEE};

static void encode_byte(uint8_t *out, uint8_t val) {
    out[0] = lut[(val >> 6) & 3];
    out[1] = lut[(val >> 4) & 3];
    out[2] = lut[(val >> 2) & 3];
    out[3] = lut[(val >> 0) & 3];
}

void ws2812b_set_pixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
    if (idx >= WS_NUM_LEDS) return;
    uint8_t *p = &buf[LED_OFFSET + idx * BYTES_PER_LED];
    encode_byte(p + 0, g);
    encode_byte(p + 4, r);
    encode_byte(p + 8, b);
}

void ws2812b_clear(void) {
    for (int i = 0; i < WS_NUM_LEDS; i++)
        ws2812b_set_pixel(i, 0, 0, 0);
}

void ws2812b_show(void) {
    memset(buf, 0, RESET_BYTES);
    memset(&buf[LED_OFFSET + WS_NUM_LEDS * BYTES_PER_LED], 0, RESET_BYTES);
    HAL_SPI_Transmit(&hspi2, buf, sizeof(buf), HAL_MAX_DELAY);
}