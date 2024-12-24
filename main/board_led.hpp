#ifndef BOARD_LED_HPP
#define BOARD_LED_HPP
#include "lib/misc_helpers.hpp"
#include <cstdint>

namespace led
{
    struct Color
    {
        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
        uint8_t a = 255;
    };

    void setup();
    void blink(bool on, Color c);
    void blink_pattern(uint32_t pattern, Color c, duration_ms_t dur);
}
#endif
