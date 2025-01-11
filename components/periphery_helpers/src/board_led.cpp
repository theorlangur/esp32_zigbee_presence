#include "ph_board_led.hpp"
#include "led_strip.h"
#include <thread>

namespace led
{

    static led_strip_handle_t led_strip;
    void setup()
    {
        /* LED strip initialization with the GPIO and pixels number*/
        led_strip_config_t strip_config = {
            .strip_gpio_num = 8,
            .max_leds = 1, // at least one LED on board
            .led_pixel_format = {}, //GRB
            .led_model = {},//WS2812
            .flags = {.invert_out = false}
        };
        led_strip_rmt_config_t rmt_config = {
            .clk_src = {},
            .resolution_hz = 10 * 1000 * 1000, // 10MHz
            .mem_block_symbols = {},
            .flags ={.with_dma= false},
        };
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }

    void blink(bool on, Color c)
    {
        /* If the addressable LED is enabled */
        if (on) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            led_strip_set_pixel(led_strip, 0, c.r, c.g, c.b);
            /* Refresh the strip to send data */
            led_strip_refresh(led_strip);
        } else {
            /* Set all LED off to clear all pixels */
            led_strip_clear(led_strip);
        }
    }

    void blink_pattern(uint32_t pattern, Color c, duration_ms_t dur)
    {
        auto dur_bit = dur / 32;
        int prevBit = -1;
        for(int i = 0; i < 32; ++i)
        {
            int bit = (pattern >> i) & 1;
            if (bit != prevBit)
            {
                //change
                blink(bit == 1, c);
            }
            prevBit = bit;
            std::this_thread::sleep_for(dur_bit);
        }
    }
}
