#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "zb_dev_def.hpp"
#include "zb_dev_def_cmd.hpp"
#include "../colors_def.hpp"
#include "driver/gpio.h"

namespace zb
{
    void reset_pin_isr(void *param)
    {
        QueueHandle_t q = (QueueHandle_t)param;
        int l = gpio_get_level(gpio_num_t(PINS_RESET));
        xQueueSendFromISR(q, &l, nullptr);
    }

    void config_reset_pin(QueueHandle_t q)
    {
        gpio_config_t reset_pin_cfg = {
            .pin_bit_mask = 1ULL << PINS_RESET,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
            .hys_ctrl_mode = gpio_hys_ctrl_mode_t{}
#endif
        };

        gpio_config(&reset_pin_cfg);
        gpio_isr_handler_add(gpio_num_t(PINS_RESET), reset_pin_isr, q);
    }

    void reset_button_loop()
    {
        QueueHandle_t mainQueue = xQueueCreate(10, sizeof(int));
        config_reset_pin(mainQueue);

        auto waitTime = portMAX_DELAY;
        // FACTORY_RESET_TIMEOUT_WAIT
        enum States{
            Idle,
            ResetPressed,
        } state = States::Idle;

        using clock_t = std::chrono::system_clock;
        auto pressed_time = clock_t::now();
        while(true)
        {
            int v;
            if (xQueueReceive(mainQueue, &v, waitTime)) //process
            {
                switch(state)
                {
                    case States::Idle:
                    {
                        if (v == 0)//pressed
                        {
                            FMT_PRINT("detected RESET pin LOW\n");
                            state = States::ResetPressed;
                            pressed_time = clock_t::now();
                            waitTime = FACTORY_RESET_TIMEOUT_WAIT;
                        }
                    }
                    break;
                    case States::ResetPressed:
                    {
                        //need a gate here to prevent sporadic changes
                        if (v == 1)//actually released. before timeout
                        {
                            state = States::Idle;
                            waitTime = portMAX_DELAY;

                            if (std::chrono::duration_cast<std::chrono::milliseconds>(clock_t::now() - pressed_time).count() > 100)
                            {
                                FMT_PRINT("detected RESET pin HIGH before timeout\n");
                                esp_restart();//simple restart
                            }else
                            {
                                FMT_PRINT("filtered out pin HIGH before timeout\n");
                            }
                        }
                    }
                    break;
                }
            }else
            {
                //timeout
                if (state == States::ResetPressed)
                {
                    //yep, pressed
                    //factory reset
                    FMT_PRINT("detected RESET pin LOW long time. Making Factory Reset\n");
                    waitTime = portMAX_DELAY;
                    state = States::Idle;
                    APILock l;
                    led::blink_pattern(colors::kBlinkPatternFactoryReset, colors::kColorSpecial, duration_ms_t(1500));
                    ld2412_cmd_factory_reset();
                    esp_restart();//not sure if this is necessarry, since zigbee factory resets should restart as well
                }
            }
        }
    }
}
