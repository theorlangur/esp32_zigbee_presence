/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "i2c.hpp"
#include "aht21.hpp"
#include <thread>
#include <atomic>
#include "ld2420.hpp"

void print_ld2420_error(::Err &uartErr) 
{ 
    printf("uart Error at %s: %s\n", uartErr.pLocation, esp_err_to_name(uartErr.code)); 
    fflush(stdout); 
}

void print_ld2420_error(LD2420::Err &e) 
{ 
    print_ld2420_error(e.uartErr);
    printf("LD2420 Error at %s: %s\n", e.pLocation, LD2420::err_to_str(e.code)); 
    fflush(stdout); 
}

void print_ld2420_error(LD2420::CmdErr &e) 
{ 
    print_ld2420_error(e.e);
    printf("CmdStatus %d\n", e.returnCode);
    fflush(stdout); 
}

void print_bytes(std::span<uint8_t> d)
{
    printf("1 Received hex bytes: ");
    for(uint8_t b : d)
        printf(" %X", b);
    printf("\n");
    ((char*)d.data())[d.size()] = 0;
    printf("ASCII:%s\n", (const char*)d.data());
    fflush(stdout);
}

struct QueueMsg
{
    enum class Type: size_t 
    {
        Stop,
        GPIO
    };

    Type m_Type;
    union
    {
        struct{
            gpio_num_t num;
        }m_GPIO;
    };
};

struct ld2420_isr
{
    QueueHandle_t q;
    gpio_num_t num;
};

void ld2420_pin_isr(void *_pCtx)
{
    ld2420_isr *pCtx = (ld2420_isr *)_pCtx;
    QueueMsg msg{.m_Type=QueueMsg::Type::GPIO, .m_GPIO={pCtx->num}};
    xQueueSendFromISR(pCtx->q, &msg, nullptr);
}

void ld2420_loop(LD2420 &d, QueueHandle_t q)
{
    QueueMsg msg;
    while(true)
    {
        if (xQueueReceive(q, &msg, 10000 / portTICK_PERIOD_MS))
        {
            switch(msg.m_Type)
            {
                case QueueMsg::Type::Stop: return;
                case QueueMsg::Type::GPIO: 
                {
                    int l = gpio_get_level(msg.m_GPIO.num);
                    printf("Level changed to %d", l);
                }
                break;
            }
        }
    }
}

extern "C" void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;

    LD2420 presence(uart::Port::Port1);
    if (auto e = presence.Init(11, 10); !e)
    {
        print_ld2420_error(e.error());
        return;
    }

    if (auto e = presence.ReloadConfig(); !e)
    {
        print_ld2420_error(e.error());
        return;
    }

    printf("Before change\n");
    printf("Version: %s\n", presence.GetVersion().data());
    printf("Current Mode: %d\n", (int)presence.GetSystemMode());
    printf("Min distance: %dm; Max distance: %dm; Timeout: %ld\n", presence.GetMinDistance(), presence.GetMaxDistance(), presence.GetTimeout());
    for(uint8_t i = 0; i < 16; ++i)
    {
        printf("Gate %d Thresholds: Move=%d Still=%d\n", i, presence.GetMoveThreshold(i), presence.GetStillThreshold(i));
    }

    //printf("Restarting...\n");
    //if (auto e = presence.Restart(); !e)
    //{
    //    print_ld2420_error(e.error());
    //    return;
    //}

    auto cfg = presence.ChangeConfiguration();
                                    cfg.SetTimeout(5)
                                    .SetSystemMode(LD2420::SystemMode::Energy)
                                    .SetMinDistanceRaw(1)
                                    .SetMaxDistanceRaw(12)
                                    .SetMoveThreshold(0, 60000)
                                    .SetStillThreshold(0, 40000)
                                    .SetMoveThreshold(1, 30000)
                                    .SetStillThreshold(1, 20000)
                                    .SetMoveThreshold(2, 400)
                                    .SetStillThreshold(2, 200)
                                    .SetMoveThreshold(3, 300)
                                    .SetStillThreshold(3, 250);
    
for(uint8_t gate = 4; gate < 16; ++gate)
{
    cfg.SetMoveThreshold(gate, 250)
        .SetStillThreshold(gate, 150);
}
auto changeConfig = cfg.EndChange();
                                    //.SetMoveThreshold(4, 250)
                                    //.SetStillThreshold(4, 150)
                                    //.SetMoveThreshold(5, 250)
                                    //.SetStillThreshold(5, 150)
                                    //.SetMoveThreshold(6, 250)
                                    //.SetStillThreshold(6, 150)
                                //.EndChange();

    if (!changeConfig)
    {
        print_ld2420_error(changeConfig.error());
        return;
    }

    if (auto e = presence.ReloadConfig(); !e)
    {
        print_ld2420_error(e.error());
        return;
    }

    printf("After change\n");
    printf("Version: %s\n", presence.GetVersion().data());
    printf("Current Mode: %d\n", (int)presence.GetSystemMode());
    printf("Min distance: %dm; Max distance: %dm; Timeout: %ld\n", presence.GetMinDistance(), presence.GetMaxDistance(), presence.GetTimeout());
    for(uint8_t i = 0; i < 16; ++i)
    {
        printf("Gate %d Thresholds: Move=%d Still=%d\n", i, presence.GetMoveThreshold(i), presence.GetStillThreshold(i));
    }

    QueueHandle_t ld2420_q = xQueueCreate(10, sizeof(QueueMsg));
    ld2420_isr isrCtx{ld2420_q, gpio_num_t(12)};
    gpio_config_t ld2420_presence_pin_cfg = {
        .pin_bit_mask = 1ULL << isrCtx.num,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&ld2420_presence_pin_cfg);

    std::jthread ld2420_task(ld2420_loop, presence, ld2420_q);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(isrCtx.num, ld2420_pin_isr, (void*) &isrCtx);

    //vTaskDelay(600 * 1000 / portTICK_PERIOD_MS);
    //while(true)
    //{
    //    if (presence.GetSystemMode() == LD2420::SystemMode::Simple)
    //    {
    //        if (auto te = presence.TryReadSimpleFrame(); !te)
    //        {
    //            print_ld2420_error(te.error());
    //            break;
    //        }
    //    }else
    //    {
    //        if (auto te = presence.TryReadEnergyFrame(); !te)
    //        {
    //            print_ld2420_error(te.error());
    //            break;
    //        }
    //    }
    //    auto p = presence.GetPresence();
    //    printf("Presence: %s; Distance: %.2fm\n", p.m_Detected ? "Detected" : "Clear", p.m_Distance);
    //    if (presence.GetSystemMode() == LD2420::SystemMode::Energy)
    //    {
    //        printf("Energy:");
    //        for(uint8_t i = 0; i < 16; ++i)
    //            printf("%d=%d ", (int)i, (int)presence.GetMeasuredEnergy(i));
    //        printf("\n");
    //    }
    //    fflush(stdout);
    //}

    fflush(stdout);
    return;

    auto print_error = [](auto &e) { printf("i2c Error at %s: %s\n", e.pLocation, esp_err_to_name(e.code)); fflush(stdout); };
        
    i2c::I2CBusMaster bus(i2c::SDAType(gpio_num_t(6)), i2c::SCLType(gpio_num_t(5)));
    auto r = bus.Open();
    if (!r)
    {
        print_error(r.error());
        return;
    }

    auto print_aht21_error = [&](auto &e) 
    { 
        printf("AHT21 Error at %s: %s\n", e.pLocation, AHT21::err_to_str(e.code));
        print_error(e.i2cErr);
    };
    AHT21 sensor(bus);
    if (auto r = sensor.Init(); !r)
    {
        print_aht21_error(r.error());
        return;
    }

    std::atomic_flag sensorUpdateRunning{true};
    std::jthread sensor_update([&]{
            while(true)
            {
                if (auto r = sensor.UpdateMeasurements(); !r)
                {
                    print_aht21_error(r.error());
                    sensorUpdateRunning.clear(std::memory_order_relaxed);
                    return;
                }
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
    });
        
    while(sensorUpdateRunning.test(std::memory_order_relaxed))
    {
        auto r = sensor.GetLastMeasurements();
        if (r)
        {
            auto [_t, _h] = r.value();
            printf("Temp: %f; Hum: %f\n", _t, _h);
            fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    printf("Something happened with the sensor. Loop stopped");

    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
