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
#include "ld2420_component.hpp"

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

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    ld2420::Component ld2420;
    ld2420.SetCallbackOnMovement([&](bool presence, float distance){
            printf("Presence: %d; Distance: %f; Addr: %p\n", (int)presence, distance, &ld2420);
    });
    if (!ld2420.Setup(ld2420::Component::setup_args_t{.txPin=11, .rxPin=10, .presencePin=8}))
    {
        printf("Failed to configure ld2420\n");
        fflush(stdout);
        return;
    }
    //ld2420.ChangeMode(LD2420::SystemMode::Energy);
    fflush(stdout);

    while(true)
    {
        vTaskDelay(5 * 1000 / portTICK_PERIOD_MS);
    }

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
