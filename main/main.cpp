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
#include "ld2412.hpp"

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

void test_ld2412()
{
    FMT_PRINT("2412: tests will start in 2 seconds\n");
    fflush(stdout);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    LD2412 d;
    if (auto te = d.Init(11, 10); !te)
    {
        FMT_PRINT("2412: Init failed: {}\n", te.error());
        return;
    }
    FMT_PRINT("2412: init done\n");
    fflush(stdout);

    //if (auto te = d.SwitchBluetooth(true); !te)
    //{
    //    FMT_PRINT("2412: Turning off bluetooth failed: {}\n", te.error());
    //    return;
    //}

    auto PrintConfig = [&](const char *pTitle){
    FMT_PRINT("\n{}:\n", pTitle);
    FMT_PRINT("Version: {}\n", d.GetVersion());
    FMT_PRINT("MAC: {}\n", d.GetBluetoothMAC());
    FMT_PRINT("Mode: {}\n", d.GetSystemMode());
    FMT_PRINT("Min dist raw: {}; Max dist raw: {};\n", d.GetMinDistanceRaw(), d.GetMaxDistanceRaw());
    FMT_PRINT("Timeout: {};\n", d.GetTimeout());
    FMT_PRINT("Out pin low on presence: {};\n", d.GetOutPinPolarity());
    FMT_PRINT("Move sensitivities: {};\n", d.GetAllMoveThresholds());
    FMT_PRINT("Still sensitivities: {};\n", d.GetAllStillThresholds());
    fflush(stdout);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    };
    PrintConfig("Initial Config");

    //{
    //    auto te = d.ChangeConfiguration()
    //        .SetTimeout(d.GetTimeout() + 1)
    //        .SetMaxDistanceRaw(7)
    //        .SetOutPinPolarity(true)
    //        .SetStillThreshold(1, 10)
    //        .EndChange();
    //    if (!te)
    //    {
    //        FMT_PRINT("2412: Changing config failed: {}\n", te.error());
    //        return;
    //    }
    //}
    //
    //if (auto te = d.ReloadConfig(); !te)
    //{
    //        FMT_PRINT("2412: Reloading config after change failed: {}\n", te.error());
    //        return;
    //}
    //PrintConfig("Config after change");
    //
    //if (auto te = d.FactoryReset(); !te)
    //{
    //        FMT_PRINT("2412: Factory reset failed: {}\n", te.error());
    //        return;
    //}
    //PrintConfig("Config after factory reset");
    //return;

    if (auto ce = d.ChangeConfiguration().SetSystemMode(LD2412::SystemMode::Energy).EndChange(); !ce)
    {
        FMT_PRINT("2412: Change mode failed: {}\n", ce.error());
        return;
    }

    while(true)
    {
        if (auto te = d.TryReadFrame(); !te)
        {
            FMT_PRINT("2412: reading frame failed: {}\n", te.error());
        }else
        {
            auto p = d.GetPresence();
            FMT_PRINT("2412: presence data: {}\n", p);
            FMT_PRINT("2412: eng data:\n{}\n", d.GetEngeneeringData());
        }
        //if (auto te = d.UpdateDistanceRes(); !te)
        //{
        //    FMT_PRINT("2412: updating distance res failed: {}\n", te.error());
        //}else
        //{
        //    FMT_PRINT("2412: distance res: {:x}\n", d.GetDistanceRes());
        //}
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

    //gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    test_ld2412();
    fflush(stdout);
    return;

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
