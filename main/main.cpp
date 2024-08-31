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

#include "i2c.hpp"
#include <thread>

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
    i2c::I2CBusMaster bus(i2c::SDAType(gpio_num_t(6)), i2c::SCLType(gpio_num_t(5)));
    bus.Open();
    auto aht21 = bus.Add(0x38);
    if (!aht21)
    {
        printf("i2c add device failed");
        return;
    }

    auto _aht21 = aht21
        .transform([](i2c::I2CDevice &d) { return std::ref(d); })
        .transform_error([](i2c::I2CBusMaster::Err e) { return i2c::I2CDevice::Err{e.pLocation, e.code};});

    auto print_error = [](auto &e) { printf("i2c Error at %s: %s", e.pLocation, esp_err_to_name(e.code)); };
    if (!_aht21)
    {
        print_error(_aht21.error());
        return;
    }

    auto read_status = [](i2c::I2CDevice &d){
        uint8_t cmd = 0x71;
        uint8_t status;
        return d.SendRecv(&cmd, sizeof(cmd), &status, sizeof(status))
        .transform([&](i2c::I2CDevice &d){
            return std::make_tuple(std::ref(d), status);
        });
    };

    auto init_aht21 = [&](i2c::I2CDevice &d)->i2c::I2CDevice::ExpectedResult{
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        return read_status(d)
            .and_then([](auto &&dev_n_status)->i2c::I2CDevice::ExpectedResult{ 
                    auto && [d,status] = dev_n_status;
                    if (!(status & 0x04))
                    {
                        //not calibrated
                        uint8_t init_cmd[] = {0xbe, 0x08, 0x00};
                        return d.Send(init_cmd, sizeof(init_cmd)).and_then([](i2c::I2CDevice &d)->i2c::I2CDevice::ExpectedResult{ 
                                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                return std::ref(d);
                        });
                    }
                    return std::ref(d);
            });
    };

    auto read_data = [&](i2c::I2CDevice &d){
        uint8_t trigger_measurement[] = {0xac, 0x33, 0x00};
        return d.Send(trigger_measurement, sizeof(trigger_measurement))
            .and_then([&](i2c::I2CDevice &d)->std::expected<std::tuple<float,float>,i2c::I2CDevice::Err>{
                    uint8_t data[7];
                    auto r = d.Recv(data, sizeof(data));
                    if (r)
                    {
                        if (data[0] & 0x80)//busy
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(40));
                            r = d.Recv(data, sizeof(data));
                            if (r)
                            {
                                uint32_t temperature   = data[3] & 0x0F;                //20-bit raw temperature data
                                temperature <<= 8;
                                temperature  |= data[4];
                                temperature <<= 8;
                                temperature  |= data[5];
                                float _t = ((float)temperature / 0x100000) * 200 - 50;

                                uint32_t humidity   = data[1];                          //20-bit raw humidity data
                                humidity <<= 8;
                                humidity  |= data[2];
                                humidity <<= 4;
                                humidity  |= data[3] >> 4;

                                if (humidity > 0x100000) {humidity = 0x100000;}             //check if RH>100, no need to check for RH<0 since "humidity" is "uint"

                                float _h = ((float)humidity / 0x100000) * 100;
                                return std::make_tuple(_t, _h);
                            }
                        }
                    }
                    return r.transform([&](auto &d){ return std::make_tuple(0.f, 0.f); });
            });
    };

    auto init_res = init_aht21(_aht21.value());
    if (!init_res)
    {
        print_error(init_res.error());
        return;
    }
    while(true)
    {
        auto data = read_data(_aht21.value());
        if (!data)
        {
            print_error(data.error());
            return;
        }
        auto [_t, _h] = data.value();
        printf("Temp: %f; Hum: %f\n", _t, _h);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }


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
