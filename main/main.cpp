/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "functional_helpers.hpp"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

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

    LD2420::version_buf_t verBuf;
    auto e = presence.OpenCommandMode()
                | and_then([&](LD2420 &d, LD2420::OpenCmdModeResponse prot){
                        printf("Protocol: %d; Buffer=%d\n", prot.protocol_version, prot.buffer_size);
                        return d.GetVersion(verBuf);
                  })
                | and_then([](LD2420 &d, std::string_view ver){
                        ((char*)ver.data())[ver.size()] = 0;
                        printf("Version: %s\n", ver.data());
                        return d.ReadRawADBSingle(0x0000);
                  })
                | and_then([](LD2420 &d, uint32_t val){
                        printf("Param %X: %lX\n", 0, val);
                        return d.ReadRawADBSingle(0x0001);
                  })
                | and_then([](LD2420 &d, uint32_t val){
                        printf("Param %X: %lX\n", 0x0001, val);
                        return d.ReadRawADBSingle(0x0002);
                  })
                | and_then([](LD2420 &d, uint32_t val){
                        printf("Param %X: %lX\n", 0x0002, val);
                        return d.ReadRawADBSingle(0x0003);
                  })
                | and_then([](LD2420 &d, uint32_t val){
                        printf("Param %X: %lX\n", 0x0003, val);
                        return d.ReadRawADBSingle(0x0010);
                  })
                | and_then([](LD2420 &d, uint32_t val){
                        printf("Param %X: %lX\n", 0x0010, val);
                        return d.ReadRawADBSingle(0x0011);
                  })
                | and_then([](LD2420 &d, uint32_t val){
                        printf("Param %X: %lX\n", 0x0011, val);
                        return d.ReadRawADBSingle(0x0012);
                  })
                | and_then([](LD2420 &d, uint32_t val){
                        printf("Param %X: %lX\n", 0x0012, val);
                        return d.CloseCommandMode();
                  })
                ;
    //auto e = presence.OpenCommandMode()
    //            | and_then([&]{ return presence.SetSystemMode(0x04); })//energy
    //            | and_then([&]{ return presence.ReadRawADBMulti(uint16_t(0x0), uint16_t(0x01), uint16_t(0x04)); })
    //            | and_then([&](LD2420 &d, LD2420::Params<3> &params){ 
    //                    printf("Min gate distance: %d\nMax gate distance: %d\nTimeout: %d\n"
    //                            , (uint16_t)params.value[0]
    //                            , (uint16_t)params.value[1]
    //                            , (uint16_t)params.value[2]);
    //                    using p = LD2420::ADBParam;
    //                    return presence.WriteRawADBMulti(p{0x0, 1}, p{0x01, 12}, p{0x04, 120}); 
    //            })
    //            | and_then([&]{ 
    //                    std::this_thread::sleep_for(duration_ms_t(100));
    //                    return presence.ReadRawADBMulti(uint16_t(0x0), uint16_t(0x01), uint16_t(0x04)); 
    //                    })
    //            | and_then([&](LD2420 &d, LD2420::Params<3> &params){ 
    //                    printf("After set:\nMin gate distance: %d\nMax gate distance: %d\nTimeout: %d\n"
    //                            , (uint16_t)params.value[0]
    //                            , (uint16_t)params.value[1]
    //                            , (uint16_t)params.value[2]);
    //                    return d.CloseCommandMode();
    //              });
    if (!e)
    {
        print_ld2420_error(e.error());
        return;
    }

    //uint8_t buf[1024];
    //for(int i = 0; i < 50000; ++i)
    //{
    //    size_t l;
    //    if (auto e = presence.GetReadyToReadDataLen(); !e)
    //    {
    //        printf("Reading length failed. Iteration %d", i);
    //        print_ld2420_error(e.error());
    //        return;
    //    }else
    //        l = e.value().v;
    //
    //    if (l)
    //    {
    //        printf("Available %d bytes\n", l);
    //        if (auto e = presence.Read(buf, l); !e)
    //        {
    //            printf("Reading failed. Iteration %d", i);
    //            print_ld2420_error(e.error());
    //            return;
    //        }else
    //            l = e.value().v;
    //
    //        print_bytes(std::span<uint8_t>(buf, l));
    //        fflush(stdout);
    //    }else
    //    {
    //        std::this_thread::sleep_for(duration_ms_t(500));
    //        //vTaskDelay(10 / portTICK_PERIOD_MS);
    //    }
    //}

    //if (!e)
    //{
    //    print_ld2420_error(e.error());
    //    return;
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
