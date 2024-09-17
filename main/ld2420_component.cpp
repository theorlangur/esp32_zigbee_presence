#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>
#include "ld2420_component.hpp"
#include "ld2420.hpp"
#include "driver/gpio.h"
#include <thread>

namespace ld2420
{
    QueueHandle_t g_FastQueue;
    QueueHandle_t g_ManagingQueue;
    LD2420 g_Presence(uart::Port::Port1);

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

    void presence_pin_isr(void *_)
    {
        QueueMsg msg{.m_Type=QueueMsg::Type::PresenceIntr};
        xQueueSendFromISR(g_FastQueue, &msg, nullptr);
    }

    static void ld2420_fast_loop(LD2420 &d)
    {
        printf("Entering level tracking loop\n");
        fflush(stdout);
        QueueMsg msg;
        while(true)
        {
            if (xQueueReceive(g_FastQueue, &msg, 10000 / portTICK_PERIOD_MS))
            {
                switch(msg.m_Type)
                {
                    case QueueMsg::Type::Stop: return;
                    case QueueMsg::Type::PresenceIntr: 
                    {
                        int l = gpio_get_level(kLD2420_PresencePin);
                        printf("Level changed to %d\n", l);
                        fflush(stdout);
                    }
                    break;
                    default:
                    //don't care
                    printf("Unprocessed message of type %d\n", (int)msg.m_Type);
                    break;
                }
            }
        }
    }

    static void ld2420_handle_msg(LD2420 &d, QueueMsg &msg)
    {
        switch(msg.m_Type)
        {
            case QueueMsg::Type::Restart:
                {
                    auto te = d.Restart();
                    if (!te)
                    {
                        //report
                    }
                }
                break;
            case QueueMsg::Type::SetMode:
                {
                    auto te = d.ChangeConfiguration()
                        .SetSystemMode(msg.m_Mode == Mode::Simple ? LD2420::SystemMode::Simple : LD2420::SystemMode::Energy)
                        .EndChange();
                    if (!te)
                    {
                        //report
                    }
                }
                break;
            case QueueMsg::Type::SetTimeout:
                {
                    auto te = d.ChangeConfiguration()
                        .SetTimeout(msg.m_Timeout)
                        .EndChange();
                    if (!te)
                    {
                        //report
                    }
                }
                break;
            case QueueMsg::Type::SetMinDistance:
                {
                    auto te = d.ChangeConfiguration()
                        .SetMinDistance(msg.m_MinDistance)
                        .EndChange();
                    if (!te)
                    {
                        //report
                    }
                }
                break;
            case QueueMsg::Type::SetMaxDistance:
                {
                    auto te = d.ChangeConfiguration()
                        .SetMaxDistance(msg.m_MaxDistance)
                        .EndChange();
                    if (!te)
                    {
                        //report
                    }
                }
                break;
            default:
                //don't care
                //report
                break;
        }
    }

    static void ld2420_managing_loop(LD2420 &d)
    {
        bool initial = true;
        bool lastPresence;
        float lastDistance;
        QueueMsg msg;
        while(true)
        {
            if (xQueueReceive(g_ManagingQueue, &msg, 200 / portTICK_PERIOD_MS)) //process
                ld2420_handle_msg(d, msg);

            bool simpleMode = d.GetSystemMode() == LD2420::SystemMode::Simple;
            auto te = simpleMode
                ? d.TryReadSimpleFrame(3/*, true*/)
                : d.TryReadEnergyFrame(3/*, true*/);

            bool reportPresence = false;
            bool reportDistance = false;
            auto p = d.GetPresence();
            if (!te)
            {
                //report?
            }else if (initial)
            {
                reportPresence = !simpleMode;//in simple mode the reporting is via interrupt
                reportDistance = true;
                initial = false;
                lastPresence = p.m_Detected;
                lastDistance = p.m_Distance;
            }else if (lastPresence != p.m_Detected)
            {
                lastPresence = p.m_Detected;
                lastDistance = p.m_Distance;
                reportDistance = true;
                reportPresence = !simpleMode;//in simple mode the reporting is via interrupt
            }else if (std::abs(p.m_Distance - lastDistance) > kDistanceReportChangeThreshold)//10cm
            {
                reportDistance = true;
                lastDistance = p.m_Distance;
            }

            if (reportDistance && reportPresence)
            {
                msg.m_Type = QueueMsg::Type::PresenceAndDistance;
                msg.m_PresenceAndDistance.m_Presence = lastPresence;
                msg.m_PresenceAndDistance.m_Distance = lastDistance;
            }else if (reportPresence)
            {
                msg.m_Type = QueueMsg::Type::Presence;
                msg.m_Presence = lastPresence;
            }else if (reportDistance)
            {
                msg.m_Type = QueueMsg::Type::Distance;
                msg.m_Distance = lastDistance;
            }

            if (reportDistance || reportPresence)
                xQueueSend(g_FastQueue, &msg, nullptr);
        }
    }

    static configure_presence_isr_pin()
    {
        gpio_config_t ld2420_presence_pin_cfg = {
            .pin_bit_mask = 1ULL << kLD2420_PresencePin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        gpio_config(&ld2420_presence_pin_cfg);
        gpio_isr_handler_add(gpio_num_t(kLD2420_PresencePin), presence_pin_isr, nullptr);
    }

    bool setup_ld2420()
    {
        auto e = g_Presence.Init(11, 10) 
            | functional::and_then([&]{ return g_Presence.ReloadConfig(); });

        if (!e)
        {
            print_ld2420_error(e.error());
            return false;
        }

        auto changeConfig = g_Presence.ChangeConfiguration()
                                .SetSystemMode(LD2420::SystemMode::Simple)
                                .SetTimeout(5)
                                .EndChange();
        if (!changeConfig)
        {
            print_ld2420_error(changeConfig.error());
            return;
        }

        printf("Version: %s\n", presence.GetVersion().data());
        printf("Current Mode: %d\n", (int)presence.GetSystemMode());
        printf("Min distance: %dm; Max distance: %dm; Timeout: %ld\n", presence.GetMinDistance(), presence.GetMaxDistance(), presence.GetTimeout());
        for(uint8_t i = 0; i < 16; ++i)
        {
            printf("Gate %d Thresholds: Move=%d Still=%d\n", i, presence.GetMoveThreshold(i), presence.GetStillThreshold(i));
        }

        g_FastQueue = xQueueCreate(10, sizeof(QueueMsg));
        g_ManagingQueue = xQueueCreate(10, sizeof(QueueMsg));

        std::thread ld2420_task(ld2420_managing_loop, std::ref(presence));
        ld2420_task.detach();

        configure_presence_isr_pin();

        return true;

    //printf("Restarting...\n");
    //if (auto e = presence.Restart(); !e)
    //{
    //    print_ld2420_error(e.error());
    //    return;
    //}

    //auto cfg = presence.ChangeConfiguration();
    //                                cfg.SetTimeout(5)
    //                                .SetSystemMode(LD2420::SystemMode::Energy)
    //                                .SetMinDistanceRaw(1)
    //                                .SetMaxDistanceRaw(12)
    //                                .SetMoveThreshold(0, 60000)
    //                                .SetStillThreshold(0, 40000)
    //                                .SetMoveThreshold(1, 30000)
    //                                .SetStillThreshold(1, 20000)
    //                                .SetMoveThreshold(2, 400)
    //                                .SetStillThreshold(2, 200)
    //                                .SetMoveThreshold(3, 300)
    //                                .SetStillThreshold(3, 250);
    //
    //for(uint8_t gate = 4; gate < 16; ++gate)
    //{
    //    cfg.SetMoveThreshold(gate, 250)
    //        .SetStillThreshold(gate, 150);
    //}
    //auto changeConfig = cfg.EndChange();
                                    //.SetMoveThreshold(4, 250)
                                    //.SetStillThreshold(4, 150)
                                    //.SetMoveThreshold(5, 250)
                                    //.SetStillThreshold(5, 150)
                                    //.SetMoveThreshold(6, 250)
                                    //.SetStillThreshold(6, 150)
                                //.EndChange();
    }
}
