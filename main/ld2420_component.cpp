#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>
#include "ld2420_component.hpp"
#include "driver/gpio.h"
#include <thread>

namespace ld2420
{
    struct Component::QueueMsg
    {
        enum class Type: std::size_t 
        {
            //commands
            Stop,
            Restart,
            StartCalibrate,
            StopCalibrate,
            //report
            Presence,
            PresenceIntr,
            Distance,
            PresenceAndDistance,
            //config
            SetTimeout,
            SetMinDistance,
            SetMaxDistance,
            SetMode,
        };

        Type m_Type;
        union
        {
            bool m_Presence;
            float m_Distance;
            struct{
                bool m_Presence;
                float m_Distance;
            }m_PresenceAndDistance;
            uint32_t m_Timeout;
            Mode m_Mode;
        };
    };

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

    Component::~Component()
    {
        gpio_isr_handler_remove(gpio_num_t(m_PresencePin));
    }

    void Component::presence_pin_isr(void *param)
    {
        Component &c = *static_cast<Component*>(param);
        QueueMsg msg{.m_Type=QueueMsg::Type::PresenceIntr, .m_Presence=false};
        xQueueSendFromISR(c.m_FastQueue, &msg, nullptr);
    }

    void Component::HandleMessage(QueueMsg &msg)
    {
        auto &d = m_Sensor;
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
                        .SetMinDistance(msg.m_Distance)
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
                        .SetMaxDistance(msg.m_Distance)
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

    void Component::fast_loop(Component &c)
    {
        printf("Entering level tracking loop\n");
        fflush(stdout);
        bool lastPresence = false;
        float lastDistance = 0;
        QueueMsg msg;
        while(true)
        {
            if (xQueueReceive(c.m_FastQueue, &msg, 10000 / portTICK_PERIOD_MS))
            {
                switch(msg.m_Type)
                {
                    case QueueMsg::Type::Stop: return;
                    case QueueMsg::Type::PresenceIntr: 
                    {
                        int l = gpio_get_level(gpio_num_t(c.m_PresencePin));
                        printf("Msg presence interrupt: %d\n", l);
                        lastPresence = l == 1;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastPresence, lastDistance);
                        //printf("Level changed to %d\n", l);
                        //fflush(stdout);
                    }
                    break;
                    case QueueMsg::Type::Presence: 
                        printf("Msg Presence\n");
                        if (lastPresence != msg.m_Presence)
                        {
                            lastPresence = msg.m_Presence;
                            if (c.m_MovementCallback)
                                c.m_MovementCallback(lastPresence, lastDistance);
                        }
                    break;
                    case QueueMsg::Type::Distance: 
                        printf("Msg Distance\n");
                        lastDistance = msg.m_Distance;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastPresence, lastDistance);
                    break;
                    case QueueMsg::Type::PresenceAndDistance: 
                        printf("Msg PresenceAndDistance\n");
                        lastDistance = msg.m_PresenceAndDistance.m_Distance;
                        lastPresence = msg.m_PresenceAndDistance.m_Presence;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastPresence, lastDistance);
                    break;
                    default:
                    //don't care
                    printf("Unprocessed message of type %d\n", (int)msg.m_Type);
                    break;
                }
            }
        }
    }

    void Component::manage_loop(Component &c)
    {
        bool initial = true;
        bool lastPresence;
        float lastDistance;
        auto &d = c.m_Sensor;
        QueueMsg msg;
        while(true)
        {
            if (xQueueReceive(c.m_ManagingQueue, &msg, 200 / portTICK_PERIOD_MS)) //process
                c.HandleMessage(msg);

            bool simpleMode = d.GetSystemMode() == LD2420::SystemMode::Simple;
            //printf("manage loop before read frame\n");
            auto te = d.TryReadFrame(3, true, LD2420::Drain::Try);
            //printf("manage loop after read frame\n");

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
                xQueueSend(c.m_FastQueue, &msg, portMAX_DELAY);
        }
    }

    void Component::ConfigurePresenceIsr()
    {
        if (m_PresencePin == -1)
            return;
        gpio_config_t ld2420_presence_pin_cfg = {
            .pin_bit_mask = 1ULL << m_PresencePin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };

        gpio_config(&ld2420_presence_pin_cfg);
        gpio_isr_handler_add(gpio_num_t(m_PresencePin), presence_pin_isr, this);
    }

    bool Component::Setup(setup_args_t const& args)
    {
        if (m_Setup)
            return false;

        printf("Init\n");
        auto e = m_Sensor.Init(args.txPin, args.rxPin) 
            | functional::and_then([&]{ return m_Sensor.ReloadConfig(); });

        if (!e)
        {
            print_ld2420_error(e.error());
            return false;
        }

        m_PresencePin = args.presencePin;

        printf("Config\n");
        auto changeConfig = m_Sensor.ChangeConfiguration()
                                .SetSystemMode(LD2420::SystemMode::Simple)
                                .SetTimeout(5)
                                .EndChange();
        if (!changeConfig)
        {
            print_ld2420_error(changeConfig.error());
            return false;
        }

        printf("Reload config\n");
        if (auto e = m_Sensor.ReloadConfig(); !e)
        {
            print_ld2420_error(e.error());
            return false;
        }

        printf("Version: %s\n", m_Sensor.GetVersion().data());
        printf("Current Mode: %d\n", (int)m_Sensor.GetSystemMode());
        printf("Min distance: %dm; Max distance: %dm; Timeout: %ld\n", m_Sensor.GetMinDistance(), m_Sensor.GetMaxDistance(), m_Sensor.GetTimeout());
        for(uint8_t i = 0; i < 16; ++i)
        {
            printf("Gate %d Thresholds: Move=%d Still=%d\n", i, m_Sensor.GetMoveThreshold(i), m_Sensor.GetStillThreshold(i));
        }

        m_FastQueue = xQueueCreate(10, sizeof(QueueMsg));
        m_ManagingQueue = xQueueCreate(10, sizeof(QueueMsg));

        m_ManagingTask = std::jthread(&manage_loop, std::ref(*this));
        m_FastTask = std::jthread(&fast_loop, std::ref(*this));

        ConfigurePresenceIsr();

        m_Setup = true;
        return true;
    }
}
