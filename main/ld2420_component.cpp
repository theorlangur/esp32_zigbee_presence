#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>
#include "ld2420_component.hpp"
#include "driver/gpio.h"
#include <thread>
#include <format>

namespace ld2420
{
    struct Component::QueueMsg
    {
        enum class Type: std::size_t 
        {
            //commands
            Stop,
            Restart,
            FactoryReset,
            StartCalibrate,
            StopCalibrate,
            ResetEnergyStat,
            Flush,
            ReadData,
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
            LD2420::SystemMode m_Mode;
        };
    };

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
            case QueueMsg::Type::Flush:
                {
                    if (auto te = d.Flush(); !te)
                        FMT_PRINT("Flushing has failed: {}\n", te.error());
                }
                break;
            case QueueMsg::Type::Restart:
                {
                    auto te = d.Restart();
                    if (!te)
                        FMT_PRINT("Restarting request has failed: {}\n", te.error());
                }
                break;
            case QueueMsg::Type::FactoryReset:
                {
                    auto te = d.FactoryReset();
                    if (!te)
                        FMT_PRINT("Factory resetting has failed: {}\n", te.error());
                }
                break;
            case QueueMsg::Type::ResetEnergyStat:
                {
                    for(auto &e : m_MeasuredMinMax)
                    {
                        e.min = 0xffff;
                        e.max = 0;
                    }
                }
                break;
            case QueueMsg::Type::StartCalibrate:
                {
                    if (!m_CalibrationStarted)
                    {
                        m_ModeBeforeCalibration = d.GetSystemMode();
                        auto te = d.ChangeConfiguration()
                            .SetSystemMode(LD2420::SystemMode::Energy)
                            .EndChange();
                        if (!te)
                        {
                            FMT_PRINT("Setting mode to energy for calibration has failed: {}\n", te.error());
                        }else
                            m_CalibrationStarted = true;
                    }else
                    {
                        FMT_PRINT("Calibration is already running\n");
                    }
                }
                break;
            case QueueMsg::Type::StopCalibrate:
                {
                    if (m_CalibrationStarted)
                    {
                        FMT_PRINT("Applying calibration...\n");
                        m_CalibrationStarted = false;
                        auto cfg = d.ChangeConfiguration();
                        for(uint8_t g = 0; g < 16; ++g)
                        {
                            uint32_t still = m_MeasuredMinMax[g].max * 11 / 10;
                            uint32_t move = m_MeasuredMinMax[g].max * 15 / 10;
                            FMT_PRINT("Gate {}: prev=[still:{}; move:{}]; new=[still:{}; move:{}]; measured=[min:{}; max:{}]\n"
                                    , g, d.GetStillThreshold(g), d.GetMoveThreshold(g)
                                    , still, move
                                    , m_MeasuredMinMax[g].min, m_MeasuredMinMax[g].max);
                            cfg.SetStillThreshold(g, still)
                               .SetMoveThreshold(g, move);
                        }
                        auto te = cfg.SetSystemMode(m_ModeBeforeCalibration).EndChange();

                        if (!te)
                        {
                            FMT_PRINT("Applying calibration and setting mode has failed: {}\n", te.error());
                        }
                    }else
                    {
                        FMT_PRINT("Calibration was not running. Nothing to stop\n");
                    }
                }
                break;
            case QueueMsg::Type::SetMode:
                {
                    FMT_PRINT("Changing mode to: {}\n", msg.m_Mode);
                    auto te = d.ChangeConfiguration()
                        .SetSystemMode(msg.m_Mode)
                        .EndChange();
                    if (!te)
                    {
                        FMT_PRINT("Setting mode has failed: {}\n", te.error());
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
                        FMT_PRINT("Setting timeout has failed: {}\n", te.error());
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
                        FMT_PRINT("Setting min distance has failed: {}\n", te.error());
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
                        FMT_PRINT("Setting max distance has failed: {}\n", te.error());
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
                        FMT_PRINT("Msg presence interrupt: {}\n", l);
                        lastPresence = l == 1;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastPresence, lastDistance);
                    }
                    break;
                    case QueueMsg::Type::Presence: 
                        FMT_PRINT("Msg presence\n");
                        if (lastPresence != msg.m_Presence)
                        {
                            lastPresence = msg.m_Presence;
                            if (c.m_MovementCallback)
                                c.m_MovementCallback(lastPresence, lastDistance);
                        }
                    break;
                    case QueueMsg::Type::Distance: 
                        FMT_PRINT("Msg Distance\n");
                        lastDistance = msg.m_Distance;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastPresence, lastDistance);
                    break;
                    case QueueMsg::Type::PresenceAndDistance: 
                        FMT_PRINT("Msg PresenceAndDistance\n");
                        lastDistance = msg.m_PresenceAndDistance.m_Distance;
                        lastPresence = msg.m_PresenceAndDistance.m_Presence;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastPresence, lastDistance);
                    break;
                    default:
                    //don't care
                    FMT_PRINT("Unprocessed message of type {}\n", (int)msg.m_Type);
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
        if ((c.m_PresencePin != -1) && (d.GetSystemMode() == LD2420::SystemMode::Simple))
        {
            //need to read initial state
            QueueMsg msg{.m_Type=QueueMsg::Type::PresenceIntr, .m_Presence=false};
            xQueueSendFromISR(c.m_FastQueue, &msg, nullptr);
        }

        while(true)
        {
            if (xQueueReceive(c.m_ManagingQueue, &msg, 200 / portTICK_PERIOD_MS)) //process
            {
                if (msg.m_Type != QueueMsg::Type::ReadData)
                {
                    c.HandleMessage(msg);
                    continue;
                }

                bool simpleMode = d.GetSystemMode() == LD2420::SystemMode::Simple;
                auto te = d.TryReadFrame(3, true, LD2420::Drain::Try);

                if (!simpleMode)
                {
                    for(uint8_t g = 0; g < 16; ++g)
                    {
                        auto e = c.GetMeasuredEnergy(g);
                        if (e > c.m_MeasuredMinMax[g].max)
                            c.m_MeasuredMinMax[g].max = e;
                        if (e < c.m_MeasuredMinMax[g].min)
                            c.m_MeasuredMinMax[g].min = e;
                    }
                }

                bool reportPresence = false;
                bool reportDistance = false;
                auto p = d.GetPresence();
                if (!te)
                {
                    FMT_PRINT("Failed to read frame: {}\n", te.error());
                }else if (initial)
                {
                    reportPresence = !simpleMode || (c.m_PresencePin == -1);//in simple mode the reporting is via interrupt
                    reportDistance = true;
                    initial = false;
                    lastPresence = p.m_Detected;
                    lastDistance = p.m_Distance;
                }else if (lastPresence != p.m_Detected)
                {
                    lastPresence = p.m_Detected;
                    lastDistance = p.m_Distance;
                    reportDistance = true;
                    reportPresence = !simpleMode || (c.m_PresencePin == -1);//in simple mode the reporting is via interrupt
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

    void Component::ChangeMode(LD2420::SystemMode m)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetMode, .m_Mode = m};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeTimeout(uint32_t to)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetTimeout, .m_Timeout = to};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeMinDistance(float d)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetMinDistance, .m_Distance = d};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeMaxDistance(float d)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetMaxDistance, .m_Distance = d};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::Restart()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::Restart, .m_Presence = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::FactoryReset()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::FactoryReset, .m_Presence = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::StartCalibration()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::ResetEnergyStat, .m_Presence = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
        msg = {.m_Type = QueueMsg::Type::StartCalibrate, .m_Presence = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::StopCalibration()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::StopCalibrate, .m_Presence = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::ResetEnergyStatistics()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::ResetEnergyStat, .m_Presence = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    LD2420::SystemMode Component::GetMode() const { return m_Sensor.GetSystemMode(); }
    int Component::GetMinDistance() const { return m_Sensor.GetMinDistance(); }
    uint32_t Component::GetMinDistanceRaw() const { return m_Sensor.GetMinDistanceRaw(); }

    int Component::GetMaxDistance() const { return m_Sensor.GetMaxDistance(); }
    uint32_t Component::GetMaxDistanceRaw() const { return m_Sensor.GetMaxDistanceRaw(); }

    uint32_t Component::GetMoveThreshold(uint8_t gate) const { return m_Sensor.GetMoveThreshold(gate); }
    uint32_t Component::GetStillThreshold(uint8_t gate) const { return m_Sensor.GetStillThreshold(gate); }
    uint16_t Component::GetMeasuredEnergy(uint8_t gate) const { return m_Sensor.GetMeasuredEnergy(gate); }

    uint32_t Component::GetTimeout() const { return m_Sensor.GetTimeout(); }

    bool Component::Setup(setup_args_t const& args)
    {
        if (m_Setup)
            return false;

        {
            printf("Init\n");
            m_Sensor.SetEventCallback([this](uart_event_type_t e){
                auto q = m_ManagingQueue.load(std::memory_order_relaxed);
                if (!q)
                    return;//ignore, too early
                switch (e) 
                {
                    case UART_DATA:
                    {
                        //using clock_t = std::chrono::system_clock;
                        //using time_point_t = std::chrono::time_point<clock_t>;
                        //static time_point_t prev = clock_t::now();
                        //auto now = clock_t::now();
                        //FMT_PRINT("Data available: {}; Ellapsed since prev: {}ms\n"
                        //        , m_Sensor.GetReadyToReadDataLen().value().v
                        //        , std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count());
                        //prev = now;
                        QueueMsg msg{.m_Type = QueueMsg::Type::ReadData, .m_Presence = true};
                        xQueueSend(q, &msg, 0);
                    }
                    break;
                    case UART_BUFFER_FULL:
                    case UART_FIFO_OVF:
                    {
                        FMT_PRINT("{}\n", e == UART_BUFFER_FULL ? "buffer full" : "fifo overflow");
                        QueueMsg msg{.m_Type = QueueMsg::Type::Flush, .m_Presence = true};
                        xQueueSend(q, &msg, 0);
                    }
                    break;
                    default:
                    break;
                }
            });
            auto e = m_Sensor.Init(args.txPin, args.rxPin) 
                | functional::and_then([&]{ return m_Sensor.ReloadConfig(); });

            if (!e)
            {
                FMT_PRINT("Setup failed to init and reload config: {}\n", e.error());
                return false;
            }
        }

        m_PresencePin = args.presencePin;

        {
            printf("Config\n");
            auto changeConfig = m_Sensor.ChangeConfiguration()
                .SetSystemMode(LD2420::SystemMode::Simple)
                .SetTimeout(5)
                .EndChange();
            if (!changeConfig)
            {
                FMT_PRINT("Setup failed to set system mode to simple and a timeout: {}\n", changeConfig.error());
                return false;
            }
        }

        {
            printf("Reload config\n");
            if (auto e = m_Sensor.ReloadConfig(); !e)
            {
                FMT_PRINT("Setup failed to reload config: {}\n", e.error());
                return false;
            }
        }

        printf("Version: %s\n", m_Sensor.GetVersion().data());
        printf("Current Mode: %d\n", (int)m_Sensor.GetSystemMode());
        printf("Min distance: %dm; Max distance: %dm; Timeout: %ld\n", m_Sensor.GetMinDistance(), m_Sensor.GetMaxDistance(), m_Sensor.GetTimeout());
        for(uint8_t i = 0; i < 16; ++i)
        {
            printf("Gate %d Thresholds: Move=%d Still=%d\n", i, m_Sensor.GetMoveThreshold(i), m_Sensor.GetStillThreshold(i));
        }

        m_FastQueue = xQueueCreate(10, sizeof(QueueMsg));
        m_ManagingQueue.store(xQueueCreate(10, sizeof(QueueMsg)), std::memory_order_relaxed);
        {
            //enque reading data first
            QueueMsg msg{.m_Type = QueueMsg::Type::ReadData, .m_Presence = true};
            xQueueSend(m_ManagingQueue.load(std::memory_order_relaxed), &msg, 0);
        }

        m_ManagingTask = std::jthread(&manage_loop, std::ref(*this));
        m_FastTask = std::jthread(&fast_loop, std::ref(*this));

        ConfigurePresenceIsr();

        m_Setup = true;
        return true;
    }
}
