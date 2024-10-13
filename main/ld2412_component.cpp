#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>
#include "ld2412_component.hpp"
#include "driver/gpio.h"
#include "thread_helper.hpp"
#include <format>

namespace ld2412
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
            //config
            SetTimeout,
            SetMinDistance,
            SetMaxDistance,
            SetMode,
        };

        Type m_Type;
        union
        {
            uint8_t m_Dummy;
            struct{
                uint16_t m_DistanceStill;
                uint16_t m_DistanceMove;
                uint8_t m_EnergyStill;
                uint8_t m_EnergyMove;

                uint8_t m_PresenceStill: 1;
                uint8_t m_PresenceMove: 1;

                uint8_t m_ChangePresenceStill: 1;
                uint8_t m_ChangePresenceMove: 1;
                uint8_t m_ChangeDistanceStill: 1;
                uint8_t m_ChangeDistanceMove: 1;
                uint8_t m_ChangeEnergyStill: 1;
                uint8_t m_ChangeEnergyMove: 1;
                uint8_t m_Dummy;

                bool changed() const
                {
                    return m_ChangePresenceStill 
                        || m_ChangePresenceMove 
                        || m_ChangeEnergyMove 
                        || m_ChangeEnergyStill
                        || m_ChangeDistanceMove 
                        || m_ChangeDistanceStill;
                }
            }m_Presence;
            struct{
                uint16_t raw1;
                uint16_t raw2;
                uint8_t raw3;
                uint8_t raw4;
                uint8_t changedRaw;
                uint8_t dummy;
            }m_PresenceRaw;
            uint16_t m_Timeout;
            uint16_t m_Distance;
            LD2412::SystemMode m_Mode;
        };
    };

    Component::~Component()
    {
        gpio_isr_handler_remove(gpio_num_t(m_PresencePin));
    }

    void Component::presence_pin_isr(void *param)
    {
        Component &c = *static_cast<Component*>(param);
        QueueMsg msg{.m_Type=QueueMsg::Type::PresenceIntr, .m_Dummy=false};
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
                    {
                        FMT_PRINT("Factory resetting has failed: {}\n", te.error());
                    }
                    else if (m_ConfigUpdateCallback)
                        m_ConfigUpdateCallback();
                }
                break;
            case QueueMsg::Type::ResetEnergyStat:
                {
                    for(auto &e : m_MeasuredMinMax)
                    {
                        e.move = {.min=0xffff, .max=0};
                        e.still = {.min=0xffff, .max=0};
                    }
                }
                break;
            case QueueMsg::Type::StartCalibrate:
                {
                    if (!m_CalibrationStarted)
                    {
                        m_ModeBeforeCalibration = d.GetSystemMode();
                        auto te = d.ChangeConfiguration()
                            .SetSystemMode(LD2412::SystemMode::Energy)
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
                        for(uint8_t g = 0; g < 14; ++g)
                        {
                            uint8_t still = uint8_t(m_MeasuredMinMax[g].still.max * 11 / 10);
                            uint8_t move = uint8_t(m_MeasuredMinMax[g].move.max * 13 / 10);
                            FMT_PRINT("Gate {}: prev=[still:{}; move:{}]; new=[still:{}; move:{}];"
                                    " measured move=[min:{}; max:{}]"
                                    " measured still=[min:{}; max:{}]"
                                    "\n"
                                    , g, d.GetStillThreshold(g), d.GetMoveThreshold(g)
                                    , still, move
                                    , m_MeasuredMinMax[g].move.min, m_MeasuredMinMax[g].move.max
                                    , m_MeasuredMinMax[g].still.min, m_MeasuredMinMax[g].still.max
                                    );
                            cfg.SetStillThreshold(g, still)
                               .SetMoveThreshold(g, move);
                        }
                        auto te = cfg.SetSystemMode(m_ModeBeforeCalibration).EndChange();

                        if (!te)
                        {
                            FMT_PRINT("Applying calibration and setting mode has failed: {}\n", te.error());
                        }
                        else if (m_ConfigUpdateCallback)
                            m_ConfigUpdateCallback();
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
                    } else if (m_ConfigUpdateCallback)
                            m_ConfigUpdateCallback();
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
                    else if (m_ConfigUpdateCallback)
                        m_ConfigUpdateCallback();
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
                    else if (m_ConfigUpdateCallback)
                        m_ConfigUpdateCallback();
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
                    else if (m_ConfigUpdateCallback)
                        m_ConfigUpdateCallback();
                }
                break;
            default:
                //don't care
                //report
                break;
        }
    }

    void Component::fast_loop(Component *pC)
    {
        Component &c = *pC;
        bool lastPresence = false;
        LD2412::PresenceResult lastPresenceData;
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
                            c.m_MovementCallback(lastPresence, lastPresenceData);
                    }
                    break;
                    case QueueMsg::Type::Presence: 
                        //FMT_PRINT("Msg presence: {:x}\n", msg.m_PresenceRaw.changedRaw);
                        if (c.m_PresencePin != -1)
                            msg.m_Presence.m_ChangePresenceStill = msg.m_Presence.m_ChangePresenceMove = 0;
                        if (msg.m_Presence.changed())
                        {
                            //FMT_PRINT("Msg presence something changed\n");
                            lastPresence = msg.m_Presence.m_PresenceStill | msg.m_Presence.m_PresenceMove;
                            if (msg.m_Presence.m_PresenceStill && msg.m_Presence.m_PresenceMove)
                                lastPresenceData.m_State = LD2412::TargetState::MoveAndStill;
                            else if (msg.m_Presence.m_PresenceStill)
                                lastPresenceData.m_State = LD2412::TargetState::Still;
                            else if (msg.m_Presence.m_PresenceMove)
                                lastPresenceData.m_State = LD2412::TargetState::Move;
                            else
                                lastPresenceData.m_State = LD2412::TargetState::Clear;

                            lastPresenceData.m_StillDistance = msg.m_Presence.m_DistanceStill;
                            lastPresenceData.m_MoveDistance = msg.m_Presence.m_DistanceMove;
                            lastPresenceData.m_StillEnergy = msg.m_Presence.m_EnergyStill;
                            lastPresenceData.m_MoveEnergy = msg.m_Presence.m_EnergyMove;

                            if (c.m_MovementCallback)
                                c.m_MovementCallback(lastPresence, lastPresenceData);
                        }
                    break;
                    default:
                    //don't care
                    FMT_PRINT("Unprocessed message of type {}\n", (int)msg.m_Type);
                    break;
                }
            }
        }
    }

    void Component::manage_loop(Component *pC)
    {
        Component &c = *pC;
        bool initial = true;
        LD2412::PresenceResult lastPresence;
        auto &d = c.m_Sensor;
        QueueMsg msg;
        if ((c.m_PresencePin != -1) && (d.GetSystemMode() == LD2412::SystemMode::Simple))
        {
            //need to read initial state
            QueueMsg msg{.m_Type=QueueMsg::Type::PresenceIntr, .m_Dummy=false};
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

                bool simpleMode = d.GetSystemMode() == LD2412::SystemMode::Simple;
                auto te = d.TryReadFrame(3, true, LD2412::Drain::Try);

                if (!simpleMode)
                {
                    for(uint8_t g = 0; g < 14; ++g)
                    {
                        auto e = c.GetMeasuredMoveEnergy(g);
                        if (e > c.m_MeasuredMinMax[g].move.max)
                            c.m_MeasuredMinMax[g].move.max = e;
                        if (e < c.m_MeasuredMinMax[g].move.min)
                            c.m_MeasuredMinMax[g].move.min = e;
                        e = c.GetMeasuredStillEnergy(g);
                        if (e > c.m_MeasuredMinMax[g].still.max)
                            c.m_MeasuredMinMax[g].still.max = e;
                        if (e < c.m_MeasuredMinMax[g].still.min)
                            c.m_MeasuredMinMax[g].still.min = e;
                    }
                }

                auto p = d.GetPresence();
                msg.m_Type = QueueMsg::Type::Presence;
                msg.m_Presence.m_DistanceStill = p.m_StillDistance;
                msg.m_Presence.m_DistanceMove = p.m_MoveDistance;
                msg.m_Presence.m_EnergyMove = p.m_MoveEnergy;
                msg.m_Presence.m_EnergyStill = p.m_StillEnergy;
                msg.m_Presence.m_PresenceStill = p.m_State & LD2412::TargetState::Still;
                msg.m_Presence.m_PresenceMove = p.m_State & LD2412::TargetState::Move;
                msg.m_Presence.m_ChangePresenceStill = false;
                msg.m_Presence.m_ChangePresenceMove = false;
                msg.m_Presence.m_ChangeDistanceStill = false;
                msg.m_Presence.m_ChangeDistanceMove = false;
                msg.m_Presence.m_ChangeEnergyMove = false;
                msg.m_Presence.m_ChangeEnergyStill = false;
                if (!te)
                {
                    FMT_PRINT("Failed to read frame: {}\n", te.error());
                }else if (initial)
                {
                    lastPresence = p;
                    msg.m_Presence.m_ChangePresenceStill = true;
                    msg.m_Presence.m_ChangePresenceMove = true;
                    msg.m_Presence.m_ChangeDistanceStill = true;
                    msg.m_Presence.m_ChangeDistanceMove = true;
                    msg.m_Presence.m_ChangeEnergyMove = true;
                    msg.m_Presence.m_ChangeEnergyStill = true;
                    initial = false;
                }else
                {
                    msg.m_Presence.m_ChangePresenceStill = (lastPresence.m_State & LD2412::TargetState::Still) != (p.m_State & LD2412::TargetState::Still);
                    msg.m_Presence.m_ChangePresenceMove = (lastPresence.m_State & LD2412::TargetState::Move) != (p.m_State & LD2412::TargetState::Move);
                    lastPresence.m_State = p.m_State;

                    msg.m_Presence.m_ChangeDistanceStill = std::abs((int)lastPresence.m_StillDistance - (int)p.m_StillDistance) > kDistanceReportChangeThreshold;
                    if (msg.m_Presence.m_ChangeDistanceStill) lastPresence.m_StillDistance = p.m_StillDistance;

                    msg.m_Presence.m_ChangeDistanceMove = std::abs((int)lastPresence.m_MoveDistance - (int)p.m_MoveDistance) > kDistanceReportChangeThreshold;
                    if (msg.m_Presence.m_ChangeDistanceMove) lastPresence.m_MoveDistance = p.m_MoveDistance;

                    msg.m_Presence.m_ChangeEnergyMove = std::abs((int)lastPresence.m_MoveEnergy - (int)p.m_MoveEnergy) > kEnergyReportChangeThreshold;
                    if (msg.m_Presence.m_ChangeEnergyMove) lastPresence.m_MoveEnergy = p.m_MoveEnergy;

                    msg.m_Presence.m_ChangeEnergyStill = std::abs((int)lastPresence.m_StillEnergy - (int)p.m_StillEnergy) > kEnergyReportChangeThreshold;
                    if (msg.m_Presence.m_ChangeEnergyStill) lastPresence.m_StillEnergy = p.m_StillEnergy;
                }


                bool anythingChanged = 
                      msg.m_Presence.m_ChangePresenceStill 
                    | msg.m_Presence.m_ChangePresenceMove
                    | msg.m_Presence.m_ChangeEnergyStill 
                    | msg.m_Presence.m_ChangeEnergyMove
                    | msg.m_Presence.m_ChangeDistanceStill 
                    | msg.m_Presence.m_ChangeDistanceMove;
                
                if (anythingChanged)
                    xQueueSend(c.m_FastQueue, &msg, portMAX_DELAY);
            }
        }
    }

    void Component::ConfigurePresenceIsr()
    {
        if (m_PresencePin == -1)
            return;
        gpio_config_t ld2412_presence_pin_cfg = {
            .pin_bit_mask = 1ULL << m_PresencePin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };

        gpio_config(&ld2412_presence_pin_cfg);
        gpio_isr_handler_add(gpio_num_t(m_PresencePin), presence_pin_isr, this);
    }

    void Component::ChangeMode(LD2412::SystemMode m)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetMode, .m_Mode = m};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeTimeout(uint16_t to)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetTimeout, .m_Timeout = to};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeMinDistance(uint16_t d)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetMinDistance, .m_Distance = d};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeMaxDistance(uint16_t d)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetMaxDistance, .m_Distance = d};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::Restart()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::Restart, .m_Dummy = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::FactoryReset()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::FactoryReset, .m_Dummy = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::StartCalibration()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::ResetEnergyStat, .m_Dummy = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
        msg = {.m_Type = QueueMsg::Type::StartCalibrate, .m_Dummy = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::StopCalibration()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::StopCalibrate, .m_Dummy = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::ResetEnergyStatistics()
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::ResetEnergyStat, .m_Dummy = true};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    LD2412::SystemMode Component::GetMode() const { return m_Sensor.GetSystemMode(); }
    int Component::GetMinDistance() const { return m_Sensor.GetMinDistance(); }
    uint8_t Component::GetMinDistanceRaw() const { return m_Sensor.GetMinDistanceRaw(); }

    int Component::GetMaxDistance() const { return m_Sensor.GetMaxDistance(); }
    uint8_t Component::GetMaxDistanceRaw() const { return m_Sensor.GetMaxDistanceRaw(); }

    uint8_t Component::GetMoveThreshold(uint8_t gate) const { return m_Sensor.GetMoveThreshold(gate); }
    uint8_t Component::GetStillThreshold(uint8_t gate) const { return m_Sensor.GetStillThreshold(gate); }
    uint8_t Component::GetMeasuredMoveEnergy(uint8_t gate) const { return m_Sensor.GetMeasuredMoveEnergy(gate); }
    uint8_t Component::GetMeasuredStillEnergy(uint8_t gate) const { return m_Sensor.GetMeasuredStillEnergy(gate); }

    uint16_t Component::GetTimeout() const { return m_Sensor.GetTimeout(); }

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
                        QueueMsg msg{.m_Type = QueueMsg::Type::ReadData, .m_Dummy = true};
                        xQueueSend(q, &msg, 0);
                    }
                    break;
                    case UART_BUFFER_FULL:
                    case UART_FIFO_OVF:
                    {
                        FMT_PRINT("{}\n", e == UART_BUFFER_FULL ? "buffer full" : "fifo overflow");
                        QueueMsg msg{.m_Type = QueueMsg::Type::Flush, .m_Dummy = true};
                        xQueueSend(q, &msg, 0);
                    }
                    break;
                    default:
                    break;
                }
            });

            auto e = m_Sensor.Init(args.txPin, args.rxPin);
            if (!!e)
                e = m_Sensor.ReloadConfig();

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
                .SetSystemMode(LD2412::SystemMode::Simple)
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

        FMT_PRINT("Version: {}\n", m_Sensor.GetVersion());
        FMT_PRINT("Current Mode: {}\n", m_Sensor.GetSystemMode());
        FMT_PRINT("Min distance: {}m; Max distance: {}m; Timeout: {}s\n", m_Sensor.GetMinDistance(), m_Sensor.GetMaxDistance(), m_Sensor.GetTimeout());
        for(uint8_t i = 0; i < 14; ++i)
        {
            FMT_PRINT("Gate {} Thresholds: Move={} Still={}\n", i, m_Sensor.GetMoveThreshold(i), m_Sensor.GetStillThreshold(i));
        }

        m_FastQueue = xQueueCreate(10, sizeof(QueueMsg));
        m_ManagingQueue.store(xQueueCreate(10, sizeof(QueueMsg)), std::memory_order_relaxed);
        {
            //enque reading data first
            QueueMsg msg{.m_Type = QueueMsg::Type::ReadData, .m_Dummy = true};
            xQueueSend(m_ManagingQueue.load(std::memory_order_relaxed), &msg, 0);
        }

        thread::start_task({.pName="LD2412_Manage", .stackSize = 2*4096}, &manage_loop, this).detach();
        thread::start_task({.pName="LD2412_Fast", .stackSize = 4096}, &fast_loop, this).detach();

        FMT_PRINT("ld2412 component: configuring isr\n");
        fflush(stdout);
        ConfigurePresenceIsr();

        m_Setup = true;
        FMT_PRINT("ld2412 component: setup done\n");
        fflush(stdout);
        if (m_ConfigUpdateCallback)
            m_ConfigUpdateCallback();
        return true;
    }
}
