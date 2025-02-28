#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ld2412_component.hpp"
#include "driver/gpio.h"
#include "lib_thread.hpp"

namespace ld2412
{
    struct Component::QueueMsg
    {
        enum class Type: uint8_t 
        {
            //commands
            Stop,
            Restart,
            FactoryReset,
            RunDynamicBackgroundAnalysis,
            RunDynamicBackgroundAnalysisDone,
            StartCalibrate,
            StopCalibrate,
            ResetEnergyStat,
            SwitchBluetooth,
            Flush,
            ReadData,
            //report
            Presence,
            PresenceIntr,
            PIRPresenceIntr,
            GatesEnergyState,
            //config
            SetTimeout,
            SetMinDistance,
            SetMaxDistance,
            SetMode,
            SetMoveSensitivity,
            SetStillSensitivity,
            SetDistanceRes,
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
            LD2412::DistanceRes m_DistRes;
            uint8_t m_Sensitivity[14];
            bool m_Bluetooth;
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

    void Component::presence_pir_pin_isr(void *param)
    {
        static int g_last = -1;
        Component &c = *static_cast<Component*>(param);
        int l = gpio_get_level(gpio_num_t(c.m_PIRPresencePin));
        if (l != g_last)
        {
            g_last = l;
            QueueMsg msg{.m_Type=QueueMsg::Type::PIRPresenceIntr, .m_Dummy=bool(l)};
            xQueueSendFromISR(c.m_FastQueue, &msg, nullptr);
        }
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
            case QueueMsg::Type::RunDynamicBackgroundAnalysis:
                if (!d.IsDynamicBackgroundAnalysisRunning())
                {
                    auto te = d.RunDynamicBackgroundAnalysis();
                    if (!te)
                    {
                        FMT_PRINT("Running dynamic background analysis has failed: {}\n", te.error());
                    }else
                    {
                        m_DynamicBackgroundAnalysis = true;
                        xQueueSend(m_FastQueue, &msg, portMAX_DELAY);
                    }
                }
                break;
            case QueueMsg::Type::ResetEnergyStat:
                {
                    for(auto &e : m_MeasuredMinMax)
                    {
                        e.move = {.min=0xffff, .max=0, .last = 0};
                        e.still = {.min=0xffff, .max=0, .last = 0};
                    }
                }
                break;
            case QueueMsg::Type::SwitchBluetooth:
                {
                    auto te = d.SwitchBluetooth(msg.m_Bluetooth);
                    if (!te)
                    {
                        FMT_PRINT("Switching bluetooth has failed: {}\n", te.error());
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
                        {
                            m_CalibrationStarted = true;
                            xQueueSend(m_FastQueue, &msg, portMAX_DELAY);
                        }
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

                        xQueueSend(m_FastQueue, &msg, portMAX_DELAY);
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
            case QueueMsg::Type::SetDistanceRes:
                {
                    FMT_PRINT("Changing distance resolution to: {}\n", msg.m_DistRes);
                    auto te = d.ChangeConfiguration()
                        .SetDistanceRes(msg.m_DistRes)
                        .EndChange();
                    if (!te)
                    {
                        FMT_PRINT("Setting mode has failed: {}\n", te.error());
                    } else if (m_ConfigUpdateCallback)
                            m_ConfigUpdateCallback();
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
            case QueueMsg::Type::SetMoveSensitivity:
                {
                    auto ch = d.ChangeConfiguration();
                    for(int i = 0; i < 14; ++i)
                        ch.SetMoveThreshold(i, msg.m_Sensitivity[i]);
                    auto te = ch.EndChange();
                    if (!te)
                    {
                        FMT_PRINT("Setting move sensitivity has failed: {}\n", te.error());
                    }
                    else if (m_ConfigUpdateCallback)
                        m_ConfigUpdateCallback();
                }
                break;
            case QueueMsg::Type::SetStillSensitivity:
                {
                    auto ch = d.ChangeConfiguration();
                    for(int i = 0; i < 14; ++i)
                        ch.SetStillThreshold(i, msg.m_Sensitivity[i]);
                    auto te = ch.EndChange();
                    if (!te)
                    {
                        FMT_PRINT("Setting still sensitivity has failed: {}\n", te.error());
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
        bool lastPIRPresence = false;
        bool lastCompositePresence = false;
        PresenceResult lastPresenceData;
        ExtendedState exState = ExtendedState::Normal;
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
                        //bool prev = lastCompositePresence;
                        lastCompositePresence = lastPresence || lastPIRPresence;
                        if (lastPresenceData.mmPresence != lastPresence)
                        {
                            lastPresenceData.mmPresence = lastPresence;
                            if (c.m_MovementCallback)
                                c.m_MovementCallback(lastCompositePresence, lastPresenceData, exState);
                        }
                    }
                    break;
                    case QueueMsg::Type::PIRPresenceIntr: 
                    {
                        int l = gpio_get_level(gpio_num_t(c.m_PIRPresencePin));
                        FMT_PRINT("Msg PIR presence interrupt: {}\n", l);
                        lastPIRPresence = l == 1;
                        //blink_led(lastPIRPresence);
                        lastCompositePresence = lastPresence || lastPIRPresence;
                        if (lastPresenceData.pirPresence != lastPIRPresence)
                        {
                            lastPresenceData.pirPresence = lastPIRPresence;
                            if (c.m_MovementCallback)
                                c.m_MovementCallback(lastCompositePresence, lastPresenceData, exState);
                        }
                    }
                    break;
                    case QueueMsg::Type::RunDynamicBackgroundAnalysis: 
                    {
                        exState = ExtendedState::RunningDynamicBackgroundAnalysis;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastCompositePresence, lastPresenceData, exState);
                    }
                    break;
                    case QueueMsg::Type::RunDynamicBackgroundAnalysisDone: 
                    {
                        exState = ExtendedState::Normal;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastCompositePresence, lastPresenceData, exState);
                    }
                    break;
                    case QueueMsg::Type::StartCalibrate: 
                    {
                        exState = ExtendedState::RunningCalibration;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastCompositePresence, lastPresenceData, exState);
                    }
                    break;
                    case QueueMsg::Type::StopCalibrate: 
                    {
                        exState = ExtendedState::Normal;
                        if (c.m_MovementCallback)
                            c.m_MovementCallback(lastCompositePresence, lastPresenceData, exState);
                    }
                    break;
                    case QueueMsg::Type::GatesEnergyState:
                    {
                        if (c.m_MeasurementsUpdateCallback)
                            c.m_MeasurementsUpdateCallback();
                    }
                    break;
                    case QueueMsg::Type::Presence: 
                        //FMT_PRINT("Msg presence: {:x}\n", msg.m_PresenceRaw.changedRaw);
                        //if (c.m_PresencePin != -1)
                        //    msg.m_Presence.m_ChangePresenceStill = msg.m_Presence.m_ChangePresenceMove = 0;
                        if (msg.m_Presence.changed())
                        {
                            //FMT_PRINT("Msg presence something changed\n");
                            if (c.m_PresencePin == -1)//if a dedicated pin is configured - it dictates presence
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
                            lastCompositePresence = lastPresence || lastPIRPresence;
                            lastPresenceData.mmPresence = lastPresence;

                            if (c.m_MovementCallback)
                                c.m_MovementCallback(lastCompositePresence, lastPresenceData, exState);
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
            xQueueSend(c.m_FastQueue, &msg, portMAX_DELAY);
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

                if (d.IsDynamicBackgroundAnalysisRunning())
                {
                    if (!c.m_DynamicBackgroundAnalysis)
                    {
                        c.m_DynamicBackgroundAnalysis = true;
                        msg.m_Type = QueueMsg::Type::RunDynamicBackgroundAnalysis;
                        xQueueSend(c.m_FastQueue, &msg, portMAX_DELAY);
                    }
                    continue;
                }else if (c.m_DynamicBackgroundAnalysis)
                {
                    c.m_DynamicBackgroundAnalysis = false;
                    msg.m_Type = QueueMsg::Type::RunDynamicBackgroundAnalysisDone;
                    xQueueSend(c.m_FastQueue, &msg, portMAX_DELAY);
                }

                bool simpleMode = d.GetSystemMode() == LD2412::SystemMode::Simple;
                auto te = d.TryReadFrame(3, true, LD2412::Drain::Try);

                if (!simpleMode)
                {
                    bool changed = false;
                    for(uint8_t g = 0; g < 14; ++g)
                    {
                        auto e = c.GetMeasuredMoveEnergy(g);
                        auto &move = c.m_MeasuredMinMax[g].move;
                        auto &still = c.m_MeasuredMinMax[g].still;
                        if (move.last != e)
                        {
                            move.last = e;
                            changed = true;
                        }
                        if (e > move.max) move.max = e;
                        if (e < move.min) move.min = e;
                        e = c.GetMeasuredStillEnergy(g);
                        if (still.last != e)
                        {
                            still.last = e;
                            changed = true;
                        }
                        if (e > still.max) still.max = e;
                        if (e < still.min) still.min = e;
                    }

                    if (c.m_MeasuredLight != d.GetMeasuredLight())
                    {
                        c.m_MeasuredLight = d.GetMeasuredLight();
                        changed = true;
                    }

                    if (changed)
                    {
                        msg.m_Type = QueueMsg::Type::GatesEnergyState;
                        xQueueSend(c.m_FastQueue, &msg, portMAX_DELAY);
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
            }else
            {
                if (d.IsDynamicBackgroundAnalysisRunning())
                {
                    if (!c.m_DynamicBackgroundAnalysis)
                    {
                        c.m_DynamicBackgroundAnalysis = true;
                        msg.m_Type = QueueMsg::Type::RunDynamicBackgroundAnalysis;
                        xQueueSend(c.m_FastQueue, &msg, portMAX_DELAY);
                    }
                }else if (c.m_DynamicBackgroundAnalysis)
                {
                    c.m_DynamicBackgroundAnalysis = false;
                    msg.m_Type = QueueMsg::Type::RunDynamicBackgroundAnalysisDone;
                    xQueueSend(c.m_FastQueue, &msg, portMAX_DELAY);
                }
            }
        }
    }

    void Component::ConfigurePresenceIsr()
    {
        if (m_PresencePin != -1)
        {
            gpio_config_t ld2412_presence_pin_cfg = {
                .pin_bit_mask = 1ULL << m_PresencePin,
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                .intr_type = GPIO_INTR_ANYEDGE,
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
                .hys_ctrl_mode = gpio_hys_ctrl_mode_t{}
#endif
            };

            gpio_config(&ld2412_presence_pin_cfg);
            gpio_isr_handler_add(gpio_num_t(m_PresencePin), presence_pin_isr, this);
        }
        if (m_PIRPresencePin != -1)
        {
            gpio_config_t ld2412_presence_pin_cfg = {
                .pin_bit_mask = 1ULL << m_PIRPresencePin,
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                .intr_type = GPIO_INTR_ANYEDGE,
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
                .hys_ctrl_mode = gpio_hys_ctrl_mode_t{}
#endif
            };

            gpio_config(&ld2412_presence_pin_cfg);
            gpio_isr_handler_add(gpio_num_t(m_PIRPresencePin), presence_pir_pin_isr, this);
        }
    }

    void Component::ChangeMode(LD2412::SystemMode m)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetMode, .m_Mode = m};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeDistanceRes(LD2412::DistanceRes r)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetDistanceRes, .m_DistRes = r};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeTimeout(uint16_t to)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetTimeout, .m_Timeout = to};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    void Component::ChangeMoveSensitivity(const uint8_t (&sensitivity)[14])
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetMoveSensitivity, .m_Dummy = 0};
        static_assert(sizeof(sensitivity) == sizeof(msg.m_Sensitivity));
        memcpy(msg.m_Sensitivity, sensitivity, sizeof(msg.m_Sensitivity));
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }
    void Component::ChangeStillSensitivity(const uint8_t (&sensitivity)[14])
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SetStillSensitivity, .m_Dummy = 0};
        static_assert(sizeof(sensitivity) == sizeof(msg.m_Sensitivity));
        memcpy(msg.m_Sensitivity, sensitivity, sizeof(msg.m_Sensitivity));
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

    void Component::SwitchBluetooth(bool on)
    {
        QueueMsg msg{.m_Type = QueueMsg::Type::SwitchBluetooth, .m_Bluetooth = on};
        xQueueSend(m_ManagingQueue, &msg, portMAX_DELAY);
    }

    LD2412::SystemMode Component::GetMode() const { return m_Sensor.GetSystemMode(); }
    LD2412::DistanceRes Component::GetDistanceRes() const { return m_Sensor.GetDistanceRes(); }
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
            if (!m_Sensor.HasEventCallback())
            {
                m_Sensor.SetEventCallback([this](uart_event_type_t e){
                        auto q = m_ManagingQueue.load(std::memory_order_relaxed);
                        if (!q)
                        return;//ignore, too early
                        switch (e) 
                        {
                        case UART_DATA:
                        {
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
            }

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
        m_PIRPresencePin = args.presencePIRPin;

        {
            printf("Config\n");
            auto changeConfig = m_Sensor.ChangeConfiguration()
                .SetSystemMode(args.mode)
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

        m_FastQueue = xQueueCreate(256, sizeof(QueueMsg));
        m_ManagingQueue.store(xQueueCreate(16, sizeof(QueueMsg)), std::memory_order_relaxed);
        {
            //enque reading data first
            QueueMsg msg{.m_Type = QueueMsg::Type::ReadData, .m_Dummy = true};
            xQueueSend(m_ManagingQueue.load(std::memory_order_relaxed), &msg, 0);
        }

        thread::start_task({.pName="LD2412_Manage", .stackSize = 4*4096, .prio=thread::kPrioElevated}, &manage_loop, this).detach();
        thread::start_task({.pName="LD2412_Fast", .stackSize = 8*4096, .prio=thread::kPrioHigh}, &fast_loop, this).detach();

        FMT_PRINT("ld2412 component: configuring isr\n");
        fflush(stdout);
        //configure_led();
        ConfigurePresenceIsr();

        m_Setup = true;
        FMT_PRINT("ld2412 component: setup done\n");
        fflush(stdout);
        if (m_ConfigUpdateCallback)
            m_ConfigUpdateCallback();

        {
            //initial read of the presence pins
            QueueMsg msg{.m_Type=QueueMsg::Type::PresenceIntr, .m_Dummy=false};
            if (m_PresencePin != -1)
            {
                xQueueSend(m_FastQueue, &msg, 0);
            }

            if (m_PIRPresencePin != -1)
            {
                msg.m_Type=QueueMsg::Type::PIRPresenceIntr;
                xQueueSendFromISR(m_FastQueue, &msg, 0);
            }
        }

        return true;
    }
}
