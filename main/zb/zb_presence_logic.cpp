#include "zb_dev_def.hpp"
#include "../colors_def.hpp"

namespace zb
{
    static ZbAlarmExt16 g_DelayedAttrUpdate;

    static void on_local_on_timer_finished(void* param)
    {
        //this runs in the context of zigbee task
        if (g_State.m_LastPresence)//presence still active. start another timer
        {
            if (g_Config.GetOnOffTimeout())//still valid timeout?
                g_State.m_RunningTimer.Setup(&on_local_on_timer_finished, nullptr, g_Config.GetOnOffTimeout() * 1000);
        }
        else
        {
#ifndef NDEBUG
            using clock_t = std::chrono::system_clock;
            auto now = clock_t::now();
            auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
            //no presence. send 'off' command, no timer reset
            FMT_PRINT("{} Sending off command on timer\n", _n);
#endif
            if (g_State.CanSendCommandsToBind())
                g_State.m_OffSender.Send();

            ZbAlarm::check_counter_of_death();
        }
    }

    bool send_on_off(bool on)
    {
        auto m = g_Config.GetOnOffMode();
        auto t = g_Config.GetOnOffTimeout();
        if (m == OnOffMode::Nothing)
        {
            //g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::ReturnOnNothing;
            return false;//nothing
        }
        if (on && (m == OnOffMode::OffOnly))
        {
            //g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::ReturnOnOffOnly;
            return false;//nothing
        }
        if (!on && (m == OnOffMode::OnOnly || m == OnOffMode::TimedOn || m == OnOffMode::TimedOnLocal))
        {
            //g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::ReturnOnOnTimedOnTimedLocal;
            return false;//nothing
        }

        if (!g_State.CanSendCommandsToBind())
        {
            //g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::ReturnOnNoBoundDevices;
            return false;//no bound devices with on/off cluster, no reason to send a command
        }

        g_State.m_RunningTimer.Cancel();
        if (m == OnOffMode::TimedOn)
        {
            //g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::SentTimedOn;
            FMT_PRINT("Sending timed on command to binded with timeout: {};\n", t);
            g_State.m_OnTimedSender.Send();
            return true;
        }
        else if (m == OnOffMode::TimedOnLocal)
        {
            //g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::SentTimedOnLocal;
#ifndef NDEBUG
            using clock_t = std::chrono::system_clock;
            auto now = clock_t::now();
            auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
            FMT_PRINT("{} Sending ON (timed-on-local) command to binded\n", _n);
#endif

            if (t)//makes sense only for non-0
            {
                //set/reset the local timer
                g_State.StartLocalTimer(&on_local_on_timer_finished, t * 1000);
            }
            g_State.m_OnSender.Send();
            return true;
        }
        else
        {
            FMT_PRINT("Sending command to binded: {};\n", (int)on);
            if (on)
            {
                //g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::SentOn;
                g_State.m_OnSender.Send();
                return true;
            }
            else
            {
                //g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::SentOff;
                g_State.m_OffSender.Send();
                return true;
            }
        }
        return false;
    }

    //returns 'true' if changed
    bool update_presence_state()
    {
        bool changed = false;
        auto cfg = g_Config.GetPresenceDetectionMode();
        //FMT_PRINT("Upd Presence state. First: {}, Trigger allowed: {}, last pres: {}\n", g_State.m_FirstRun, g_State.m_TriggerAllowed, g_State.m_LastPresence);

        if (g_State.m_FirstRun || g_State.m_TriggerAllowed || !g_State.m_LastPresence)
        {
            //edge detection
            bool trigger = 
                   (cfg.m_Edge_mmWave && g_State.m_LastPresenceMMWave)
                || (cfg.m_Edge_PIRInternal && g_State.m_LastPresencePIRInternal)
                || (cfg.m_Edge_External && g_State.m_LastPresenceExternal);

            if (trigger)
            {
                g_State.m_LastPresence = true;
                g_State.m_TriggerAllowed = false;
            }
            changed = trigger;
            g_State.m_FirstRun = false;
        }

        if (!g_State.m_FirstRun && g_State.m_LastPresence)
        {
            //keep detection
            g_State.m_LastPresence = 
                   (cfg.m_Keep_mmWave && g_State.m_LastPresenceMMWave)
                || (cfg.m_Keep_PIRInternal && g_State.m_LastPresencePIRInternal)
                || (cfg.m_Keep_External && g_State.m_LastPresenceExternal);

            if (!g_State.m_LastPresence)
            {
                changed = true;
                g_State.m_TriggerAllowed = true;
            }
        }

        if (changed)
        {
            FMT_PRINT("Presence update to {}\n", g_State.m_LastPresence);
            if (ZbAlarm::g_RunningOutOfHandles)
            {
                if (g_State.m_LastPresence)
                    ZbAlarm::deactivate_counter_of_death();
                else if (!g_State.m_RunningTimer.IsRunning())
                    ZbAlarm::check_counter_of_death();
            }
        }

        /**********************************************************************/
        /* Illuminance threshold logic                                        */
        /**********************************************************************/
        if (changed && g_State.m_LastPresence)//react only to the 'front', or 'clear->detected' event
        {
            //only if threshold is set to something below kMaxIlluminance we might have a change in the logic
            //otherwise - no effect
            if ((g_Config.GetIlluminanceThreshold() < LocalConfig::kMaxIlluminance) && (g_State.GetIlluminance() > g_Config.GetIlluminanceThreshold()))
                g_State.m_SuppressedByIllulminance = true;
            else
                g_State.m_SuppressedByIllulminance = false;
        }

        return changed;
    }

    static void update_on_movement_attr()
    {
        /**********************************************************************/
        /* Zigbee attributes update                                           */
        /**********************************************************************/
        update_zb_occupancy_attr();
        {
            if (auto status = g_LD2412State.Set(LD2412State(g_State.m_LastLD2412State)); !status)
            {
                FMT_PRINT("Failed to set state attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412ExState.Set(g_State.m_LastLD2412ExtendedState); !status)
            {
                FMT_PRINT("Failed to set extended state attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412PIRPresence.Set(g_State.m_LastPresencePIRInternal); !status)
            {
                FMT_PRINT("Failed to set PIR presence attribute with error {:x}\n", (int)status.error());
            }

#if defined(ENABLE_ENGINEERING_ATTRIBUTES)
            if (auto status = g_LD2412MoveDistance.Set(p.m_MoveDistance); !status)
            {
                FMT_PRINT("Failed to set move dist attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412StillDistance.Set(p.m_StillDistance); !status)
            {
                FMT_PRINT("Failed to set still dist attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412MoveEnergy.Set(p.m_MoveEnergy); !status)
            {
                FMT_PRINT("Failed to set move dist attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412StillEnergy.Set(p.m_StillEnergy); !status)
            {
                FMT_PRINT("Failed to set still dist attribute with error {:x}\n", (int)status.error());
            }
#endif
        }
    }

    static void on_movement_callback(bool _presence, ld2412::Component::PresenceResult const& p, ld2412::Component::ExtendedState exState)
    {
        APILock l;
        using clock_t = std::chrono::system_clock;
        if (p.pirPresence && !g_State.m_LastPresencePIRInternal)
        {
            //PIR off -> on
            if (!p.mmPresence)
            {
                g_State.m_FalsePIRProbe = true;
                g_State.m_LastPIRStartedTick = xTaskGetTickCount();
                g_State.m_LastPIRTimeMS = std::chrono::time_point_cast<std::chrono::milliseconds>(clock_t::now()).time_since_epoch().count();
            }
        }else if (!p.pirPresence && g_State.m_LastPresencePIRInternal)
        {
            //PIR on -> off
            if (g_State.m_FalsePIRProbe && !p.mmPresence)
            {
                g_State.m_FalsePIRProbe = false;
                ++g_State.m_Internals.m_PIRFalsePositives;
                g_State.m_Internals.m_LastFalsePIRTickDuration = xTaskGetTickCount() - g_State.m_LastPIRStartedTick;
                g_State.m_Internals.m_LastFalsePIRDuration = std::chrono::time_point_cast<std::chrono::milliseconds>(clock_t::now()).time_since_epoch().count()
                    -
                    g_State.m_LastPIRTimeMS;
            }
        }

        if (p.mmPresence && g_State.m_FalsePIRProbe)
            g_State.m_FalsePIRProbe = false;

        g_State.m_LastPresenceMMWave = p.mmPresence;
        g_State.m_LastPresencePIRInternal = p.pirPresence;
        g_State.m_LastLD2412State = p.m_State;
        g_State.m_LastLD2412ExtendedState = exState;

        bool presence_changed = update_presence_state();

        if (g_State.m_SuppressedByIllulminance)
            return;

        FMT_PRINT("Presence: {}; Data: {}\n", (int)g_State.m_LastPresence, p);
        if (presence_changed)
        {
            if (send_on_off(g_State.m_LastPresence))
            {
                FMT_PRINT("Delaying attribute on presence update\n");
                g_DelayedAttrUpdate.Setup(update_on_movement_attr, kDelayedAttrChangeTimeout);
                return;
            }
        }
        update_on_movement_attr();
    }

    static void on_measurements_callback()
    {
#if defined(ENABLE_ENGINEERING_ATTRIBUTES)
        EnergyBufType moveBuf, stillBuf;
        EnergyBufType moveMinBuf, stillMinBuf;
        EnergyBufType moveMaxBuf, stillMaxBuf;
        for(uint8_t i = 0; auto const& m : g_ld2412.GetMeasurements())
        {
            moveBuf.data[i] = m.move.last;
            stillBuf.data[i] = m.still.last;
            moveMinBuf.data[i] = m.move.min;
            stillMinBuf.data[i] = m.still.min;
            moveMaxBuf.data[i] = m.move.max;
            stillMaxBuf.data[i] = m.still.max;
            ++i;
        }

        //FMT_PRINT("Measurements update: stillMax {};\n", stillMaxBuf.sv());
        //FMT_PRINT("Measurements update: stillMin {};\n", stillMinBuf.sv());
        //FMT_PRINT("Measurements update: move {};\n", moveBuf.sv());
        {
            APILock l;
            if (auto status = g_LD2412EngineeringEnergyMove.Set(moveBuf); !status)
            {
                FMT_PRINT("Failed to set measured move energy attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412EngineeringEnergyStill.Set(stillBuf); !status)
            {
                FMT_PRINT("Failed to set measured still energy attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412EngineeringEnergyMoveMin.Set(moveMinBuf); !status)
            {
                FMT_PRINT("Failed to set measured min move energy attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412EngineeringEnergyStillMin.Set(stillMinBuf); !status)
            {
                FMT_PRINT("Failed to set measured min still energy attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412EngineeringEnergyMoveMax.Set(moveMaxBuf); !status)
            {
                FMT_PRINT("Failed to set measured max move energy attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412EngineeringEnergyStillMax.Set(stillMaxBuf); !status)
            {
                FMT_PRINT("Failed to set measured max still energy attribute with error {:x}\n", (int)status.error());
            }
        }
#endif
    }

    static void on_config_update_callback()
    {
        SensitivityBufType moveBuf, stillBuf;
        for(uint8_t i = 0; i < 14; ++i)
        {
            moveBuf.data[i] = g_ld2412.GetMoveThreshold(i);
            stillBuf.data[i] = g_ld2412.GetStillThreshold(i);
        }
        auto timeout = g_ld2412.GetTimeout();
        auto minDistance = g_ld2412.GetMinDistance();
        auto maxDistance = g_ld2412.GetMaxDistance();

        FMT_PRINT("Setting move sensitivity attribute with {}\n", moveBuf.sv());
        FMT_PRINT("Setting still sensitivity attribute with {}\n", stillBuf.sv());
        FMT_PRINT("Setting timeout attribute with {}\n", timeout);
        {
            APILock l;
            if (auto status = g_LD2412MoveSensitivity.Set(moveBuf); !status)
            {
                FMT_PRINT("Failed to set move sensitivity attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412StillSensitivity.Set(stillBuf); !status)
            {
                FMT_PRINT("Failed to set move sensitivity attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_OccupiedToUnoccupiedTimeout.Set(timeout); !status)
            {
                FMT_PRINT("Failed to set occupied to unoccupied timeout with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412MinDistance.Set(minDistance); !status)
            {
                FMT_PRINT("Failed to set min distance with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412MaxDistance.Set(maxDistance); !status)
            {
                FMT_PRINT("Failed to set max distance with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412Mode.Set(g_ld2412.GetMode()); !status)
            {
                FMT_PRINT("Failed to set initial system mode with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412DistanceRes.Set(g_ld2412.GetDistanceRes()); !status)
            {
                FMT_PRINT("Failed to set initial distance resolution with error {:x}\n", (int)status.error());
            }
        }

        g_Config.SetLD2412Mode(g_ld2412.GetMode());//save in the config
    }

    void setup_sensor()
    {
        g_ld2412.SetCallbackOnMovement(on_movement_callback);
        g_ld2412.SetCallbackOnMeasurementsUpdate(on_measurements_callback);
        g_ld2412.SetCallbackOnConfigUpdate(on_config_update_callback);

        //set initial state of certain attributes
        {
            APILock l;
            if (auto status = g_LD2412State.Set(LD2412State::Configuring); !status)
            {
                FMT_PRINT("Failed to set initial state with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412Mode.Set(g_ld2412.GetMode()); !status)
            {
                FMT_PRINT("Failed to set initial system mode with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412ExState.Set(ld2412::Component::ExtendedState::Normal); !status)
            {
                FMT_PRINT("Failed to set initial extended state with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412EngineeringLight.Set(0); !status)
            {
                FMT_PRINT("Failed to set initial measured light state with error {:x}\n", (int)status.error());
            }
            if (auto status = g_OnOffCommandMode.Set(g_Config.GetOnOffMode()); !status)
            {
                FMT_PRINT("Failed to set initial on-off mode {:x}\n", (int)status.error());
            }
            if (auto status = g_OnOffCommandTimeout.Set(g_Config.GetOnOffTimeout()); !status)
            {
                FMT_PRINT("Failed to set initial on-off timeout {:x}\n", (int)status.error());
            }
            if (auto status = g_PresenceDetectionIlluminanceThreshold.Set(g_Config.GetIlluminanceThreshold()); !status)
            {
                FMT_PRINT("Failed to set initial illuminance threshold {:x}\n", (int)status.error());
            }
            if (auto status = g_ExternalOnTime.Set(g_Config.GetExternalOnOffTimeout()); !status)
            {
                FMT_PRINT("Failed to set initial on time for external {:x}\n", (int)status.error());
            }
            if (auto status = g_FailureStatus.Set((uint16_t)g_State.m_LastFailedStatus); !status)
            {
                FMT_PRINT("Failed to set initial failure status {:x}\n", (int)status.error());
            }
            if (auto status = g_Internals.Set(g_State.m_Internals.GetVal()); !status)
            {
                FMT_PRINT("Failed to set initial internals {:x}\n", (int)status.error());
            }
            if (auto status = g_RestartsCount.Set(g_Config.GetRestarts()); !status)
            {
                FMT_PRINT("Failed to set initial internals {:x}\n", (int)status.error());
            }
            
            auto presenceDetectionMode = g_Config.GetPresenceDetectionMode();
            //FMT_PRINT("initial detection mode: edge: {} {} {}; keep: {} {} {}\n"
            //        , (bool)presenceDetectionMode.m_Edge_mmWave
            //        , (bool)presenceDetectionMode.m_Edge_PIRInternal
            //        , (bool)presenceDetectionMode.m_Edge_External
            //        , (bool)presenceDetectionMode.m_Keep_mmWave
            //        , (bool)presenceDetectionMode.m_Keep_PIRInternal
            //        , (bool)presenceDetectionMode.m_Keep_External
            //        );
            if (auto status = g_PresenceDetectionConfig.Set(presenceDetectionMode.m_Raw); !status)
            {
                FMT_PRINT("Failed to set initial detection config {:x}\n", (int)status.error());
            }
        }

        constexpr int kMaxTries = 3;
        for(int tries = 0; tries < kMaxTries; ++tries)
        {
            if (!g_ld2412.Setup(ld2412::Component::setup_args_t{
                        .txPin=LD2412_PINS_TX, 
                        .rxPin=LD2412_PINS_RX, 
                        .presencePin=LD2412_PINS_PRESENCE,
                        .presencePIRPin=LD2412_PINS_PIR_PRESENCE,
                        .mode=g_Config.GetLD2412Mode()
                        }))
            {
                printf("Failed to configure ld2412 (attempt %d)\n", tries);
                fflush(stdout);
                if (tries == (kMaxTries - 1))
                {
                    APILock l;
                    if (auto status = g_LD2412State.Set(LD2412State::Failed); !status)
                    {
                        FMT_PRINT("Failed to set initial state with error {:x}\n", (int)status.error());
                        led::blink(true, colors::kLD2412ConfigError);
                    }
                    return;
                }
            }else
            {
                break;
            }
        }
        ESP_LOGI(TAG, "Sensor setup done");
    }

}
