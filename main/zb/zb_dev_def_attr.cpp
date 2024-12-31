#include "zb_dev_def.hpp"
#include "esp_check.h"

namespace zb
{
    ZclAttributeOccupiedToUnoccupiedTimeout_t g_OccupiedToUnoccupiedTimeout;
    ZclAttributeOccupancy_t g_OccupancyState;

    /**********************************************************************/
    /* Attributes for external signal on/off server cluster               */
    /**********************************************************************/
    ZclAttributeExternalOnOff_t g_ExternalOnOff;
    /**********************************************************************/
    /* Attributes for a custom cluster                                    */
    /**********************************************************************/
    ZclAttributeLD2412MoveSensetivity_t                 g_LD2412MoveSensitivity;
    ZclAttributeLD2412StillSensetivity_t                g_LD2412StillSensitivity;
    ZclAttributeState_t                                 g_LD2412State;
    ZclAttributeExState_t                               g_LD2412ExState;
    ZclAttributeMaxDistance_t                           g_LD2412MaxDistance;
    ZclAttributeMinDistance_t                           g_LD2412MinDistance;
    ZclAttributeMode_t                                  g_LD2412Mode;
    ZclAttributeEngineeringLight_t                      g_LD2412EngineeringLight;
    ZclAttributePIRPresence_t                           g_LD2412PIRPresence;
    ZclAttributeOnOffCommandMode_t                      g_OnOffCommandMode;
    ZclAttributeOnOffCommandTimeout_t                   g_OnOffCommandTimeout;
    ZclAttributePresenceDetectionIlluminanceThreshold_t g_PresenceDetectionIlluminanceThreshold;
    ZclAttributePresenceEdgeDetectionMMWave_t           g_PresenceEdgeDetectionMMWave;
    ZclAttributePresenceEdgeDetectionPIRInternal_t      g_PresenceEdgeDetectionPIRInternal;
    ZclAttributePresenceEdgeDetectionExternal_t         g_PresenceEdgeDetectionExternal;
    ZclAttributePresenceKeepDetectionMMWave_t           g_PresenceKeepDetectionMMWave;
    ZclAttributePresenceKeepDetectionPIRInternal_t      g_PresenceKeepDetectionPIRInternal;
    ZclAttributePresenceKeepDetectionExternal_t         g_PresenceKeepDetectionExternal;
    ZclAttributeExternalOnTime_t                        g_ExternalOnTime;
    ZclAttributeFailureStatus_t                         g_FailureStatus;
    ZclAttributeInternals_t                             g_Internals;
    ZclAttributeRestartsCount_t                         g_RestartsCount;
    ZclAttributeIlluminanceExternal_t                   g_IlluminanceExternal;
    ZclAttributeInternals2_t                            g_Internals2;

#if defined(ENABLE_ENGINEERING_ATTRIBUTES)
    ZclAttributeStillDistance_t                         g_LD2412StillDistance;
    ZclAttributeMoveDistance_t                          g_LD2412MoveDistance;
    ZclAttributeStillEnergy_t                           g_LD2412StillEnergy;
    ZclAttributeMoveEnergy_t                            g_LD2412MoveEnergy;
    ZclAttributeEngineeringEnergyMove_t                 g_LD2412EngineeringEnergyMove;
    ZclAttributeEngineeringEnergyStill_t                g_LD2412EngineeringEnergyStill;
    ZclAttributeEngineeringEnergyMoveMin_t              g_LD2412EngineeringEnergyMoveMin;
    ZclAttributeEngineeringEnergyStillMin_t             g_LD2412EngineeringEnergyStillMin;
    ZclAttributeEngineeringEnergyMoveMax_t              g_LD2412EngineeringEnergyMoveMax;
    ZclAttributeEngineeringEnergyStillMax_t             g_LD2412EngineeringEnergyStillMax;
#endif

    static ZbAlarmExt16 g_DelayedAttrUpdate;
    static void update_presence_attr_only()
    {
        esp_zb_zcl_occupancy_sensing_occupancy_t val = g_State.m_LastPresence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
        if (auto status = g_OccupancyState.Set(val); !status)
        {
            FMT_PRINT("Failed to set occupancy attribute with error {:x}\n", (int)status.error());
        }
    }
    /**********************************************************************/
    /* Attributes                                                         */
    /**********************************************************************/
    static const SetAttributeHandler g_AttributeHandlers[] = {
        AttrDescr<ZclAttributeOccupiedToUnoccupiedTimeout_t, 
            [](auto const& to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing timeout to {}\n", to);
                g_ld2412.ChangeTimeout(to);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeExternalOnOff_t, 
            [](auto const& to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing external on/off state to {}\n", to);
                g_State.m_LastPresenceExternal = to;
                if (to)//use external timeout if provided
                {
                    auto ex_timeout = g_Config.GetExternalOnOffTimeout();
                    if (ex_timeout)
                        g_State.StartExternalTimer(&on_external_on_timer_finished, ex_timeout * 1000);
                    else
                        g_State.m_ExternalRunningTimer.Cancel();
                }

                if (update_presence_state())
                {
                    g_DelayedAttrUpdate.Setup([lp = g_State.m_LastPresence]{
                            if (lp == g_State.m_LastPresence)
                            {
                                (void)send_on_off(g_State.m_LastPresence);
                                update_presence_attr_only();
                            }
                    }, kExternalTriggerCmdDelay);
                }
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeExternalOnTime_t, 
            [](auto const& to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing external on time to {}\n", to);
                g_Config.SetExternalOnOffTimeout(to);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeLD2412MoveSensetivity_t, 
            [](SensitivityBufType const& to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing move sensitivity to {}\n", to.sv());
                g_ld2412.ChangeMoveSensitivity(to.data);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeLD2412StillSensetivity_t, 
            [](SensitivityBufType const& to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing still sensitivity to {}\n", to.sv());
                g_ld2412.ChangeStillSensitivity(to.data);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeMaxDistance_t, 
            [](const uint16_t &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing max distance to {}\n", to);
                g_ld2412.ChangeMaxDistance(to);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeMinDistance_t, 
            [](const uint16_t &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing min distance to {}\n", to);
                g_ld2412.ChangeMinDistance(to);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeMode_t, 
            [](const LD2412::SystemMode &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing system mode to {}\n", to);
                g_ld2412.ChangeMode(to);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeOnOffCommandMode_t, 
            [](const OnOffMode &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing on-off command mode to {}\n", to);
                g_Config.SetOnOffMode(to);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributeOnOffCommandTimeout_t, 
            [](const uint16_t &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing on-off command timeout to {}\n", to);
                g_Config.SetOnOffTimeout(to);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributePresenceDetectionIlluminanceThreshold_t, 
            [](const uint8_t &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing presence detection illuminance threshold to {}\n", to);
                g_Config.SetIlluminanceThreshold(to);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributePresenceEdgeDetectionMMWave_t, 
            [](const bool &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing presence edge detection mmWave to {}\n", to);
                auto c = g_Config.GetPresenceDetectionMode();
                c.m_Edge_mmWave = to;
                g_Config.SetPresenceDetectionMode(c);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributePresenceEdgeDetectionPIRInternal_t, 
            [](const bool &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing presence edge detection PIR Internal to {}\n", to);
                auto c = g_Config.GetPresenceDetectionMode();
                c.m_Edge_PIRInternal = to;
                g_Config.SetPresenceDetectionMode(c);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributePresenceEdgeDetectionExternal_t, 
            [](const bool &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing presence edge detection external to {}\n", to);
                auto c = g_Config.GetPresenceDetectionMode();
                c.m_Edge_External = to;
                g_Config.SetPresenceDetectionMode(c);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributePresenceKeepDetectionMMWave_t, 
            [](const bool &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing presence Keep detection mmWave to {}\n", to);
                auto c = g_Config.GetPresenceDetectionMode();
                c.m_Keep_mmWave = to;
                g_Config.SetPresenceDetectionMode(c);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributePresenceKeepDetectionPIRInternal_t, 
            [](const bool &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing presence Keep detection PIR Internal to {}\n", to);
                auto c = g_Config.GetPresenceDetectionMode();
                c.m_Keep_PIRInternal = to;
                g_Config.SetPresenceDetectionMode(c);
                return ESP_OK;
            }
        >{},
        AttrDescr<ZclAttributePresenceKeepDetectionExternal_t, 
            [](const bool &to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing presence Keep detection external to {}\n", to);
                auto c = g_Config.GetPresenceDetectionMode();
                c.m_Keep_External = to;
                g_Config.SetPresenceDetectionMode(c);
                return ESP_OK;
            }
        >{},

        {}//last one
    };

    const SetAttributesHandlingDesc g_AttributeHandlingDesc = {
        /*default*/[](const esp_zb_zcl_set_attr_value_message_t *message)->esp_err_t
        {
            esp_err_t ret = ESP_OK;

            ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
            ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                                message->info.status);
            ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d), data type(%d)", message->info.dst_endpoint, message->info.cluster,
                     message->attribute.id, message->attribute.data.size, message->attribute.data.type);
            return ret;
        }
        ,g_AttributeHandlers
    };

}
