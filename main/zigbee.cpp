#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "zigbee.hpp"
#include "esp_zigbee_core.h"

#include "zb_helpers.hpp"

#include <thread>
#include <bitset>
#include "ld2412_component.hpp"

namespace zb
{
    enum class LD2412State: std::underlying_type_t<LD2412::TargetState>
    {
        Configuring = 0x80,
        Failed = 0x81,
    };
    struct SensitivityBufType: ZigbeeOctetBuf<14> { SensitivityBufType(){sz=14;} };
    struct EnergyBufType: ZigbeeOctetBuf<14> { EnergyBufType(){sz=14;} };


    static auto g_Manufacturer = ZbStr("Orlangur");
    static auto g_Model = ZbStr("P-NextGen");
    static uint8_t g_AppVersion = 1;
    static const char *TAG = "ESP_ZB_PRESENCE_SENSOR";

    static ld2412::Component g_ld2412;//THE presence sensor component
    static constexpr int LD2412_PINS_TX = 11;
    static constexpr int LD2412_PINS_RX = 10;
    static constexpr int LD2412_PINS_PRESENCE = 4;
    static constexpr int LD2412_PINS_PIR_PRESENCE = 5;
    constexpr uint8_t PRESENCE_EP = 1;
    static constexpr const uint16_t CLUSTER_ID_LD2412 = kManufactureSpecificCluster;

    /**********************************************************************/
    /* Custom attributes IDs                                              */
    /**********************************************************************/
    static constexpr const uint16_t LD2412_ATTRIB_MOVE_SENSITIVITY = 0;
    static constexpr const uint16_t LD2412_ATTRIB_STILL_SENSITIVITY = 1;
    static constexpr const uint16_t LD2412_ATTRIB_MOVE_ENERGY = 2;
    static constexpr const uint16_t LD2412_ATTRIB_STILL_ENERGY = 3;
    static constexpr const uint16_t LD2412_ATTRIB_MOVE_DISTANCE = 4;
    static constexpr const uint16_t LD2412_ATTRIB_STILL_DISTANCE = 5;
    static constexpr const uint16_t LD2412_ATTRIB_STATE = 6;
    static constexpr const uint16_t LD2412_ATTRIB_MIN_DISTANCE = 7;
    static constexpr const uint16_t LD2412_ATTRIB_MAX_DISTANCE = 8;
    static constexpr const uint16_t LD2412_ATTRIB_EX_STATE = 9;
    static constexpr const uint16_t LD2412_ATTRIB_MODE = 10;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_LIGHT = 11;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE = 12;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_STILL = 13;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MIN = 14;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MIN = 15;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MAX = 16;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MAX = 17;
    static constexpr const uint16_t LD2412_ATTRIB_PIR_PRESENCE = 18;

    /**********************************************************************/
    /* Commands IDs                                                       */
    /**********************************************************************/
    static constexpr const uint8_t LD2412_CMD_RESTART = 0;
    static constexpr const uint8_t LD2412_CMD_FACTORY_RESET = 1;
    static constexpr const uint8_t LD2412_CMD_RESET_ENERGY_STAT = 2;
    static constexpr const uint8_t LD2412_CMD_BLUETOOTH = 3;

    /**********************************************************************/
    /* Cluster type definitions                                           */
    /**********************************************************************/
    using LD2412OccupancyCluster_t = ZclServerCluster<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING>;
    using LD2412CustomCluster_t    = ZclServerCluster<PRESENCE_EP, CLUSTER_ID_LD2412>;

    /**********************************************************************/
    /* Attributes types for occupancy cluster                             */
    /**********************************************************************/
    using ZclAttributeOccupancy_t                   = LD2412OccupancyCluster_t::Attribute<ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, esp_zb_zcl_occupancy_sensing_occupancy_t>;
    using ZclAttributeOccupiedToUnoccupiedTimeout_t = LD2412OccupancyCluster_t::Attribute<ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY_ID , uint16_t>;

    /**********************************************************************/
    /* Attributes types for a custom cluster                              */
    /**********************************************************************/
    using ZclAttributeLD2412MoveSensetivity_t     = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MOVE_SENSITIVITY, SensitivityBufType>;
    using ZclAttributeLD2412StillSensetivity_t    = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STILL_SENSITIVITY, SensitivityBufType>;
    using ZclAttributeStillDistance_t             = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STILL_DISTANCE, uint16_t>;
    using ZclAttributeMoveDistance_t              = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MOVE_DISTANCE, uint16_t>;
    using ZclAttributeMoveEnergy_t                = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MOVE_ENERGY, uint8_t>;
    using ZclAttributeStillEnergy_t               = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STILL_ENERGY, uint8_t>;
    using ZclAttributeState_t                     = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STATE , LD2412State>;
    using ZclAttributeExState_t                   = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_EX_STATE, ld2412::Component::ExtendedState>;
    using ZclAttributeMaxDistance_t               = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MAX_DISTANCE, uint16_t>;
    using ZclAttributeMinDistance_t               = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MIN_DISTANCE, uint16_t>;
    using ZclAttributeMode_t                      = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MODE, LD2412::SystemMode>;
    using ZclAttributeEngineeringLight_t          = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_LIGHT, uint8_t>;
    using ZclAttributeEngineeringEnergyStill_t    = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMove_t     = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE, EnergyBufType>;
    using ZclAttributeEngineeringEnergyStillMin_t = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MIN, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMoveMin_t  = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MIN, EnergyBufType>;
    using ZclAttributeEngineeringEnergyStillMax_t = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MAX, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMoveMax_t  = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MAX, EnergyBufType>;
    using ZclAttributePIRPresence_t               = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_PIR_PRESENCE, bool>;

    /**********************************************************************/
    /* Attributes for occupancy cluster                                   */
    /**********************************************************************/
    static ZclAttributeOccupiedToUnoccupiedTimeout_t g_OccupiedToUnoccupiedTimeout;
    static ZclAttributeOccupancy_t g_OccupancyState;

    /**********************************************************************/
    /* Attributes for a custom cluster                                    */
    /**********************************************************************/
    static ZclAttributeLD2412MoveSensetivity_t     g_LD2412MoveSensitivity;
    static ZclAttributeLD2412StillSensetivity_t    g_LD2412StillSensitivity;
    static ZclAttributeStillDistance_t             g_LD2412StillDistance;
    static ZclAttributeMoveDistance_t              g_LD2412MoveDistance;
    static ZclAttributeStillEnergy_t               g_LD2412StillEnergy;
    static ZclAttributeMoveEnergy_t                g_LD2412MoveEnergy;
    static ZclAttributeState_t                     g_LD2412State;
    static ZclAttributeExState_t                   g_LD2412ExState;
    static ZclAttributeMaxDistance_t               g_LD2412MaxDistance;
    static ZclAttributeMinDistance_t               g_LD2412MinDistance;
    static ZclAttributeMode_t                      g_LD2412Mode;
    static ZclAttributeEngineeringLight_t          g_LD2412EngineeringLight;
    static ZclAttributeEngineeringEnergyMove_t     g_LD2412EngineeringEnergyMove;
    static ZclAttributeEngineeringEnergyStill_t    g_LD2412EngineeringEnergyStill;
    static ZclAttributeEngineeringEnergyMoveMin_t  g_LD2412EngineeringEnergyMoveMin;
    static ZclAttributeEngineeringEnergyStillMin_t g_LD2412EngineeringEnergyStillMin;
    static ZclAttributeEngineeringEnergyMoveMax_t  g_LD2412EngineeringEnergyMoveMax;
    static ZclAttributeEngineeringEnergyStillMax_t g_LD2412EngineeringEnergyStillMax;
    static ZclAttributePIRPresence_t               g_LD2412PIRPresence;

    //initialized at start
    esp_zb_ieee_addr_t g_CoordinatorIeee;

    static void send_on_off(bool on)
    {
        FMT_PRINT("Sending command to binded: {};\n", (int)on);
        esp_zb_zcl_on_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        if (on)
            cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_ON_ID;
        else
            cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;
        esp_zb_zcl_on_off_cmd_req(&cmd_req);
    }

    static void on_movement_callback(bool presence, ld2412::Component::PresenceResult const& p, ld2412::Component::ExtendedState exState)
    {
        static bool g_FirstRun = true;
        static bool g_LastPresence = false;
        bool presence_changed = !g_FirstRun && (g_LastPresence != presence);
        g_FirstRun = false;
        g_LastPresence = presence;
        esp_zb_zcl_occupancy_sensing_occupancy_t val = presence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
        {
            APILock l;
            if (auto status = g_OccupancyState.Set(val); !status)
            {
                FMT_PRINT("Failed to set occupancy attribute with error {:x}\n", (int)status.error());
            }
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
            if (auto status = g_LD2412State.Set(LD2412State(p.m_State)); !status)
            {
                FMT_PRINT("Failed to set state attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412ExState.Set(exState); !status)
            {
                FMT_PRINT("Failed to set extended state attribute with error {:x}\n", (int)status.error());
            }
            if (auto status = g_LD2412PIRPresence.Set(p.pirPresence); !status)
            {
                FMT_PRINT("Failed to set PIR presence attribute with error {:x}\n", (int)status.error());
            }
            if (presence_changed)
            {
                send_on_off(presence);
            }
        }
        FMT_PRINT("Presence: {}; Data: {}\n", (int)presence, p);
    }

    static void on_measurements_callback()
    {
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
            if (auto status = g_LD2412EngineeringLight.Set(g_ld2412.GetMeasuredLight()); !status)
            {
                FMT_PRINT("Failed to set measured light attribute with error {:x}\n", (int)status.error());
            }
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
        }
    }

    static void setup_sensor()
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
        }

        constexpr int kMaxTries = 3;
        for(int tries = 0; tries < kMaxTries; ++tries)
        {
            if (!g_ld2412.Setup(ld2412::Component::setup_args_t{
                        .txPin=LD2412_PINS_TX, 
                        .rxPin=LD2412_PINS_RX, 
                        .presencePin=LD2412_PINS_PRESENCE,
                        .presencePIRPin=LD2412_PINS_PIR_PRESENCE
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

    /**********************************************************************/
    /* Commands                                                           */
    /**********************************************************************/
    esp_err_t ld2412_cmd_restart()
    {
        FMT_PRINT("Restarting...\n");
        g_ld2412.Restart();
        return ESP_OK;
    }

    esp_err_t ld2412_cmd_factory_reset()
    {
        FMT_PRINT("Factory resetting...\n");
        g_ld2412.FactoryReset();
        return ESP_OK;
    }

    esp_err_t ld2412_cmd_reset_energy_stat()
    {
        FMT_PRINT("Resetting collected energy stat...\n");
        g_ld2412.ResetEnergyStatistics();
        return ESP_OK;
    }

    esp_err_t ld2412_cmd_switch_bluetooth(bool on)
    {
        FMT_PRINT("Switching bluetooth {}...\n", on);
        //g_ld2412.SwitchBluetooth(on);
        return ESP_OK;
    }

    static const ZbCmdHandler g_Commands[] = {
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_RESTART, &ld2412_cmd_restart>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_FACTORY_RESET, &ld2412_cmd_factory_reset>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_RESET_ENERGY_STAT, &ld2412_cmd_reset_energy_stat>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_BLUETOOTH, &ld2412_cmd_switch_bluetooth, bool>{},
        {} //last, terminating one
    };

    static const ZbCmdHandlingDesc g_CommandsDesc{
        /*default*/[](const esp_zb_zcl_custom_cluster_command_message_t *message)->esp_err_t
        {
            FMT_PRINT("Unknown command: ep {:x}; cluster {:x}; cmd {:x}; data size {:x}\n",
                    message->info.dst_endpoint,
                    message->info.cluster,
                    message->info.command.id,
                    message->data.size
                    );
            return ESP_OK;
        },
        g_Commands
    };


    /**********************************************************************/
    /* Attributes                                                         */
    /**********************************************************************/
    static const SetAttributeHandler g_AttributeHandlers[] = {
        AttrDescr<ZclAttributeOccupiedToUnoccupiedTimeout_t, 
            [](auto const& to, const auto *message)->esp_err_t
            {
                //FMT_PRINT("Changing timeout to. Attr type: {}\n", (int)message->attribute.data.type);
                //uint8_t *pData = (uint8_t *)message->attribute.data.value;
                //FMT_PRINT("Changing timeout to. b1={:x}; b2={:x}\n", pData[0], pData[1]);
                FMT_PRINT("Changing timeout to {}\n", to);
                g_ld2412.ChangeTimeout(to);
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

        {}//last one
    };
    static const SetAttributesHandlingDesc g_AttributeHandlingDesc = {
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

    /**********************************************************************/
    /* Registering ZigBee device with clusters and attributes             */
    /**********************************************************************/
    static void create_presence_config_custom_cluster(esp_zb_cluster_list_t *cluster_list)
    {
        esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID_LD2412);

        ESP_ERROR_CHECK(g_LD2412MoveSensitivity.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_LD2412StillSensitivity.AddToCluster(custom_cluster, Access::RW));

        ESP_ERROR_CHECK(g_LD2412MoveDistance.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412StillDistance.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412MoveEnergy.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412StillEnergy.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412State.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412MaxDistance.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_LD2412MinDistance.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_LD2412ExState.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412Mode.AddToCluster(custom_cluster, Access::RWP));
        ESP_ERROR_CHECK(g_LD2412PIRPresence.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412EngineeringLight.AddToCluster(custom_cluster, Access::Report));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyStill.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyMove.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyMoveMin.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyStillMin.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyMoveMax.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyStillMax.AddToCluster(custom_cluster, Access::Read));

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    }

    static void create_presence_ep(esp_zb_ep_list_t *ep_list, uint8_t ep_id)
    {

        /**********************************************************************/
        /* Boilerplate config for standard clusters: basic, identify          */
        /**********************************************************************/
        esp_zb_basic_cluster_cfg_t basic_cfg =                                                                                \
            {                                                                                       
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                          
                .power_source = 0x1,//mains                        
            };                                                                                      
        esp_zb_identify_cluster_cfg_t identify_cfg =                                                                             
            {                                                                                       
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,                   
            };                                                                                      
        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
        esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, g_Manufacturer));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, g_Model));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &g_AppVersion));

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

        /**********************************************************************/
        /* Occupancy cluster config                                           */
        /**********************************************************************/
        esp_zb_occupancy_sensing_cluster_cfg_s presence_cfg =                                                                            
            {                                                                                       
                /*uint8_t*/  .occupancy = 0,                                                               /*!<  Bit 0 specifies the sensed occupancy as follows: 1 = occupied, 0 = unoccupied. */
                /*uint32_t*/ .sensor_type = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC, /*!<  The attribute specifies the type of the occupancy sensor */
                /*uint8_t*/  .sensor_type_bitmap = uint8_t(1) << ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC /*!<  The attribute specifies the types of the occupancy sensor */
            };                                                                                      
        esp_zb_attribute_list_t *pOccupancyAttributes = esp_zb_occupancy_sensing_cluster_create(&presence_cfg);
        uint16_t delay = 10;
        ESP_ERROR_CHECK(esp_zb_occupancy_sensing_cluster_add_attr(pOccupancyAttributes, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY_ID, &delay));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, pOccupancyAttributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));


        /**********************************************************************/
        /* Custom cluster                                                     */
        /**********************************************************************/
        create_presence_config_custom_cluster(cluster_list);

        /**********************************************************************/
        /* Client on/off cluster for direct binding purposes                  */
        /**********************************************************************/
        esp_zb_on_off_cluster_cfg_t on_off_cfg{.on_off = false};
        esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

        /**********************************************************************/
        /* Endpoint configuration                                             */
        /**********************************************************************/
        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint = ep_id,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
    }


    /**********************************************************************/
    /* Common zigbee network handling                                     */
    /**********************************************************************/
    static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
    {
        ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                            TAG, "Failed to start Zigbee bdb commissioning");
    }

    extern "C" void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
    {
        uint32_t *p_sg_p     = signal_struct->p_app_signal;
        esp_err_t err_status = signal_struct->esp_err_status;
        esp_zb_app_signal_type_t sig_type = *(esp_zb_app_signal_type_t*)p_sg_p;
        switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                //async setup
                thread::start_task({.pName="LD2412_Setup", .stackSize = 2*4096}, &setup_sensor).detach();
                //setup_sensor();

                ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted");
                    esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_CoordinatorIeee);
                }
            } else {
                /* commissioning failed */
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

                esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_CoordinatorIeee);
            } else {
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
        }
    }


    /**********************************************************************/
    /* Zigbee Task Entry Point                                            */
    /**********************************************************************/
    void zigbee_main(void *)
    {
        ESP_LOGI(TAG, "ZB main");
        fflush(stdout);
        esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_CRITICAL, 0);
        {
            esp_zb_cfg_t zb_nwk_cfg = {                                                               
                .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       
                .install_code_policy = false,           
                .nwk_cfg = {
                    .zed_cfg = {                                        
                        .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16MIN,                         
                        .keep_alive = 3000,                            
                    }
                },                                                          
            };
            esp_zb_init(&zb_nwk_cfg);
        }
        ESP_LOGI(TAG, "ZB after init");
        fflush(stdout);

        //config clusters here
        esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
        create_presence_ep(ep_list, PRESENCE_EP);
        ESP_LOGI(TAG, "ZB created ep");
        fflush(stdout);

        /* Register the device */
        esp_zb_device_register(ep_list);
        //here we install our handler for attributes and commands
        esp_zb_core_action_handler_register(generic_zb_action_handler<&g_AttributeHandlingDesc, &g_CommandsDesc>);
        ESP_LOGI(TAG, "ZB registered device");
        fflush(stdout);

        ESP_LOGI(TAG, "ZB updated attribute reporting");
        fflush(stdout);

        esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
        ESP_LOGI(TAG, "ZB set channel masks");
        fflush(stdout);

        ESP_ERROR_CHECK(esp_zb_start(false));
        ESP_LOGI(TAG, "ZB started, looping...");
        esp_zb_stack_main_loop();
    }

    void setup()
    {
        esp_zb_platform_config_t config = {
            .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE, .radio_uart_config = {}},
            .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, .host_uart_config = {}},
        };
        ESP_ERROR_CHECK(nvs_flash_init());
        FMT_PRINT("nvs_flash_init done\n");
        ESP_ERROR_CHECK(esp_zb_platform_config(&config));
        FMT_PRINT("esp_zb_platform_config done\n");
        
        xTaskCreate(zigbee_main, "Zigbee_main", 2*4096, NULL, 5, NULL);
    }
}
