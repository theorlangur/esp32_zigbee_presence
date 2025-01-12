#ifndef ZB_DEV_DEF_ATTR_HPP_
#define ZB_DEV_DEF_ATTR_HPP_

#include "zb_dev_def_const.hpp"
#include "../device_common.hpp"
#include "../periph/ld2412_component.hpp"

namespace zb
{
    enum class LD2412State: std::underlying_type_t<LD2412::TargetState>
    {
        Configuring = 0x80,
        Failed = 0x81,
    };
    struct SensitivityBufType: ZigbeeOctetBuf<14> { SensitivityBufType(){sz=14;} };
    struct EnergyBufType: ZigbeeOctetBuf<14> { EnergyBufType(){sz=14;} };

    /**********************************************************************/
    /* Custom attributes IDs                                              */
    /**********************************************************************/
    static constexpr const uint16_t LD2412_ATTRIB_MOVE_SENSITIVITY = 0;
    static constexpr const uint16_t LD2412_ATTRIB_STILL_SENSITIVITY = 1;
#if defined(ENABLE_ENGINEERING_ATTRIBUTES)
    static constexpr const uint16_t LD2412_ATTRIB_MOVE_ENERGY = 2;
    static constexpr const uint16_t LD2412_ATTRIB_STILL_ENERGY = 3;
    static constexpr const uint16_t LD2412_ATTRIB_MOVE_DISTANCE = 4;
    static constexpr const uint16_t LD2412_ATTRIB_STILL_DISTANCE = 5;
#endif
    static constexpr const uint16_t LD2412_ATTRIB_STATE = 6;
    static constexpr const uint16_t LD2412_ATTRIB_MIN_DISTANCE = 7;
    static constexpr const uint16_t LD2412_ATTRIB_MAX_DISTANCE = 8;
    static constexpr const uint16_t LD2412_ATTRIB_EX_STATE = 9;
    static constexpr const uint16_t LD2412_ATTRIB_MODE = 10;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_LIGHT = 11;
#if defined(ENABLE_ENGINEERING_ATTRIBUTES)
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE = 12;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_STILL = 13;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MIN = 14;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MIN = 15;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MAX = 16;
    static constexpr const uint16_t LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MAX = 17;
#endif
    static constexpr const uint16_t LD2412_ATTRIB_PIR_PRESENCE = 18;
    static constexpr const uint16_t ON_OFF_COMMAND_MODE = 19;
    static constexpr const uint16_t ON_OFF_COMMAND_TIMEOUT = 20;
    static constexpr const uint16_t PRESENCE_DETECTION_ILLUMINANCE_THRESHOLD = 21;
    static constexpr const uint16_t ATTRIB_PRESENCE_DETECTION_CONFIG = 22;
    static constexpr const uint16_t ATTRIB_ARMED_FOR_TRIGGER = 23;
    static constexpr const uint16_t LD2412_ATTRIB_DISTANCE_RESOLUTION = 24;
    static constexpr const uint16_t ATTRIB_INTERNALS3 = 25;
    static constexpr const uint16_t EXTERNAL_ON_TIME = 28;
    static constexpr const uint16_t ATTRIB_FAILURE_STATUS = 29;
    static constexpr const uint16_t ATTRIB_INTERNALS = 30;
    static constexpr const uint16_t ATTRIB_RESTARTS_COUNT = 31;
    static constexpr const uint16_t ATTRIB_INTERNALS2 = 33;

    /**********************************************************************/
    /* Cluster type definitions                                           */
    /**********************************************************************/
    using LD2412OccupancyCluster_t = ZclServerCluster<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING>;
    using LD2412CustomCluster_t    = ZclServerCluster<PRESENCE_EP, CLUSTER_ID_LD2412>;
    using ExternalOnOffCluster_t   = ZclServerCluster<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF>;

    /**********************************************************************/
    /* Attributes types for occupancy cluster                             */
    /**********************************************************************/
    using ZclAttributeOccupancy_t                   = LD2412OccupancyCluster_t::Attribute<ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, esp_zb_zcl_occupancy_sensing_occupancy_t>;
    using ZclAttributeOccupiedToUnoccupiedTimeout_t = LD2412OccupancyCluster_t::Attribute<ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY_ID , uint16_t>;

    /**********************************************************************/
    /* Attributes types for on/off server cluster                         */
    /**********************************************************************/
    using ZclAttributeExternalOnOff_t = ExternalOnOffCluster_t::Attribute<ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID , bool>;

    /**********************************************************************/
    /* Attributes types for a custom cluster                              */
    /**********************************************************************/
    using ZclAttributeLD2412MoveSensetivity_t                 = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MOVE_SENSITIVITY, SensitivityBufType>;
    using ZclAttributeLD2412StillSensetivity_t                = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STILL_SENSITIVITY, SensitivityBufType>;
    using ZclAttributeState_t                                 = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STATE , LD2412State>;
    using ZclAttributeExState_t                               = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_EX_STATE, ld2412::Component::ExtendedState>;
    using ZclAttributeMaxDistance_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MAX_DISTANCE, uint16_t>;
    using ZclAttributeMinDistance_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MIN_DISTANCE, uint16_t>;
    using ZclAttributeMode_t                                  = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MODE, LD2412::SystemMode>;
    using ZclAttributeEngineeringLight_t                      = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_LIGHT, uint8_t>;
    using ZclAttributePIRPresence_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_PIR_PRESENCE, bool>;
    using ZclAttributeDistanceRes_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_DISTANCE_RESOLUTION, LD2412::DistanceRes>;
    using ZclAttributeOnOffCommandMode_t                      = LD2412CustomCluster_t::Attribute<ON_OFF_COMMAND_MODE, OnOffMode>;
    using ZclAttributeOnOffCommandTimeout_t                   = LD2412CustomCluster_t::Attribute<ON_OFF_COMMAND_TIMEOUT, uint16_t>;
    using ZclAttributePresenceDetectionIlluminanceThreshold_t = LD2412CustomCluster_t::Attribute<PRESENCE_DETECTION_ILLUMINANCE_THRESHOLD, uint8_t>;
    using ZclAttributePresenceDetectionConfig_t               = LD2412CustomCluster_t::Attribute<ATTRIB_PRESENCE_DETECTION_CONFIG, uint8_t>;
    using ZclAttributeExternalOnTime_t                        = LD2412CustomCluster_t::Attribute<EXTERNAL_ON_TIME , uint16_t>;
    using ZclAttributeFailureStatus_t                         = LD2412CustomCluster_t::Attribute<ATTRIB_FAILURE_STATUS, uint16_t>;
    using ZclAttributeInternals_t                             = LD2412CustomCluster_t::Attribute<ATTRIB_INTERNALS , uint32_t>;
    using ZclAttributeRestartsCount_t                         = LD2412CustomCluster_t::Attribute<ATTRIB_RESTARTS_COUNT , uint16_t>;
    using ZclAttributeInternals2_t                            = LD2412CustomCluster_t::Attribute<ATTRIB_INTERNALS2, uint32_t>;
    using ZclAttributeArmedForTrigger_t                       = LD2412CustomCluster_t::Attribute<ATTRIB_ARMED_FOR_TRIGGER, bool>;
    using ZclAttributeInternals3_t                            = LD2412CustomCluster_t::Attribute<ATTRIB_INTERNALS3, uint32_t>;

#if defined(ENABLE_ENGINEERING_ATTRIBUTES)
    using ZclAttributeStillDistance_t                         = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STILL_DISTANCE, uint16_t>;
    using ZclAttributeMoveDistance_t                          = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MOVE_DISTANCE, uint16_t>;
    using ZclAttributeMoveEnergy_t                            = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MOVE_ENERGY, uint8_t>;
    using ZclAttributeStillEnergy_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STILL_ENERGY, uint8_t>;
    using ZclAttributeEngineeringEnergyStill_t                = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMove_t                 = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE, EnergyBufType>;
    using ZclAttributeEngineeringEnergyStillMin_t             = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MIN, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMoveMin_t              = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MIN, EnergyBufType>;
    using ZclAttributeEngineeringEnergyStillMax_t             = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MAX, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMoveMax_t              = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MAX, EnergyBufType>;
#endif

    /**********************************************************************/
    /* Inline static definitions                                          */
    /**********************************************************************/
    /**********************************************************************/
    /* Attributes for occupancy cluster                                   */
    /**********************************************************************/
    constexpr ZclAttributeOccupiedToUnoccupiedTimeout_t g_OccupiedToUnoccupiedTimeout{};
    constexpr ZclAttributeOccupancy_t g_OccupancyState{};

    /**********************************************************************/
    /* Attributes for external signal on/off server cluster               */
    /**********************************************************************/
    constexpr ZclAttributeExternalOnOff_t g_ExternalOnOff{};
    /**********************************************************************/
    /* Attributes for a custom cluster                                    */
    /**********************************************************************/
    constexpr ZclAttributeLD2412MoveSensetivity_t                 g_LD2412MoveSensitivity{};
    constexpr ZclAttributeLD2412StillSensetivity_t                g_LD2412StillSensitivity{};
    constexpr ZclAttributeState_t                                 g_LD2412State{};
    constexpr ZclAttributeExState_t                               g_LD2412ExState{};
    constexpr ZclAttributeMaxDistance_t                           g_LD2412MaxDistance{};
    constexpr ZclAttributeMinDistance_t                           g_LD2412MinDistance{};
    constexpr ZclAttributeMode_t                                  g_LD2412Mode{};
    constexpr ZclAttributeEngineeringLight_t                      g_LD2412EngineeringLight{};
    constexpr ZclAttributePIRPresence_t                           g_LD2412PIRPresence{};
    constexpr ZclAttributeDistanceRes_t                           g_LD2412DistanceRes{};
    constexpr ZclAttributeOnOffCommandMode_t                      g_OnOffCommandMode{};
    constexpr ZclAttributeOnOffCommandTimeout_t                   g_OnOffCommandTimeout{};
    constexpr ZclAttributePresenceDetectionIlluminanceThreshold_t g_PresenceDetectionIlluminanceThreshold{};
    constexpr ZclAttributePresenceDetectionConfig_t               g_PresenceDetectionConfig{};
    constexpr ZclAttributeExternalOnTime_t                        g_ExternalOnTime{};
    constexpr ZclAttributeFailureStatus_t                         g_FailureStatus{};
    constexpr ZclAttributeInternals_t                             g_Internals{};
    constexpr ZclAttributeRestartsCount_t                         g_RestartsCount{};
    constexpr ZclAttributeInternals2_t                            g_Internals2{};
    constexpr ZclAttributeArmedForTrigger_t                       g_ArmedForTrigger{};
    constexpr ZclAttributeInternals3_t                            g_Internals3{};

#if defined(ENABLE_ENGINEERING_ATTRIBUTES)
    constexpr ZclAttributeStillDistance_t                         g_LD2412StillDistance{};
    constexpr ZclAttributeMoveDistance_t                          g_LD2412MoveDistance{};
    constexpr ZclAttributeStillEnergy_t                           g_LD2412StillEnergy{};
    constexpr ZclAttributeMoveEnergy_t                            g_LD2412MoveEnergy{};
    constexpr ZclAttributeEngineeringEnergyMove_t                 g_LD2412EngineeringEnergyMove{};
    constexpr ZclAttributeEngineeringEnergyStill_t                g_LD2412EngineeringEnergyStill{};
    constexpr ZclAttributeEngineeringEnergyMoveMin_t              g_LD2412EngineeringEnergyMoveMin{};
    constexpr ZclAttributeEngineeringEnergyStillMin_t             g_LD2412EngineeringEnergyStillMin{};
    constexpr ZclAttributeEngineeringEnergyMoveMax_t              g_LD2412EngineeringEnergyMoveMax{};
    constexpr ZclAttributeEngineeringEnergyStillMax_t             g_LD2412EngineeringEnergyStillMax{};
#endif
}
#endif
