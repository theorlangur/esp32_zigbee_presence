#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "zigbee.hpp"
#include "esp_zigbee_core.h"
#include "driver/gpio.h"

#include "zb_helpers/zbh_helpers.hpp"

#include "ld2412_component.hpp"

#include "device_common.hpp"
#include "device_config.hpp"

#include "board_led.hpp"

#include "zb_binds.hpp"


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
    /*Device basic parameters                                             */
    /**********************************************************************/

    static auto g_Manufacturer = ZbStr("Orlangur");
    static auto g_Model = ZbStr("P-NextGen");
    static uint8_t g_AppVersion = 1;
    static const char *TAG = "ESP_ZB_PRESENCE_SENSOR";

    /**********************************************************************/
    /* PINS                                                               */
    /**********************************************************************/
    static constexpr int LD2412_PINS_TX = 11;
    static constexpr int LD2412_PINS_RX = 10;
    static constexpr int LD2412_PINS_PRESENCE = 4;
    static constexpr int LD2412_PINS_PIR_PRESENCE = 5;
    static constexpr int PINS_RESET = 3;

    static constexpr TickType_t FACTORY_RESET_TIMEOUT = 4;//4 seconds
    static constexpr TickType_t FACTORY_RESET_TIMEOUT_WAIT = 1000 * FACTORY_RESET_TIMEOUT / portTICK_PERIOD_MS;
    static constexpr const uint16_t CLUSTER_ID_LD2412 = kManufactureSpecificCluster;

    /**********************************************************************/
    /* LD2412 Component                                                   */
    /**********************************************************************/
    static ld2412::Component g_ld2412;//THE presence sensor component

    /**********************************************************************/
    /* Colors and patterns                                                */
    /**********************************************************************/
    static constexpr led::Color kColorInfo{255, 128, 0};
    static constexpr led::Color kColorError{255, 0, 0};
    static constexpr led::Color kColorError2{255, 0, 255};
    static constexpr led::Color kColorSpecial{0, 255, 255};
    static constexpr led::Color kColorWhite{255, 255, 255};

    static constexpr uint32_t kBlinkPatternFactoryReset = 0x0F00F00F;
    static constexpr uint32_t kBlinkPatternZStackError = 0x0F00F00F;
    static constexpr uint32_t kBlinkPatternSteeringError = 0x0000F00F;
    static constexpr uint32_t kBlinkPatternCmdError = 0b00000111000111000111000111000111;//5 pulses

    static constexpr led::Color kColorSteering = kColorInfo;
    static constexpr led::Color kSteeringError = kColorError;
    static constexpr led::Color kSteering = kColorInfo;
    static constexpr led::Color kZStackError = kColorError2;
    static constexpr led::Color kLD2412ConfigError = kColorError;
    static constexpr led::Color kCmdError = kColorWhite;
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
    static constexpr const uint16_t ON_OFF_COMMAND_MODE = 19;
    static constexpr const uint16_t ON_OFF_COMMAND_TIMEOUT = 20;
    static constexpr const uint16_t PRESENCE_DETECTION_ILLUMINANCE_THRESHOLD = 21;
    static constexpr const uint16_t ATTRIB_PRESENCE_EDGE_DETECTION_MM_WAVE = 22;
    static constexpr const uint16_t ATTRIB_PRESENCE_EDGE_DETECTION_PIR_INTERNAL = 23;
    static constexpr const uint16_t ATTRIB_PRESENCE_EDGE_DETECTION_EXTERNAL = 24;
    static constexpr const uint16_t ATTRIB_PRESENCE_KEEP_DETECTION_MM_WAVE = 25;
    static constexpr const uint16_t ATTRIB_PRESENCE_KEEP_DETECTION_PIR_INTERNAL = 26;
    static constexpr const uint16_t ATTRIB_PRESENCE_KEEP_DETECTION_EXTERNAL = 27;
    static constexpr const uint16_t EXTERNAL_ON_TIME = 28;
    static constexpr const uint16_t ATTRIB_FAILURE_COUNT = 29;
    static constexpr const uint16_t ATTRIB_FAILURE_STATUS = 30;
    static constexpr const uint16_t ATTRIB_TOTAL_FAILURE_COUNT = 31;
    static constexpr const uint16_t ATTRIB_INTERNALS = 32;
    static constexpr const uint16_t ATTRIB_RESTARTS_COUNT = 33;

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
    using ZclAttributeStillDistance_t                         = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STILL_DISTANCE, uint16_t>;
    using ZclAttributeMoveDistance_t                          = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MOVE_DISTANCE, uint16_t>;
    using ZclAttributeMoveEnergy_t                            = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MOVE_ENERGY, uint8_t>;
    using ZclAttributeStillEnergy_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STILL_ENERGY, uint8_t>;
    using ZclAttributeState_t                                 = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_STATE , LD2412State>;
    using ZclAttributeExState_t                               = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_EX_STATE, ld2412::Component::ExtendedState>;
    using ZclAttributeMaxDistance_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MAX_DISTANCE, uint16_t>;
    using ZclAttributeMinDistance_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MIN_DISTANCE, uint16_t>;
    using ZclAttributeMode_t                                  = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_MODE, LD2412::SystemMode>;
    using ZclAttributeEngineeringLight_t                      = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_LIGHT, uint8_t>;
    using ZclAttributeEngineeringEnergyStill_t                = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMove_t                 = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE, EnergyBufType>;
    using ZclAttributeEngineeringEnergyStillMin_t             = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MIN, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMoveMin_t              = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MIN, EnergyBufType>;
    using ZclAttributeEngineeringEnergyStillMax_t             = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_STILL_MAX, EnergyBufType>;
    using ZclAttributeEngineeringEnergyMoveMax_t              = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_ENGINEERING_ENERGY_MOVE_MAX, EnergyBufType>;
    using ZclAttributePIRPresence_t                           = LD2412CustomCluster_t::Attribute<LD2412_ATTRIB_PIR_PRESENCE, bool>;
    using ZclAttributeOnOffCommandMode_t                      = LD2412CustomCluster_t::Attribute<ON_OFF_COMMAND_MODE, OnOffMode>;
    using ZclAttributeOnOffCommandTimeout_t                   = LD2412CustomCluster_t::Attribute<ON_OFF_COMMAND_TIMEOUT, uint16_t>;
    using ZclAttributePresenceDetectionIlluminanceThreshold_t = LD2412CustomCluster_t::Attribute<PRESENCE_DETECTION_ILLUMINANCE_THRESHOLD, uint8_t>;
    using ZclAttributePresenceEdgeDetectionMMWave_t           = LD2412CustomCluster_t::Attribute<ATTRIB_PRESENCE_EDGE_DETECTION_MM_WAVE, bool>;
    using ZclAttributePresenceEdgeDetectionPIRInternal_t      = LD2412CustomCluster_t::Attribute<ATTRIB_PRESENCE_EDGE_DETECTION_PIR_INTERNAL, bool>;
    using ZclAttributePresenceEdgeDetectionExternal_t         = LD2412CustomCluster_t::Attribute<ATTRIB_PRESENCE_EDGE_DETECTION_EXTERNAL, bool>;
    using ZclAttributePresenceKeepDetectionMMWave_t           = LD2412CustomCluster_t::Attribute<ATTRIB_PRESENCE_KEEP_DETECTION_MM_WAVE, bool>;
    using ZclAttributePresenceKeepDetectionPIRInternal_t      = LD2412CustomCluster_t::Attribute<ATTRIB_PRESENCE_KEEP_DETECTION_PIR_INTERNAL, bool>;
    using ZclAttributePresenceKeepDetectionExternal_t         = LD2412CustomCluster_t::Attribute<ATTRIB_PRESENCE_KEEP_DETECTION_EXTERNAL, bool>;
    using ZclAttributeExternalOnTime_t                        = LD2412CustomCluster_t::Attribute<EXTERNAL_ON_TIME , uint16_t>;
    using ZclAttributeFailureCount_t                          = LD2412CustomCluster_t::Attribute<ATTRIB_FAILURE_COUNT , uint16_t>;
    using ZclAttributeFailureStatus_t                         = LD2412CustomCluster_t::Attribute<ATTRIB_FAILURE_STATUS, uint16_t>;
    using ZclAttributeTotalFailureCount_t                     = LD2412CustomCluster_t::Attribute<ATTRIB_TOTAL_FAILURE_COUNT , uint16_t>;
    using ZclAttributeInternals_t                             = LD2412CustomCluster_t::Attribute<ATTRIB_INTERNALS , uint32_t>;
    using ZclAttributeRestartsCount_t                         = LD2412CustomCluster_t::Attribute<ATTRIB_RESTARTS_COUNT , uint16_t>;

    /**********************************************************************/
    /* Attributes for occupancy cluster                                   */
    /**********************************************************************/
    static ZclAttributeOccupiedToUnoccupiedTimeout_t g_OccupiedToUnoccupiedTimeout;
    static ZclAttributeOccupancy_t g_OccupancyState;

    /**********************************************************************/
    /* Attributes for external signal on/off server cluster               */
    /**********************************************************************/
    static ZclAttributeExternalOnOff_t g_ExternalOnOff;
    /**********************************************************************/
    /* Attributes for a custom cluster                                    */
    /**********************************************************************/
    static ZclAttributeLD2412MoveSensetivity_t                 g_LD2412MoveSensitivity;
    static ZclAttributeLD2412StillSensetivity_t                g_LD2412StillSensitivity;
    static ZclAttributeStillDistance_t                         g_LD2412StillDistance;
    static ZclAttributeMoveDistance_t                          g_LD2412MoveDistance;
    static ZclAttributeStillEnergy_t                           g_LD2412StillEnergy;
    static ZclAttributeMoveEnergy_t                            g_LD2412MoveEnergy;
    static ZclAttributeState_t                                 g_LD2412State;
    static ZclAttributeExState_t                               g_LD2412ExState;
    static ZclAttributeMaxDistance_t                           g_LD2412MaxDistance;
    static ZclAttributeMinDistance_t                           g_LD2412MinDistance;
    static ZclAttributeMode_t                                  g_LD2412Mode;
    static ZclAttributeEngineeringLight_t                      g_LD2412EngineeringLight;
    static ZclAttributeEngineeringEnergyMove_t                 g_LD2412EngineeringEnergyMove;
    static ZclAttributeEngineeringEnergyStill_t                g_LD2412EngineeringEnergyStill;
    static ZclAttributeEngineeringEnergyMoveMin_t              g_LD2412EngineeringEnergyMoveMin;
    static ZclAttributeEngineeringEnergyStillMin_t             g_LD2412EngineeringEnergyStillMin;
    static ZclAttributeEngineeringEnergyMoveMax_t              g_LD2412EngineeringEnergyMoveMax;
    static ZclAttributeEngineeringEnergyStillMax_t             g_LD2412EngineeringEnergyStillMax;
    static ZclAttributePIRPresence_t                           g_LD2412PIRPresence;
    static ZclAttributeOnOffCommandMode_t                      g_OnOffCommandMode;
    static ZclAttributeOnOffCommandTimeout_t                   g_OnOffCommandTimeout;
    static ZclAttributePresenceDetectionIlluminanceThreshold_t g_PresenceDetectionIlluminanceThreshold;
    static ZclAttributePresenceEdgeDetectionMMWave_t           g_PresenceEdgeDetectionMMWave;
    static ZclAttributePresenceEdgeDetectionPIRInternal_t      g_PresenceEdgeDetectionPIRInternal;
    static ZclAttributePresenceEdgeDetectionExternal_t         g_PresenceEdgeDetectionExternal;
    static ZclAttributePresenceKeepDetectionMMWave_t           g_PresenceKeepDetectionMMWave;
    static ZclAttributePresenceKeepDetectionPIRInternal_t      g_PresenceKeepDetectionPIRInternal;
    static ZclAttributePresenceKeepDetectionExternal_t         g_PresenceKeepDetectionExternal;
    static ZclAttributeExternalOnTime_t                        g_ExternalOnTime;
    static ZclAttributeFailureCount_t                          g_FailureCount;
    static ZclAttributeFailureStatus_t                         g_FailureStatus;
    static ZclAttributeTotalFailureCount_t                     g_TotalFailureCount;
    static ZclAttributeInternals_t                             g_Internals;
    static ZclAttributeRestartsCount_t                         g_RestartsCount;


    /**********************************************************************/
    /* Storable data                                                      */
    /**********************************************************************/
    //storable configuration
    LocalConfig g_Config;

    //forward decl
    void cmd_failure(uint8_t cmd_id, esp_zb_zcl_status_t status_code);
    void cmd_total_failure(uint16_t clusterId, uint8_t cmd_id);
    bool is_coordinator(esp_zb_zcl_addr_t &addr);

    template<uint16_t ClusterId, int CmdId, int Retries, auto CmdSender>
    struct CmdWithRetries
    {
        static constexpr uint32_t kCmdResponseWait = 700;//ms
        static constexpr uint16_t kInvalidSeqNr = 0xffff;
        ZbAlarm m_WaitResponseTimer{"m_WaitResponseTimer"};
        uint16_t m_SeqNr = kInvalidSeqNr;
        int m_RetriesLeft = Retries;
        int m_FailureCount = 0;
        bool fake = false;

        zb::seq_nr_t SendRaw() { return fake ? kInvalidSeqNr : CmdSender(); }
        void Send(bool fake = false)
        {
            if (m_SeqNr != kInvalidSeqNr)//another command is already in flight
            {
                ESP_ERROR_CHECK(ESP_FAIL);
                return;//TODO: log? inc failure count?
            }

            this->fake = fake;
            m_RetriesLeft = Retries;

            if constexpr (Retries) //register response
            {
                SendAgain();
            }
            else
                SendRaw();
        }

        void SendAgain()
        {
            SetSeqNr(SendRaw());
            m_SendCallbackProcessed = m_RespCallbackProcessed = false;
            ZbCmdResponse::Register(ClusterId, CmdId, &OnCmdResponse, this);
            m_WaitResponseTimer.Setup(OnTimer, this, kCmdResponseWait);
        }
    private:
        void SetSeqNr(uint16_t nr = kInvalidSeqNr)
        {
            if (m_SeqNr != kInvalidSeqNr)
                ZbCmdSend::Unregister(m_SeqNr);
            m_SeqNr = nr;
            if (m_SeqNr != kInvalidSeqNr)
                ZbCmdSend::Register(m_SeqNr, &OnSendStatus, this);
        }
        bool m_SendCallbackProcessed = false;
        bool m_RespCallbackProcessed = false;
        void OnFailed()
        {
            m_WaitResponseTimer.Cancel();
            m_SendCallbackProcessed = m_RespCallbackProcessed = false;
            SetSeqNr();
            ZbCmdResponse::Unregister(ClusterId, CmdId);
            cmd_total_failure(ClusterId, CmdId);
        }

        static bool OnCmdResponse(uint8_t cmd_id, esp_zb_zcl_status_t status_code, esp_zb_zcl_cmd_info_t *pInfo, void *user_ctx)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)user_ctx;
            if (is_coordinator(pInfo->src_address))
            {
                FMT_PRINT("Response from coordinator on Cmd {:x}; status: {:x}\n", CmdId, (int)status_code);
                return true;//keep response handler
            }

            pCmd->m_RespCallbackProcessed = true;
            pCmd->SetSeqNr();//reset

#ifndef NDEBUG
            {
                using clock_t = std::chrono::system_clock;
                auto now = clock_t::now();
                auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
                FMT_PRINT("{} Response on Cmd {:x} from {}; status: {:x}\n", _n, CmdId, pInfo->src_address, (int)status_code);
            }
#endif
            if (status_code != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ++pCmd->m_FailureCount;
                cmd_failure(CmdId, esp_zb_zcl_status_t(status_code));

                FMT_PRINT("Cmd {:x} failed with status: {:x}\n", CmdId, (int)status_code);
                if (!pCmd->m_RetriesLeft)
                {
                    //failure
                    FMT_PRINT("Cmd {:x} completely failed\n", CmdId);
                    pCmd->OnFailed();
                    return false;//no need to keep.
                }
                //try again
                --pCmd->m_RetriesLeft;
                FMT_PRINT("Retry: Cmd {:x} (left: {})\n", CmdId, pCmd->m_RetriesLeft);
                pCmd->SendAgain();
                return true;//leave registered
            }

            //all good
            pCmd->m_WaitResponseTimer.Cancel();
            return false;
        }

        static void OnSendStatus(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)user_ctx;
            auto status_code = pSendStatus->status;
            if (is_coordinator(pSendStatus->dst_addr))
            {
                FMT_PRINT("Response from coordinator on Cmd {:x}; status: {:x}\n", CmdId, (int)status_code);
                return;//skipping coordinator
            }
            pCmd->m_SendCallbackProcessed = true;
            if (pCmd->m_RespCallbackProcessed)//we got already a resp callback?
            {
                //nevermind then
                FMT_PRINT("Send Cmd {:x} with status {:x}, But already got response earlier\n", CmdId, status_code);
                return;
            }

#ifndef NDEBUG
            {
                using clock_t = std::chrono::system_clock;
                auto now = clock_t::now();
                auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
                FMT_PRINT("{} Send Cmd {:x} seqNr={:x} to {}; status: {:x}\n", _n, CmdId, pSendStatus->tsn, pSendStatus->dst_addr, (int)status_code);
            }
#endif
            if (status_code != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ++pCmd->m_FailureCount;
                cmd_failure(CmdId, ESP_ZB_ZCL_STATUS_FAIL);

                FMT_PRINT("Cmd {:x} failed with status: {:x}\n", CmdId, (int)status_code);
                if (!pCmd->m_RetriesLeft)
                {
                    //failure
                    FMT_PRINT("Cmd {:x} completely failed\n", CmdId);
                    pCmd->OnFailed();
                    return;
                }
                //try again
                --pCmd->m_RetriesLeft;
                FMT_PRINT("Retry: Cmd {:x} (left: {})\n", CmdId, pCmd->m_RetriesLeft);
                pCmd->SendAgain();
                return;
            }
            //sending was ok, now we expect a response
        }

        static void OnTimer(void *p)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)p;
            ++pCmd->m_FailureCount;
            pCmd->SetSeqNr();//reset
            ZbCmdResponse::Unregister(ClusterId, CmdId);
            cmd_failure(CmdId, ESP_ZB_ZCL_STATUS_TIMEOUT);

            if (pCmd->m_RetriesLeft)
            {
                //report timeout
                --pCmd->m_RetriesLeft;
                FMT_PRINT("Retry after timeout: Cmd {:x} (left: {})\n", CmdId, pCmd->m_RetriesLeft);
                pCmd->SendAgain();
                return;
            }

            FMT_PRINT("Cmd {:x} timed out and no retries left\n", CmdId);
            pCmd->OnFailed();
        }
    };

    zb::seq_nr_t send_on_raw()
    {
        esp_zb_zcl_on_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_ON_ID;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        //cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;//ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        //cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = g_BoundAddr;
        //cmd_req.zcl_basic_cmd.dst_endpoint = g_EndP;
        return esp_zb_zcl_on_off_cmd_req(&cmd_req);
    }

    zb::seq_nr_t send_off_raw()
    {
        esp_zb_zcl_on_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        //cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;//ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        //cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = g_BoundAddr;
        //cmd_req.zcl_basic_cmd.dst_endpoint = g_EndP;
        return esp_zb_zcl_on_off_cmd_req(&cmd_req);
    }

    zb::seq_nr_t send_on_timed_raw()
    {
        auto t = g_Config.GetOnOffTimeout();
        esp_zb_zcl_on_off_on_with_timed_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        cmd_req.on_off_control = 0;//process unconditionally
        cmd_req.on_time = t * 10;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        //cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;//ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        //cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = g_BoundAddr;
        //cmd_req.zcl_basic_cmd.dst_endpoint = g_EndP;
        return esp_zb_zcl_on_off_on_with_timed_off_cmd_req(&cmd_req);
    }

    void update_info_on_own_binds();
    
    /**********************************************************************/
    /* Runtime data                                                       */
    /**********************************************************************/
    struct Internals
    {
        enum class SendOnOffResult: uint8_t 
        {
            Initial = 0,
            ReturnOnNothing = 1,
            ReturnOnOffOnly = 2,
            ReturnOnOnTimedOnTimedLocal = 3,
            ReturnOnNoBoundDevices = 4,
            SentTimedOn = 5,
            SentTimedOnLocal = 6,
            SentOn = 7,
            SentOff = 8,
        };

        enum class LocalTimerState: uint8_t
        {
            Initial = 0,
            StillPresenceResetTimer = 1,
            StillPresenceNoTimeout = 2,
            NoPresenceSentOff = 3,
            NoPresenceNoBoundDevices = 4,
        };

        enum class ExternalTimerState: uint8_t
        {
            Initial = 0,
            FinishedNoPresenceChange = 1,
            FinishedPresenceChangeSentOnOff = 2,
        };

        uint32_t m_LastBindResponseStatus : 8 = 0;
        uint32_t m_BoundDevices : 4 = 0;
        uint32_t m_LastSendOnOffResult : 4 = 0;
        uint32_t m_LastLocalTimerState : 4 = 0;
        uint32_t m_LastExternalTimerState : 4 = 0;
        uint32_t m_HasRunningTimer : 1 = 0;
        uint32_t m_HasExternalTimer : 1 = 0;
        uint32_t m_BindRequestActive : 1 = 0;
        uint32_t m_Unused : 5 = 0;

        uint32_t GetVal() const { return *(uint32_t*)this; }

        void Update()
        {
            if (auto status = g_Internals.Set(GetVal()); !status)
            {
                FMT_PRINT("Failed to set internals in update {:x}\n", (int)status.error());
            }
        }
    };

    static_assert(sizeof(Internals) == sizeof(uint32_t));

    esp_err_t read_reporting_cfg_response_handler(const void *message);

    //initialized at start
    struct RuntimeState
    {
        esp_zb_ieee_addr_t m_CoordinatorIeee;
        bool m_FirstRun = true;
        bool m_LastPresence = false;
        bool m_SuppressedByIllulminance = false;
        bool m_LastPresenceMMWave = false;
        bool m_LastPresencePIRInternal = false;
        bool m_LastPresenceExternal = false;
        ZbAlarm m_RunningTimer{"m_RunningTimer"};
        ZbAlarm m_ExternalRunningTimer{"m_ExternalRunningTimer"};

        CmdWithRetries<ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_ID, 2, send_on_raw> m_OnSender;
        CmdWithRetries<ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID, 2, send_off_raw> m_OffSender;
        CmdWithRetries<ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_WITH_TIMED_OFF_ID, 2, send_on_timed_raw> m_OnTimedSender;

        uint16_t m_FailureCount = 0;
        uint16_t m_TotalFailureCount = 0;
        esp_zb_zcl_status_t m_LastFailedStatus = ESP_ZB_ZCL_STATUS_SUCCESS;
        enum class ExtendedFailureStatus: uint16_t
        {
            BindRequestIncomplete = 0xe0
        };

        static constexpr esp_zb_zcl_cluster_id_t g_RelevantBoundClusters[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF};

        Internals m_Internals;
        ZbAlarm m_BindsCheck{"BindsCheck"};

        BindArray m_TrackedBinds;
        BindArray m_BindsToCleanup;
        uint8_t m_ValidBinds = 0;
        uint8_t m_BindStates = 0;//on/off, bit per bind

        static bool IsRelevant(esp_zb_zcl_cluster_id_t id)
        {
            for(auto _i : g_RelevantBoundClusters)
                if (_i == id)
                    return true;
            return false;
        }

        void StartExternalTimer(esp_zb_user_callback_t cb, uint32_t time)
        {
            FMT_PRINT("Starting external timer for {} ms\n", time);
            m_ExternalRunningTimer.Setup(cb, nullptr, time);
        }

        void StartLocalTimer(esp_zb_user_callback_t cb, uint32_t time)
        {
            FMT_PRINT("Starting local timer for {} ms\n", time);
            m_RunningTimer.Setup(cb, nullptr, time);
        }

        void RunBindsChecking()
        {
            if (!m_Internals.m_BindRequestActive)
            {
                m_Internals.m_BindRequestActive = true;
                update_info_on_own_binds();
            }
            else
                m_LastFailedStatus = (esp_zb_zcl_status_t)ExtendedFailureStatus::BindRequestIncomplete;

            m_BindsCheck.Setup([](void *p){
                        RuntimeState *pState = (RuntimeState *)p;
                        pState->RunBindsChecking();
                    }, this, 2000);
        }

        void RunService()
        {
            ZbAlarm::check_death_count();

            m_Internals.m_HasRunningTimer = m_RunningTimer.IsRunning();
            m_Internals.m_HasExternalTimer = m_ExternalRunningTimer.IsRunning();
            m_Internals.Update();//send to zigbee

            for(auto i = m_BindsToCleanup.begin(); i != m_BindsToCleanup.end(); ++i)
            {
                if ((*i)->GetState() == BindInfo::State::NonFunctional)
                {
                    m_BindsToCleanup.erase(i--);
                }
            }

            //update validity of the binds
            for(size_t i = 0, n = m_TrackedBinds.size(); i < n; ++i)
            {
                auto &bi = m_TrackedBinds[i];
                if (bi->GetState() == BindInfo::State::Functional)
                    m_ValidBinds |= 1 << i;
                else
                    m_ValidBinds &= ~(1 << i);
            }
            
            static ZbAlarm rep{"InternalsReporting"};
            rep.Setup([](void *p){
                        RuntimeState *pState = (RuntimeState *)p;
                        pState->RunService();
                    }, this, 1000);
        }
    };
    RuntimeState g_State;

    LinkedListT<ReadConfigResponseNode> g_ReadConfigResponseHandlers;
    void RegisterReadConfigResponseHandler(ReadConfigResponseNode &n)
    {
        g_ReadConfigResponseHandlers += n;
    }

    esp_err_t read_reporting_cfg_response_handler(const void *message)
    {
        esp_zb_zcl_cmd_read_report_config_resp_message_t *pResp = (esp_zb_zcl_cmd_read_report_config_resp_message_t *)message;
        FMT_PRINT("Read report resp:\n");
        for(auto *pNode : g_ReadConfigResponseHandlers)
        {
            if (pNode->Notify(pResp))
                return ESP_OK;
        }
        return ESP_OK;
    }

    LinkedListT<ConfigReportResponseNode> g_ConfigReportResponseHandlers;
    void RegisterConfigReportResponseHandler(ConfigReportResponseNode &n)
    {
        g_ConfigReportResponseHandlers += n;
    }

    esp_err_t config_report_response_handler(const void *message)
    {
        esp_zb_zcl_cmd_config_report_resp_message_t *pResp = (esp_zb_zcl_cmd_config_report_resp_message_t *)message;
        FMT_PRINT("Config Report Response:\n");
        for(auto *pNode : g_ConfigReportResponseHandlers)
        {
            if (pNode->Notify(pResp))
                return ESP_OK;
        }
        return ESP_OK;
    }


    esp_err_t report_attributes_handler(const void *message)
    {
        esp_zb_zcl_report_attr_message_t *pReport = (esp_zb_zcl_report_attr_message_t *)message;
        FMT_PRINT("Got report from {}, cluster {:x}, status {:x}; attr: {:x}; Data type: {:x}; Size: {}\n"
                , pReport->src_address
                , pReport->cluster
                , pReport->status
                , pReport->attribute.id
                , (int)pReport->attribute.data.type
                , (int)pReport->attribute.data.size
                );
        if (pReport->cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            auto bindIt = g_State.m_TrackedBinds.end();
            if (pReport->src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
                bindIt = g_State.m_TrackedBinds.find(pReport->src_address.u.short_addr, &BindInfo::m_ShortAddr);
            else if (pReport->src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
                bindIt = g_State.m_TrackedBinds.find(ieee_addr{pReport->src_address.u.ieee_addr}, &BindInfo::m_IEEE);

            if (bindIt != g_State.m_TrackedBinds.end())
            {
                size_t idx = bindIt - g_State.m_TrackedBinds.begin();
                FMT_PRINT("Found a bind info at index {}\n", idx);
                if (g_State.m_ValidBinds & (1 << idx))
                {
                    g_State.m_BindStates &= ~(1 << idx);
                    bool *pVal = (bool *)pReport->attribute.data.value;
                    FMT_PRINT("New state of the bind info: {}\n", *pVal);
                    g_State.m_BindStates |= (int(*pVal) << idx);

                    if (!(g_State.m_BindStates & g_State.m_ValidBinds) && g_State.m_LastPresence)//we still have the presence
                    {
                        //re-arm, so that we can again detect and react
                        g_State.m_LastPresence = false;
                        if (auto status = g_OccupancyState.Set(ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED); !status)
                        {
                            FMT_PRINT("(Bind logic)Failed to set occupancy attribute with error {:x}\n", (int)status.error());
                        }
                    }
                }else
                {
                    FMT_PRINT("BindInfo has an invalid state\n");
                }
            }else
            {
                //not found. Unbind?
            }
        }
        return ESP_OK;
    }

    bool is_coordinator(esp_zb_zcl_addr_t &addr)
    {
        if (addr.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
            return addr.u.short_addr == 0;
        if (addr.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
            return std::memcmp(addr.u.ieee_addr, g_State.m_CoordinatorIeee, sizeof(esp_zb_ieee_addr_t)) == 0;
        //anything else is not supported
        return false;
    }

    void cmd_failure(uint8_t cmd_id, esp_zb_zcl_status_t status_code)
    {
        ++g_State.m_FailureCount;
        g_State.m_LastFailedStatus = status_code;

        if (auto status = g_FailureCount.Set(g_State.m_FailureCount); !status)
        {
            FMT_PRINT("Failed to inc failure count {:x}\n", (int)status.error());
        }
        if (auto status = g_FailureStatus.Set((uint16_t)g_State.m_LastFailedStatus); !status)
        {
            FMT_PRINT("Failed to set failure status {:x}\n", (int)status.error());
        }
    }

    void cmd_total_failure(uint16_t clusterId, uint8_t cmd_id)
    {
        led::blink_pattern(kBlinkPatternCmdError, kCmdError, duration_ms_t(1000));
        led::blink(false, {});
        ++g_State.m_TotalFailureCount;
        if (auto status = g_TotalFailureCount.Set(g_State.m_TotalFailureCount); !status)
        {
            FMT_PRINT("Failed to inc total failure count {:x}\n", (int)status.error());
        }
    }

    void update_info_on_own_binds()
    {
        esp_zb_zdo_mgmt_bind_param_t cmd_req;
        cmd_req.dst_addr = esp_zb_get_short_address();
        cmd_req.start_index = 0;

        ESP_LOGI(TAG, "Sending request to self to get our current binds");
        FMT_PRINT("My short addr: {:x}\n", cmd_req.dst_addr);
        esp_zb_zdo_binding_table_req(&cmd_req, 
            [](const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx)
            {
                BindArray newBinds;
                g_State.m_Internals.m_BindRequestActive = false;
                g_State.m_Internals.m_LastBindResponseStatus = table_info->status;
                for(auto &bi : g_State.m_TrackedBinds)
                    bi->m_BindChecked = false;
                int foundBinds = 0;
                ESP_LOGI(TAG, "Binds callback");
                auto *pRec = table_info->record;
                int recs = 0;
                int foundExisting = 0;
                while(pRec)
                {
                    esp_zb_zcl_addr_t addr;
                    addr.addr_type = pRec->dst_addr_mode;
                    std::memcpy(&addr.u, &pRec->dst_address, sizeof(addr.u));
                    FMT_PRINT("Bind {}. Addr={} to cluster {:x}, ep={}\n", recs, addr, pRec->cluster_id, pRec->dst_endp);
                    if (pRec->dst_addr_mode == ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED)
                    {
                        if (zb::ieee_addr{pRec->dst_address.addr_long} != zb::ieee_addr{g_State.m_CoordinatorIeee})
                        {
                            if (RuntimeState::IsRelevant(esp_zb_zcl_cluster_id_t(pRec->cluster_id)))//got it. at least one
                            {
                                auto existingI = g_State.m_TrackedBinds.find(zb::ieee_addr{pRec->dst_address.addr_long}, &BindInfo::m_IEEE);
                                if (existingI == g_State.m_TrackedBinds.end())
                                {
                                    FMT_PRINT("This is a new bind\n");
                                    auto r = newBinds.emplace_back(pRec->dst_address.addr_long, esp_zb_address_short_by_ieee(pRec->dst_address.addr_long));
                                    if (r)
                                    {
                                        BindInfoPtr &ptr = *r;
                                        if (ptr)
                                        {
                                            BindInfo &bi = *ptr;
                                            bi.m_BindChecked = true;
                                            bi.m_EP = pRec->dst_endp;
                                            bi.m_AttemptsLeft = BindInfo::kMaxConfigAttempts;
                                            bi.Do();//start the whole thing
                                        }
                                    }
                                }else
                                {
                                    FMT_PRINT("This is a existing bind\n");
                                    ++foundExisting;
                                    (*existingI)->m_BindChecked = true;
                                    (*existingI)->m_EP = pRec->dst_endp;
                                }
                                ++foundBinds;
                            }
                        }
                    }
                    pRec = pRec->next;
                    ++recs;
                }

                uint8_t newStates = 0;
                uint8_t newValidity = 0;
                if (foundExisting != g_State.m_TrackedBinds.size())
                {
                    int nextOldIdx = 0, nextNewIdx = 0;
                    for(auto i = g_State.m_TrackedBinds.begin(); i != g_State.m_TrackedBinds.end(); ++i, ++nextOldIdx)
                    {
                        if (!(*i)->m_BindChecked)
                        {
                            (*i)->Unbind();
                            g_State.m_BindsToCleanup.push_back(std::move(*i));
                            g_State.m_TrackedBinds.erase(i--);
                        }
                        else
                        {
                            newStates |= (g_State.m_BindStates & (1 << nextOldIdx)) >> (nextNewIdx - nextOldIdx);
                            newValidity |= (g_State.m_ValidBinds & (1 << nextOldIdx)) >> (nextNewIdx - nextOldIdx);
                            ++nextNewIdx;
                        }
                    }
                }else
                {
                    newStates = g_State.m_BindStates;
                    newValidity = g_State.m_ValidBinds;
                }

                for(auto &bi : newBinds)
                    g_State.m_TrackedBinds.push_back(std::move(bi));

                g_State.m_BindStates = newStates;
                g_State.m_ValidBinds = newValidity;
                g_State.m_Internals.m_BoundDevices = g_State.m_TrackedBinds.size();
                FMT_PRINT("Binds found: {}\n", foundBinds);
            },
            nullptr
        );
        //ESP_LOGI(TAG, "Check own binds: finish");
    }

    void on_local_on_timer_finished(void* param)
    {
        //this runs in the context of zigbee task
        if (g_State.m_LastPresence)//presence still active. start another timer
        {
            if (g_Config.GetOnOffTimeout())//still valid timeout?
            {
                g_State.m_Internals.m_LastLocalTimerState = (uint8_t)Internals::LocalTimerState::StillPresenceResetTimer;
                g_State.m_RunningTimer.Setup(&on_local_on_timer_finished, nullptr, g_Config.GetOnOffTimeout() * 1000);
                //let's see
                //FMT_PRINT("Non-Fake On\n");
                //g_State.m_OnSender.Send(false);
            }
            else
            {
                g_State.m_Internals.m_LastLocalTimerState = (uint8_t)Internals::LocalTimerState::StillPresenceNoTimeout;
            }
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
            if (g_State.m_Internals.m_BoundDevices)
            {
                g_State.m_Internals.m_LastLocalTimerState = (uint8_t)Internals::LocalTimerState::NoPresenceSentOff;
                g_State.m_OffSender.Send();
            }else
            {
                g_State.m_Internals.m_LastLocalTimerState = (uint8_t)Internals::LocalTimerState::NoPresenceNoBoundDevices;
            }

            ZbAlarm::check_counter_of_death();
        }
    }

    static void send_on_off(bool on)
    {
        auto m = g_Config.GetOnOffMode();
        auto t = g_Config.GetOnOffTimeout();
        if (m == OnOffMode::Nothing)
        {
            g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::ReturnOnNothing;
            return;//nothing
        }
        if (on && (m == OnOffMode::OffOnly))
        {
            g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::ReturnOnOffOnly;
            return;//nothing
        }
        if (!on && (m == OnOffMode::OnOnly || m == OnOffMode::TimedOn || m == OnOffMode::TimedOnLocal))
        {
            g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::ReturnOnOnTimedOnTimedLocal;
            return;//nothing
        }

        if (!g_State.m_Internals.m_BoundDevices)
        {
            g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::ReturnOnNoBoundDevices;
            return;//no bound devices with on/off cluster, no reason to send a command
        }

        g_State.m_RunningTimer.Cancel();
        if (m == OnOffMode::TimedOn)
        {
            g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::SentTimedOn;
            FMT_PRINT("Sending timed on command to binded with timeout: {};\n", t);
            g_State.m_OnTimedSender.Send();
        }
        else if (m == OnOffMode::TimedOnLocal)
        {
            g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::SentTimedOnLocal;
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
        }
        else
        {
            FMT_PRINT("Sending command to binded: {};\n", (int)on);
            if (on)
            {
                g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::SentOn;
                g_State.m_OnSender.Send();
            }
            else
            {
                g_State.m_Internals.m_LastSendOnOffResult = (uint8_t)Internals::SendOnOffResult::SentOff;
                g_State.m_OffSender.Send();
            }
        }
    }

    //returns 'true' if changed
    bool update_presence_state()
    {
        bool changed = false;
        auto cfg = g_Config.GetPresenceDetectionMode();
        if (g_State.m_FirstRun || !g_State.m_LastPresence)
        {
            //edge detection
            g_State.m_LastPresence = 
                   (cfg.m_Edge_mmWave && g_State.m_LastPresenceMMWave)
                || (cfg.m_Edge_PIRInternal && g_State.m_LastPresencePIRInternal)
                || (cfg.m_Edge_External && g_State.m_LastPresenceExternal);
            changed = g_State.m_LastPresence;
            g_State.m_FirstRun = false;
        }else if (!g_State.m_FirstRun && g_State.m_LastPresence)
        {
            //keep detection
            g_State.m_LastPresence = 
                   (cfg.m_Keep_mmWave && g_State.m_LastPresenceMMWave)
                || (cfg.m_Keep_PIRInternal && g_State.m_LastPresencePIRInternal)
                || (cfg.m_Keep_External && g_State.m_LastPresenceExternal);
            changed = !g_State.m_LastPresence;
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
            if ((g_Config.GetIlluminanceThreshold() < LocalConfig::kMaxIlluminance) && (g_ld2412.GetMeasuredLight() > g_Config.GetIlluminanceThreshold()))
                g_State.m_SuppressedByIllulminance = true;
            else
                g_State.m_SuppressedByIllulminance = false;
        }

        return changed;
    }

    static void on_movement_callback(bool _presence, ld2412::Component::PresenceResult const& p, ld2412::Component::ExtendedState exState)
    {
        APILock l;
        g_State.m_LastPresenceMMWave = p.mmPresence;
        g_State.m_LastPresencePIRInternal = p.pirPresence;

        bool presence_changed = update_presence_state();

        if (g_State.m_SuppressedByIllulminance)
            return;

        /**********************************************************************/
        /* Zigbee attributes update                                           */
        /**********************************************************************/
        esp_zb_zcl_occupancy_sensing_occupancy_t val = g_State.m_LastPresence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
        {
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
                send_on_off(g_State.m_LastPresence);
            }
        }
        FMT_PRINT("Presence: {}; Data: {}\n", (int)g_State.m_LastPresence, p);
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

        g_Config.SetLD2412Mode(g_ld2412.GetMode());//save in the config
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
            if (auto status = g_FailureCount.Set(g_State.m_FailureCount); !status)
            {
                FMT_PRINT("Failed to set initial failure count {:x}\n", (int)status.error());
            }
            if (auto status = g_FailureStatus.Set((uint16_t)g_State.m_LastFailedStatus); !status)
            {
                FMT_PRINT("Failed to set initial failure status {:x}\n", (int)status.error());
            }
            if (auto status = g_TotalFailureCount.Set(g_State.m_TotalFailureCount); !status)
            {
                FMT_PRINT("Failed to set initial total failure count {:x}\n", (int)status.error());
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
            if (auto status = g_PresenceEdgeDetectionMMWave.Set(presenceDetectionMode.m_Edge_mmWave); !status)
            {
                FMT_PRINT("Failed to set initial detection mode edge mmWave {:x}\n", (int)status.error());
            }
            if (auto status = g_PresenceEdgeDetectionPIRInternal.Set(presenceDetectionMode.m_Edge_PIRInternal); !status)
            {
                FMT_PRINT("Failed to set initial detection mode edge PIR Internal {:x}\n", (int)status.error());
            }
            if (auto status = g_PresenceEdgeDetectionExternal.Set(presenceDetectionMode.m_Edge_External); !status)
            {
                FMT_PRINT("Failed to set initial detection mode edge external {:x}\n", (int)status.error());
            }
            if (auto status = g_PresenceKeepDetectionMMWave.Set(presenceDetectionMode.m_Keep_mmWave); !status)
            {
                FMT_PRINT("Failed to set initial detection mode keep mmWave {:x}\n", (int)status.error());
            }
            if (auto status = g_PresenceKeepDetectionPIRInternal.Set(presenceDetectionMode.m_Keep_PIRInternal); !status)
            {
                FMT_PRINT("Failed to set initial detection mode keep PIR internal {:x}\n", (int)status.error());
            }
            if (auto status = g_PresenceKeepDetectionExternal.Set(presenceDetectionMode.m_Keep_External); !status)
            {
                FMT_PRINT("Failed to set initial detection mode keep external {:x}\n", (int)status.error());
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
                        led::blink(true, kLD2412ConfigError);
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
        g_Config.FactoryReset();
        esp_zb_factory_reset();//this resets/reboots the device
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
        g_ld2412.SwitchBluetooth(on);
        return ESP_OK;
    }

    /**********************************************************************/
    /* On/Off server cluster commands for external sensor                 */
    /**********************************************************************/
    void on_external_on_timer_finished(void* param)
    {
        FMT_PRINT("External timer finished\n");
        g_State.m_LastPresenceExternal = false;
        if (auto status = g_ExternalOnOff.Set(g_State.m_LastPresenceExternal); !status)
        {
            FMT_PRINT("Failed to set on/off state attribute with error {:x}\n", (int)status.error());
        }
        //g_ExternalOnOff.Report(g_DestCoordinator);
        if (update_presence_state())
        {
            g_State.m_Internals.m_LastExternalTimerState = (uint8_t)Internals::ExternalTimerState::FinishedPresenceChangeSentOnOff;
            send_on_off(g_State.m_LastPresence);
            esp_zb_zcl_occupancy_sensing_occupancy_t val = g_State.m_LastPresence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
            if (auto status = g_OccupancyState.Set(val); !status)
            {
                FMT_PRINT("Failed to set occupancy attribute with error {:x}\n", (int)status.error());
            }
        }else
            g_State.m_Internals.m_LastExternalTimerState = (uint8_t)Internals::ExternalTimerState::FinishedNoPresenceChange;
    }

    esp_err_t cmd_on_off_external_on()
    {
        FMT_PRINT("Switching external on/off ON\n");
        g_State.m_LastPresenceExternal = true;
        g_ExternalOnOff.Set(g_State.m_LastPresenceExternal);

        if (auto et = g_Config.GetExternalOnOffTimeout(); et > 0)
            g_State.StartExternalTimer(&on_external_on_timer_finished, et * 1000);
        else
            g_State.m_ExternalRunningTimer.Cancel();

        if (update_presence_state())
        {
            send_on_off(g_State.m_LastPresence);
            esp_zb_zcl_occupancy_sensing_occupancy_t val = g_State.m_LastPresence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
            if (auto status = g_OccupancyState.Set(val); !status)
            {
                FMT_PRINT("Failed to set occupancy attribute with error {:x}\n", (int)status.error());
            }
        }
        return ESP_OK;
    }

    esp_err_t cmd_on_off_external_off()
    {
        FMT_PRINT("Switching external on/off OFF {}...\n");
        g_State.m_LastPresenceExternal = false;
        g_State.m_ExternalRunningTimer.Cancel();
        g_ExternalOnOff.Set(g_State.m_LastPresenceExternal);
        if (update_presence_state())
        {
            send_on_off(g_State.m_LastPresence);
            esp_zb_zcl_occupancy_sensing_occupancy_t val = g_State.m_LastPresence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
            if (auto status = g_OccupancyState.Set(val); !status)
            {
                FMT_PRINT("Failed to set occupancy attribute with error {:x}\n", (int)status.error());
            }
        }
        return ESP_OK;
    }

    struct OnWithTimedOffPayload{
        uint8_t on_off_ctrl;
        uint16_t on_time;
        uint16_t off_wait_time;

        static std::optional<OnWithTimedOffPayload> from(const esp_zb_zcl_custom_cluster_command_message_t *message)
        {
            if (message->data.size == 5)
            {
                OnWithTimedOffPayload r;
                std::memcpy(&r.on_off_ctrl, (const uint8_t*)message->data.value, 1);
                std::memcpy(&r.on_time, (const uint8_t*)message->data.value + 1, 2);
                std::memcpy(&r.off_wait_time, (const uint8_t*)message->data.value + 3, 2);
                return r;
            }
            return std::nullopt;
        }
    };
    esp_err_t cmd_on_off_external_on_with_timed_off(OnWithTimedOffPayload const& data)
    {
        FMT_PRINT("Switching external on/off ON with params: ctrl: {}; on time: {} (deci-seconds); off wait time: {} (deci-seconds)\n", data.on_off_ctrl, data.on_time, data.off_wait_time);
        g_State.m_LastPresenceExternal = true;
        g_State.m_ExternalRunningTimer.Cancel();
        g_ExternalOnOff.Set(g_State.m_LastPresenceExternal);
        if (data.on_time > 0 && data.on_time < 0xfffe)
        {
            //(re-)start timer
            uint32_t t = data.on_time * 100;
            if (auto et = g_Config.GetExternalOnOffTimeout(); et > 0)
                t = et * 1000;
            g_State.StartExternalTimer(&on_external_on_timer_finished, t);
        }
        if (update_presence_state())
        {
            send_on_off(g_State.m_LastPresence);
            esp_zb_zcl_occupancy_sensing_occupancy_t val = g_State.m_LastPresence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
            if (auto status = g_OccupancyState.Set(val); !status)
            {
                FMT_PRINT("Failed to set occupancy attribute with error {:x}\n", (int)status.error());
            }
        }
        return ESP_OK;
    }

    static const ZbCmdHandler g_Commands[] = {
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_RESTART, &ld2412_cmd_restart>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_FACTORY_RESET, &ld2412_cmd_factory_reset>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_RESET_ENERGY_STAT, &ld2412_cmd_reset_energy_stat>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_BLUETOOTH, &ld2412_cmd_switch_bluetooth, bool>{},
        CmdDescr<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_ID, &cmd_on_off_external_on>{},
        CmdDescr<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID, &cmd_on_off_external_off>{},
        CmdDescr<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_WITH_TIMED_OFF_ID, &cmd_on_off_external_on_with_timed_off, OnWithTimedOffPayload>{},
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
                    send_on_off(g_State.m_LastPresence);
                    esp_zb_zcl_occupancy_sensing_occupancy_t val = g_State.m_LastPresence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
                    if (auto status = g_OccupancyState.Set(val); !status)
                    {
                        FMT_PRINT("Failed to set occupancy attribute with error {:x}\n", (int)status.error());
                    }
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
    /* ReportAttributeHandler's                                           */
    /**********************************************************************/
    static const ReportAttributeHandler g_ReportHandlers[] = {
        ReportAttributeHandler(kAnyEP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
            [](const esp_zb_zcl_report_attr_message_t *message)->esp_err_t
            {
                FMT_PRINT("Got report on on/off attribute from {}; State={}\n", message->src_address, *(bool*)message->attribute.data.value);
                return ESP_OK;
            }
            ),

        {}//last one
    };
    static const ReportAttributesHandlingDesc g_ReportHandlingDesc = {
        /*default*/[](const esp_zb_zcl_report_attr_message_t *message)->esp_err_t
        {
            esp_err_t ret = ESP_OK;

            ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
            ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                                message->status);
            ESP_LOGI(TAG, "Received report message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d), data type(%d)", message->dst_endpoint, message->cluster,
                     message->attribute.id, message->attribute.data.size, message->attribute.data.type);
            return ret;
        }
        ,g_ReportHandlers
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

        ESP_ERROR_CHECK(g_OnOffCommandMode.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_OnOffCommandTimeout.AddToCluster(custom_cluster, Access::RW));

        ESP_ERROR_CHECK(g_PresenceDetectionIlluminanceThreshold.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_PresenceEdgeDetectionMMWave.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_PresenceEdgeDetectionPIRInternal.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_PresenceEdgeDetectionExternal.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_PresenceKeepDetectionMMWave.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_PresenceKeepDetectionPIRInternal.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_PresenceKeepDetectionExternal.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_ExternalOnTime.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_FailureCount.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_FailureStatus.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_TotalFailureCount.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_Internals.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_RestartsCount.AddToCluster(custom_cluster, Access::Read | Access::Report, g_Config.GetRestarts()));

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

        {
            /**********************************************************************/
            /* Client on/off cluster for direct binding purposes                  */
            /**********************************************************************/
            esp_zb_on_off_cluster_cfg_t on_off_cfg{.on_off = false};
            esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
        }

        {
            /*************************************************************************/
            /* Server on/off cluster for direct binding purposes for external signal */
            /*************************************************************************/
            esp_zb_on_off_cluster_cfg_t on_off_cfg{.on_off = false};
            esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        }

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
        static int failed_counter = 0;
        switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                failed_counter = 0;
                led::blink(false, {});
                //async setup
                thread::start_task({.pName="LD2412_Setup", .stackSize = 2*4096}, &setup_sensor).detach();
                g_State.RunService();
                g_State.RunBindsChecking();

                ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted");
                    esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_State.m_CoordinatorIeee);
                }
            } else {
                /* commissioning failed */
                if (++failed_counter > 6)
                {
                    failed_counter = 0;
                    esp_restart();
                }
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
                led::blink_pattern(kBlinkPatternZStackError, kZStackError, duration_ms_t(1000));
                led::blink(true, kSteering);
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                failed_counter = 0;
                led::blink(false, {});
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

                esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_State.m_CoordinatorIeee);
            } else {
                if (++failed_counter > 4)
                {
                    failed_counter = 0;
                    esp_restart();
                }
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                led::blink_pattern(kBlinkPatternSteeringError, kSteeringError, duration_ms_t(1000));
                led::blink(true, kSteering);
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
        esp_zb_core_action_handler_register(
                generic_zb_action_handler<
                    ActionHandler{ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID, cmd_custom_cluster_req_cb<g_CommandsDesc>},
                    ActionHandler{ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID, cmd_response_action_handler},
                    ActionHandler{ESP_ZB_CORE_CMD_READ_REPORT_CFG_RESP_CB_ID, read_reporting_cfg_response_handler},
                    ActionHandler{ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID, config_report_response_handler},
                    ActionHandler{ESP_ZB_CORE_REPORT_ATTR_CB_ID, report_attr_cb<g_ReportHandlingDesc>},
                    ActionHandler{ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID, set_attr_value_cb<g_AttributeHandlingDesc>}
                >
        );
        esp_zb_zcl_command_send_status_handler_register(&zb::ZbCmdSend::handler);
        ESP_LOGI(TAG, "ZB registered device");
        fflush(stdout);

        ESP_LOGI(TAG, "ZB updated attribute reporting");
        fflush(stdout);

        esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
        ESP_LOGI(TAG, "ZB set channel masks");
        fflush(stdout);

        led::blink(true, kSteering);
        ESP_ERROR_CHECK(esp_zb_start(false));
        ESP_LOGI(TAG, "ZB started, looping...");
        esp_zb_stack_main_loop();

        g_Config.on_end();
    }

    void reset_pin_isr(void *param)
    {
        QueueHandle_t q = (QueueHandle_t)param;
        int l = gpio_get_level(gpio_num_t(PINS_RESET));
        xQueueSendFromISR(q, &l, nullptr);
    }

    void config_reset_pin(QueueHandle_t q)
    {
        gpio_config_t reset_pin_cfg = {
            .pin_bit_mask = 1ULL << PINS_RESET,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
            .hys_ctrl_mode = gpio_hys_ctrl_mode_t{}
        };

        gpio_config(&reset_pin_cfg);
        gpio_isr_handler_add(gpio_num_t(PINS_RESET), reset_pin_isr, q);
    }

    void setup()
    {
#ifdef NDEBUG
        //esp_log_level_set(ESP_LOG_NONE); 
#endif

        led::setup();
        led::blink(false, {});

        esp_zb_platform_config_t config = {
            .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE, .radio_uart_config = {}},
            .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, .host_uart_config = {}},
        };
        ESP_ERROR_CHECK(nvs_flash_init());
        FMT_PRINT("nvs_flash_init done\n");
        ESP_ERROR_CHECK(esp_zb_platform_config(&config));
        FMT_PRINT("esp_zb_platform_config done\n");
        ESP_ERROR_CHECK(g_Config.on_start());
        QueueHandle_t mainQueue = xQueueCreate(10, sizeof(int));
        config_reset_pin(mainQueue);
        xTaskCreate(zigbee_main, "Zigbee_main", 2*4096, NULL, thread::kPrioDefault, NULL);

        auto waitTime = portMAX_DELAY;
        // FACTORY_RESET_TIMEOUT_WAIT
        enum States{
            Idle,
            ResetPressed,
        } state = States::Idle;

        using clock_t = std::chrono::system_clock;
        auto pressed_time = clock_t::now();
        while(true)
        {
            int v;
            if (xQueueReceive(mainQueue, &v, waitTime)) //process
            {
                switch(state)
                {
                    case States::Idle:
                    {
                        if (v == 0)//pressed
                        {
                            FMT_PRINT("detected RESET pin LOW\n");
                            state = States::ResetPressed;
                            pressed_time = clock_t::now();
                            waitTime = FACTORY_RESET_TIMEOUT_WAIT;
                        }
                    }
                    break;
                    case States::ResetPressed:
                    {
                        //need a gate here to prevent sporadic changes
                        if (v == 1)//actually released. before timeout
                        {
                            state = States::Idle;
                            waitTime = portMAX_DELAY;

                            if (std::chrono::duration_cast<std::chrono::milliseconds>(clock_t::now() - pressed_time).count() > 100)
                            {
                                FMT_PRINT("detected RESET pin HIGH before timeout\n");
                                esp_restart();//simple restart
                            }else
                            {
                                FMT_PRINT("filtered out pin HIGH before timeout\n");
                            }
                        }
                    }
                    break;
                }
            }else
            {
                //timeout
                if (state == States::ResetPressed)
                {
                    //yep, pressed
                    //factory reset
                    FMT_PRINT("detected RESET pin LOW long time. Making Factory Reset\n");
                    waitTime = portMAX_DELAY;
                    state = States::Idle;
                    APILock l;
                    led::blink_pattern(kBlinkPatternFactoryReset, kColorSpecial, duration_ms_t(1500));
                    ld2412_cmd_factory_reset();
                    esp_restart();//not sure if this is necessarry, since zigbee factory resets should restart as well
                }
            }
        }
    }
}
