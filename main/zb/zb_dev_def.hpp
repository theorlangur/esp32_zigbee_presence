#ifndef ZB_DEV_DEF_HPP_
#define ZB_DEV_DEF_HPP_

#include "zb_dev_def_const.hpp"
#include "zb_dev_def_attr.hpp"
#include "../device_config.hpp"
#include "zb_binds.hpp"

namespace zb
{

    /**********************************************************************/
    /*Device basic parameters                                             */
    /**********************************************************************/

    [[maybe_unused]]inline static auto g_Manufacturer = ZbStr("Orlangur");
    [[maybe_unused]]inline static auto g_Model = ZbStr("P-NextGen");
    [[maybe_unused]]inline static uint8_t g_AppVersion = 1;
    [[maybe_unused]]inline static const char *TAG = "ESP_ZB_PRESENCE_SENSOR";

    /**********************************************************************/
    /* PINS                                                               */
    /**********************************************************************/
#if defined(CONFIG_IDF_TARGET_ESP32C6)
    static constexpr int LD2412_PINS_TX = 15; 
    static constexpr int LD2412_PINS_RX = 14;
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
    static constexpr int LD2412_PINS_TX = 11; 
    static constexpr int LD2412_PINS_RX = 10;
#else
#error "Unsupported board"
#endif
    static constexpr int LD2412_PINS_PRESENCE = 4;
    static constexpr int LD2412_PINS_PIR_PRESENCE = 5;
    static constexpr int PINS_RESET = 3;

    static constexpr TickType_t FACTORY_RESET_TIMEOUT = 4;//4 seconds
    static constexpr TickType_t FACTORY_RESET_TIMEOUT_WAIT = 1000 * FACTORY_RESET_TIMEOUT / portTICK_PERIOD_MS;


    /**********************************************************************/
    /* Functions                                                          */
    /**********************************************************************/
    void reset_button_loop();

    void cmd_failure(void *, esp_zb_zcl_status_t status_code, esp_err_t e);
    void cmd_total_failure(void*, esp_zb_zcl_status_t status_code, esp_err_t e);

    zb::seq_nr_t send_on_raw(void*);
    zb::seq_nr_t send_off_raw(void*);
    zb::seq_nr_t send_on_timed_raw(void*);

    [[nodiscard]] bool send_on_off(bool on);
    bool update_presence_state();
    void setup_sensor();
    void update_zb_occupancy_attr();

    struct OnWithTimedOffPayload;

    esp_err_t cmd_on_off_external_on();
    esp_err_t cmd_on_off_external_off();
    esp_err_t cmd_on_off_external_on_with_timed_off(OnWithTimedOffPayload const& data);
    void on_external_on_timer_finished(void* param);
    void update_external_presence(bool _new);
    esp_err_t ias_zone_state_change(const void *_m);

    /**********************************************************************/
    /* Types                                                              */
    /**********************************************************************/
    struct OnWithTimedOffPayload{
        uint8_t on_off_ctrl;
        uint16_t on_time;
        uint16_t off_wait_time;
        static std::optional<OnWithTimedOffPayload> from(const esp_zb_zcl_custom_cluster_command_message_t *message);
    };
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

        uint32_t m_BoundDevices                 : 4  = 0;
        uint32_t m_DeviceUnavailable            : 4  = 0;
        uint32_t m_ConfiguredReports            : 8  = 0;
        uint32_t m_IntermediateCmdFailuireCount : 4  = 0;
        uint32_t m_TotalFailureCount            : 4  = 0;
        uint32_t m_LastIndicationStatus         : 8  = 0;

        //2nd uint32_t
        uint32_t m_LastTimeoutTSN               : 8   = 0;
        uint32_t m_WaitForSendStatus            : 1   = 0;
        uint32_t m_WaitForResponse              : 1   = 0;
        uint32_t m_UnusedBits                   : 6   = 0;
        uint32_t m_LastESP_ERR                  : 16  = 0;


        uint32_t GetVal() const { return *(uint32_t*)this; }
        uint32_t GetVal2() const { return *((uint32_t*)this + 1); }

        void Update();
    };
    static_assert(sizeof(Internals) == sizeof(uint32_t) * 2);

    //initialized at start
    struct RuntimeState
    {
        esp_zb_ieee_addr_t m_CoordinatorIeee;
        bool m_FirstRun = true;
        bool m_LastPresence = false;
        bool m_TriggerAllowed = true;
        bool m_SuppressedByIllulminance = false;
        bool m_LastPresenceMMWave = false;
        bool m_LastPresencePIRInternal = false;
        bool m_LastPresenceExternal = false;
        LD2412::TargetState m_LastLD2412State = LD2412::TargetState::Clear;
        ld2412::Component::ExtendedState m_LastLD2412ExtendedState = ld2412::Component::ExtendedState::Normal;
        ZbAlarm m_RunningTimer{"m_RunningTimer"};
        ZbAlarm m_ExternalRunningTimer{"m_ExternalRunningTimer"};

        CmdWithRetries<ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_ID, 2>                m_OnSender{send_on_raw, nullptr, cmd_total_failure, cmd_failure, &m_OnSender};
        CmdWithRetries<ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID, 2>               m_OffSender{send_off_raw, nullptr, cmd_total_failure, cmd_failure, &m_OffSender};
        CmdWithRetries<ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_WITH_TIMED_OFF_ID, 2> m_OnTimedSender{send_on_timed_raw, nullptr, cmd_total_failure, cmd_failure, &m_OnTimedSender};

        uint16_t m_FailureCount = 0;
        uint16_t m_TotalFailureCount = 0;
        esp_zb_zcl_status_t m_LastFailedStatus = ESP_ZB_ZCL_STATUS_SUCCESS;
        enum class ExtendedFailureStatus: uint16_t
        {
            BindRequestIncomplete = 0xE0
        };

        static constexpr esp_zb_zcl_cluster_id_t g_RelevantBoundClusters[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF};

        Internals m_Internals;
        ZbAlarm m_BindsCheck{"BindsCheck"};

        BindArray m_TrackedBinds;
        BindArray m_BindsToCleanup;
        uint8_t m_ValidBinds = 0;
        uint8_t m_BindStates = 0;//on/off, bit per bind

        BindArray m_TempNewBinds;
        uint8_t m_FoundExisting = 0;

        TriState8Array m_BindsReportingCapable;
        struct{
            uint8_t m_InitialBindsChecking : 1 = true;
            uint8_t m_NeedBindsChecking    : 1= true;
            uint8_t m_FailedStatusUpdated  : 1= false;
        };

        uint8_t m_ExternalIlluminance = 0;

        bool CommandsToBindInFlight() const;
        bool CanSendCommandsToBind() const;
        uint8_t GetIlluminance() const;
        static bool IsRelevant(esp_zb_zcl_cluster_id_t id)
        {
            for(auto _i : g_RelevantBoundClusters)
                if (_i == id)
                    return true;
            return false;
        }

        void StartExternalTimer(esp_zb_user_callback_t cb, uint32_t time);
        void StartLocalTimer(esp_zb_user_callback_t cb, uint32_t time);
        void ScheduleBindsChecking();
        void RunBindsChecking();
        void RunService();
    };

    /**********************************************************************/
    /* Globals                                                            */
    /**********************************************************************/
    /**********************************************************************/
    /* LD2412 Component                                                   */
    /**********************************************************************/
    extern ld2412::Component g_ld2412;//THE presence sensor component

    /**********************************************************************/
    /* Storable data                                                      */
    /**********************************************************************/
    //storable configuration
    extern LocalConfig g_Config;


    /**********************************************************************/
    /* Runtime state                                                      */
    /**********************************************************************/
    extern RuntimeState g_State;

    extern const ZbCmdHandlingDesc g_CommandsDesc;
    extern const SetAttributesHandlingDesc g_AttributeHandlingDesc;
    extern const ReportAttributesHandlingDesc g_ReportHandlingDesc;
}
#endif
