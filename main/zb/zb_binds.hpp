#ifndef ZB_BINDS_HPP_
#define ZB_BINDS_HPP_

#include <cstddef>
#include <cstring>
#include "esp_zigbee_core.h"
#include "lib_misc_helpers.hpp"
#include "lib_object_pool.hpp"
#include "lib_array_count.hpp"
#include "zbh_alarm.hpp"
#include "zbh_cmd_sender.hpp"
#include "zb_main.hpp"

namespace zb
{
    struct BindInfo: NonMovable, NonCopyable
    {
        struct ReadReportConfigNode: ReadConfigResponseNode
        {
            BindInfo* GetBindInfo();
            virtual bool Notify(esp_zb_zcl_cmd_read_report_config_resp_message_t *pResp) override;
        };

        struct ConfigReportNode: ConfigReportResponseNode
        {
            //return 'true' - was handled, no need to keep iterating
            BindInfo* GetBindInfo();
            virtual bool Notify(esp_zb_zcl_cmd_config_report_resp_message_t *pResp) override;
        };

        struct ReadAttrRespNode: ReadAttrResponseNode
        {
            //return 'true' - was handled, no need to keep iterating
            BindInfo* GetBindInfo();
            virtual bool Notify(esp_zb_zcl_cmd_read_attr_resp_message_t *pResp) override;
        };
        
        enum class State: uint8_t
        {
            New = 0,
            VerifyBinds,
            SendBindToMeReq,
            CheckConfigureReport,
            SendConfigureReport,
            CheckReportingAbility,
            TryReadAttribute,
            NonFunctional,
            Functional,
            Unbind,
        };
        static constexpr uint16_t kMaxConfigAttempts = 3;
        static constexpr uint32_t kTimeout = 2000;
        BindInfo(esp_zb_ieee_addr_t const &a, uint16_t sh):m_ShortAddr(sh)
        {
            std::memcpy(m_IEEE, a, sizeof(esp_zb_ieee_addr_t));
            m_SendStatusNode.user_ctx = this;
        }

        esp_zb_ieee_addr_t m_IEEE;
        uint16_t m_ShortAddr;
        struct{
            uint16_t m_EP              : 8 = 0;
            uint16_t m_ReportConfigured: 1 = 0;
            uint16_t m_BoundToMe       : 1 = 0;
            uint16_t m_BindChecked     : 1 = 0;
            uint16_t m_AttemptsLeft    : 2 = 0;
            uint16_t m_InitialValue    : 1 = 0;//valid only if m_Initial is true
            uint16_t m_Initial         : 1 = 1;
            uint16_t m_CheckReporting  : 1 = 0;
        };

        void Do();
        void Unbind();
        void Failed();
        void RunCheckIfRequested();
        bool IsPassive() const;
        State GetState() const { return m_State; }

        void OnReport(const esp_zb_zcl_report_attr_message_t *pReport);
    private:
        static zb::seq_nr_t SendTryOnOffCmd(void*);
        static void OnTryOnOffSuccess(void*);
        static void OnTryOnOffFail(void*, esp_zb_zcl_status_t, esp_err_t);

        State m_State = State::New;
        ZbAlarm m_Timer;
        ReadReportConfigNode m_ReadReportConfigNode;
        ConfigReportNode m_ConfigReportNode;
        ReadAttrRespNode m_ReadAttrNode;
        CmdWithRetries<ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_ID, 2> m_TryReportCmd{SendTryOnOffCmd, OnTryOnOffSuccess, OnTryOnOffFail, nullptr, this};
        ZbCmdSend::Node m_SendStatusNode;

        void TransitTo(State s);

        void SendCmdToSetInitialValue();
        void SendBindRequest();
        static void OnBindRequestResult(esp_zb_zdp_status_t zdo_status, void *user_ctx);

        void SendUnBindRequest();
        static void OnUnBindRequestResult(esp_zb_zdp_status_t zdo_status, void *user_ctx);

        void GetBindTable();
        void OnGetBindTableFailed(esp_zb_zdp_status_t status);
        static bool OnBindTableBegin(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx);
        static bool OnGetBindTableChunk(esp_zb_zdo_binding_table_record_t *entry, void *user_ctx);
        static void OnBindTableFinished(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx);
        static void OnBindTableFailure(const esp_zb_zdo_binding_table_info_t *table_info, void *pCtx);

        void CheckReportingAbility();
        void ReadAttribute();
        void CheckReportConfiguration();
        void SendReportConfiguration();

        template<State expectedState>
        static void OnTimeout(void* param);
        template<State expectedState>
        static void OnSendStatus(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx);
    };
    constexpr size_t kMaxBinds = 6;
    using BindInfoPool = ObjectPool<BindInfo, kMaxBinds * 2>;
    extern BindInfoPool g_BindInfoPool;

    using BindInfoPtr = BindInfoPool::Ptr<g_BindInfoPool>;
    using BindArray = ArrayCount<BindInfoPtr, kMaxBinds>;
};
#endif
