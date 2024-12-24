#ifndef ZB_BINDS_HPP_
#define ZB_BINDS_HPP_

#include <cstddef>
#include <cstring>
#include "esp_zigbee_core.h"
#include "lib/misc_helpers.hpp"
#include "lib/object_pool.hpp"
#include "lib/array_count.hpp"
#include "zb_helpers/zbh_alarm.hpp"
#include "zigbee.hpp"

namespace zb
{
    struct BindInfo
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
        
        enum class State: uint8_t
        {
            New = 0,
            VerifyBinds,
            SendBindToMeReq,
            CheckConfigureReport,
            SendConfigureReport,
            NonFunctional,
            Functional,
            Unbind,
        };
        static constexpr uint16_t kMaxConfigAttempts = 3;
        static constexpr uint32_t kTimeout = 2000;
        BindInfo(esp_zb_ieee_addr_t &a, uint16_t sh):m_ShortAddr(sh)
        {
            std::memcpy(m_IEEE, a, sizeof(esp_zb_ieee_addr_t));
        }

        esp_zb_ieee_addr_t m_IEEE;
        uint16_t m_ShortAddr;
        struct{
            uint16_t m_EP : 8 = 0;
            uint16_t m_ReportConfigured: 1 = 0;
            uint16_t m_BoundToMe: 1 = 0;
            uint16_t m_BindChecked: 1 = 0;
            uint16_t m_AttemptsLeft: 3 = 0;
        };

        void Do();
        void Unbind();
        State GetState() const { return m_State; }
    private:
        State m_State = State::New;
        ZbAlarm m_Timer;
        ReadReportConfigNode m_ReadReportConfigNode;
        ConfigReportNode m_ConfigReportNode;
        static constexpr uint16_t kInvalidTSN = 0xffff;
        uint16_t m_LastTSN = kInvalidTSN;

        void TransitTo(State s);

        void SendBindRequest();
        static void OnBindRequestResult(esp_zb_zdp_status_t zdo_status, void *user_ctx);

        void SendUnBindRequest();
        static void OnUnBindRequestResult(esp_zb_zdp_status_t zdo_status, void *user_ctx);

        void GetBindTable();
        void OnGetBindTableFailed(esp_zb_zdp_status_t status);

        void CheckReportConfiguration();
        void SendReportConfiguration();

        static void OnSendReportConfigSendStatus(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx);
        static void OnConfigReportTimeout(void* param);

        static void OnReadReportConfigTimeout(void* param);
        static void OnGetBindTableChunk(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx);
        static void OnReadReportConfigSendStatus(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx);
    };
    constexpr size_t kMaxBinds = 6;
    using BindInfoPool = ObjectPool<BindInfo, kMaxBinds * 2>;
    extern BindInfoPool g_BindInfoPool;

    using BindInfoPtr = BindInfoPool::Ptr<g_BindInfoPool>;
    using BindArray = ArrayCount<BindInfoPtr, kMaxBinds>;
};
#endif
