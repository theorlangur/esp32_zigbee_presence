#ifndef ZB_BINDS_HPP_
#define ZB_BINDS_HPP_

#include <cstddef>
#include <cstring>
#include "esp_zigbee_core.h"
#include "lib/misc_helpers.hpp"
#include "lib/object_pool.hpp"
#include "lib/array_count.hpp"

namespace zb
{
    struct BindInfo
    {
        enum class State: uint8_t
        {
            New = 0,
            VerifyBinds,
            SendBindToMeReq,
            CheckConfigureReport,
            SendConfigureReport,
            FailedToConfigure,
        };
        static constexpr uint16_t kMaxConfigAttempts = 3;
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
        State m_State = State::New;

        void Do();
    private:
        void TransitTo(State s);

        void SendBindRequest();

        void GetBindTable();
        void OnGetBindTableFailed(esp_zb_zdp_status_t status);

        static void OnBindRequestResult(esp_zb_zdp_status_t zdo_status, void *user_ctx);
        static void OnGetBindTableChunk(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx);
    };
    constexpr size_t kMaxBinds = 6;
    using BindInfoPool = ObjectPool<BindInfo, kMaxBinds * 2>;
    extern BindInfoPool g_BindInfoPool;

    using BindInfoPtr = BindInfoPool::Ptr<g_BindInfoPool>;
    using BindArray = ArrayCount<BindInfoPtr, kMaxBinds>;
};
#endif
