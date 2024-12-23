#ifndef ZB_BINDS_HPP_
#define ZB_BINDS_HPP_

#include <cstddef>
#include <cstring>
#include "esp_zigbee_core.h"
#include "generic_helpers.hpp"

namespace zb
{
    struct BindInfo
    {
        enum class State: uint8_t
        {
            New = 0,
            VerifyBinds = 1,
            SendBindToMeReq = 2,
            SendConfigureReport = 3,
            FailedToConfigure = 4,
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
        void GetBindTable();
    };
    constexpr size_t kMaxBinds = 6;
    using BindInfoPool = ObjectPool<BindInfo, kMaxBinds>;
    extern BindInfoPool g_BindInfoPool;

    using BindInfoPtr = BindInfoPool::Ptr<g_BindInfoPool>;
    using BindArray = ArrayCount<BindInfoPtr, kMaxBinds>;
};
#endif
