#ifndef ZBH_TYPES_HPP_
#define ZBH_TYPES_HPP_

#include "esp_zigbee_core.h"
#include <cstring>
#include <string_view>
#include <span>
#include <expected>
#include <utility>
#include "lib_misc_helpers.hpp"

namespace zb
{
    void InitHelpers();
    bool IsCoordinator(esp_zb_zcl_addr_t &addr);
    const esp_zb_ieee_addr_t& GetMyIEEE();

    struct ZigbeeStrView
    {
        char *pStr;

        operator void*() { return pStr; }
        uint8_t size() const { return pStr[0]; }
        std::string_view sv() const { return {pStr + 1, pStr[0]}; }
    };

    struct ZigbeeStrRef
    {
        char sz;

        operator void*() { return this; }
        uint8_t size() const { return sz; }
        std::string_view sv() const { return {&sz + 1, sz}; }
    };

    template<size_t N>
    struct ZigbeeStrBuf: public ZigbeeStrRef
    {
        char data[N];
    };

    struct ZigbeeOctetRef
    {
        uint8_t sz;

        operator void*() { return this; }
        uint8_t size() const { return sz; }
        std::span<const uint8_t> sv() const { return {&sz + 1, sz}; }
    };

    template<size_t N>
    struct ZigbeeOctetBuf: public ZigbeeOctetRef
    {
        uint8_t data[N];
    };

    template<size_t N>
    struct ZigbeeStr
    {
        char name[N];
        operator void*() { return name; }
        size_t size() const { return N - 1; }
        std::string_view sv() const { return {name + 1, N - 1}; }
        ZigbeeStrView zsv() const { return {name}; }
        ZigbeeStrRef& zsv_ref() { return *(ZigbeeStrRef*)name; }
    };


    template<size_t N>
    constexpr ZigbeeStr<N> ZbStr(const char (&n)[N])
    {
        static_assert(N < 255, "String too long");
        return [&]<size_t...idx>(std::index_sequence<idx...>){
            return ZigbeeStr<N>{.name={N-1, n[idx]...}};
        }(std::make_index_sequence<N-1>());
    }

    struct APILock
    {
        static bool g_State;
        APILock() { esp_zb_lock_acquire(portMAX_DELAY); g_State = true; }
        ~APILock() { g_State = false; esp_zb_lock_release(); }
    };
    inline bool APILock::g_State = false;

    struct DestAddr
    {
        esp_zb_zcl_address_mode_t mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;         /*!< APS addressing mode constants refer to esp_zb_zcl_address_mode_t */
        esp_zb_addr_u addr;

        constexpr DestAddr() = default;
        constexpr DestAddr(uint16_t shortAddr): 
            mode(ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT)
            , addr{.addr_short = shortAddr}
        {}

        DestAddr(esp_zb_ieee_addr_t ieeeAddr): 
            mode(ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT)
        {
            std::memcpy(addr.addr_long, ieeeAddr, sizeof(esp_zb_ieee_addr_t));
        }
    };
    constexpr static DestAddr g_DestCoordinator{uint16_t(0)};

    struct ieee_addr
    {
        const esp_zb_ieee_addr_t &a;
        bool operator==(ieee_addr const& rhs) const { return std::memcmp(a, rhs.a, sizeof(esp_zb_ieee_addr_t)) == 0; }
        friend bool operator==(const esp_zb_ieee_addr_t &lhs, ieee_addr const& rhs) { return std::memcmp(lhs, rhs.a, sizeof(esp_zb_ieee_addr_t)) == 0; }
        friend bool operator==(ieee_addr const& rhs, const esp_zb_ieee_addr_t &lhs) { return lhs == rhs; }
    };
}

template<>
struct tools::formatter_t<esp_zb_zcl_addr_t>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, esp_zb_zcl_addr_t const& a)
    {
        if (a.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
            return tools::format_to(std::forward<Dest>(dst), "[short={:x}]" , a.u.short_addr);
        if (a.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
            return tools::format_to(std::forward<Dest>(dst), "[ieee={}]" , a.u.ieee_addr);
        return tools::format_to(std::forward<Dest>(dst), "[src_id={:x}]" , a.u.src_id);
    }
};

#endif
