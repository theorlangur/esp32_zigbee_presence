#ifndef ZB_HELPERS_HPP_
#define ZB_HELPERS_HPP_

#include "esp_zigbee_core.h"
#include <cstring>
#include <string_view>
#include <expected>

namespace zb
{
    template<size_t N>
    struct ZigbeeStr
    {
        char name[N];
        operator void*() { return name; }
        size_t size() const { return N - 1; }
        std::string_view sv() const { return {name + 1, N - 1}; }
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
        APILock() { esp_zb_lock_acquire(portMAX_DELAY); }
        ~APILock() { esp_zb_lock_release(); }
    };

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


    template<uint8_t EP, uint16_t ClusterID, uint8_t Role, uint16_t Attr, typename T>
    struct ZclAttributeAccess
    {
        using ZCLResult = std::expected<esp_zb_zcl_status_t, esp_zb_zcl_status_t>;
        using ESPResult = std::expected<esp_err_t, esp_err_t>;
        ZCLResult Set(T &v)
        {
            auto status = esp_zb_zcl_set_attribute_val(EP, ClusterID, Role, Attr, &v, false);
            if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
                return std::unexpected(status);
            return ESP_ZB_ZCL_STATUS_SUCCESS;
        }

        ESPResult Report(DestAddr addr = {})
        {
            esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
            report_attr_cmd.address_mode = addr.mode;
            report_attr_cmd.zcl_basic_cmd.dst_addr_u = addr.addr; //coordinator
            report_attr_cmd.zcl_basic_cmd.src_endpoint = EP;
            report_attr_cmd.zcl_basic_cmd.dst_endpoint = {};
            report_attr_cmd.attributeID = Attr;
            report_attr_cmd.cluster_role = Role;
            report_attr_cmd.clusterID = ClusterID;
            if (auto r = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd); r != ESP_OK)
                return std::unexpected(r);
            return ESP_OK;
        }

    };
}
#endif
