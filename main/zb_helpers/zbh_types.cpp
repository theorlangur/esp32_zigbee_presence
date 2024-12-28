#include "zbh_types.hpp"

namespace zb
{
    void InitHelpers()
    {
        (void)GetMyIEEE();
        esp_zb_zcl_addr_t dummy;
        (void)IsCoordinator(dummy);
    }

    bool IsCoordinator(esp_zb_zcl_addr_t &addr)
    {
        static esp_zb_ieee_addr_t g_Coordinator;
        [[maybe_unused]] static bool init = []{ esp_zb_ieee_address_by_short(0, g_Coordinator); return true;}();

        if (addr.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
            return addr.u.short_addr == 0;
        if (addr.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
            return ieee_addr{addr.u.ieee_addr} == ieee_addr{g_Coordinator};
        //anything else is not supported
        return false;
    }

    const esp_zb_ieee_addr_t& GetMyIEEE()
    {
        static esp_zb_ieee_addr_t g_MyIEEE;
        [[maybe_unused]] static bool init = []{ esp_zb_get_long_address(g_MyIEEE); return true;}();

        return g_MyIEEE;
    }
}
