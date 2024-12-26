#include "zbh_cmd_sender.hpp"

namespace zb
{
    bool is_coordinator(esp_zb_zcl_addr_t &addr)
    {
        static esp_zb_ieee_addr_t g_MyIEEE;
        static bool init = false;
        if (!init)
        {
            init = true;
            esp_zb_get_long_address(g_MyIEEE);
        }

        if (addr.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
            return addr.u.short_addr == 0;
        if (addr.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
            return ieee_addr{addr.u.ieee_addr} == ieee_addr{g_MyIEEE};
        //anything else is not supported
        return false;
    }
}
