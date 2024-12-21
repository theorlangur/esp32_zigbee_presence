#ifndef ZBH_ACTION_HANDLERS_HPP_
#define ZBH_ACTION_HANDLERS_HPP_

#include "zbh_types.hpp"

namespace zb
{
    /**********************************************************************/
    /* generic_zb_action_handler                                          */
    /**********************************************************************/
    struct ActionHandler
    {
        using callback_t = esp_err_t(*)(const void *message);
        esp_zb_core_action_callback_id_t id;
        callback_t cb;
    };

    template<ActionHandler... handlers>
    esp_err_t generic_zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
    {
        esp_err_t res = ESP_OK;
        bool handled = (([&]{ 
                    if (handlers.id == callback_id) 
                    {
                        res = handlers.cb(message);
                        return true; 
                    }
                    return false;}())||...);
        if (!handled)
        {
#ifndef NDEBUG
            //using clock_t = std::chrono::system_clock;
            //auto now = clock_t::now();
            //auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
            FMT_PRINT("task={}; lock={} Receive Zigbee action({:x}) callback\n", (const char*)pcTaskGetName(nullptr), APILock::g_State, (int)callback_id);
#endif
        }
        return res;
    }
}

#endif
