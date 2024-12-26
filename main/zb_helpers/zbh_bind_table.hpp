#ifndef ZBH_BIND_TABLE_HPP_
#define ZBH_BIND_TABLE_HPP_
#include "zdo/esp_zigbee_zdo_command.h"

namespace zb
{
    constexpr size_t kMaxBindTableRequests = 8;
    using bind_entry_callback_t = bool(*)(esp_zb_zdo_binding_table_record_t *pRec, void *pCtx);
    using bind_begin_callback_t = bool(*)(const esp_zb_zdo_binding_table_info_t *table_info, void *pCtx);
    using bind_end_callback_t = void(*)(const esp_zb_zdo_binding_table_info_t *table_info, void *pCtx);
    using bind_error_callback_t = void(*)(const esp_zb_zdo_binding_table_info_t *table_info, void *pCtx);

    struct BindIteratorConfig
    {
        void *pCtx = nullptr;
        bind_entry_callback_t on_entry = nullptr;
        bind_begin_callback_t on_begin = nullptr;
        bind_end_callback_t on_end = nullptr;
        bind_error_callback_t on_error = nullptr;
        bool skipCoordinator = true;
        bool debug = false;
    };

    void bind_table_iterate(uint16_t shortAddr, BindIteratorConfig config);
}
#endif
