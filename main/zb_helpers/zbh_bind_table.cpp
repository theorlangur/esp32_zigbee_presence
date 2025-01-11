#include "zbh_bind_table.hpp"
#include "lib_object_pool.hpp"
#include "zbh_types.hpp"

namespace zb
{
    struct BindTableIterationState
    {
        BindIteratorConfig config;
        uint16_t dstAddr;
    };

    using BindTablePool = ObjectPool<BindTableIterationState, kMaxBindTableRequests>;
    static BindTablePool g_BindTableIterationsPool;
    using BindTablePoolPtr = BindTablePool::Ptr<g_BindTableIterationsPool>;

    static void OnGetBindTableChunk(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx);
    void bind_table_iterate(uint16_t shortAddr, BindIteratorConfig config)
    {
        esp_zb_zdo_mgmt_bind_param_t cmd_req{};
        cmd_req.dst_addr = shortAddr;
        cmd_req.start_index = 0;

        auto *pState = g_BindTableIterationsPool.Acquire(config, shortAddr);
        if (config.debug)
            FMT_PRINT("Sending request to {:x} to get binds\n", cmd_req.dst_addr);
        esp_zb_zdo_binding_table_req(&cmd_req, OnGetBindTableChunk, pState);
    }

    static void OnGetBindTableChunk(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx)
    {
        BindTablePoolPtr ptr((BindTableIterationState*)user_ctx);
        if (ptr->config.debug)
            FMT_PRINT("OnGetBindTableChunk: status: {:x}\n", table_info->status);
        if (!table_info->index && ptr->config.on_begin)
        {
            if (!(ptr->config.on_begin)(table_info, ptr->config.pCtx))
                return;
        }

        if (table_info->status != esp_zb_zdp_status_t::ESP_ZB_ZDP_STATUS_SUCCESS)
        {
            //request failed
            if (ptr->config.on_error)
                (ptr->config.on_error)(table_info, ptr->config.pCtx);
            return;
        }

        if (ptr->config.on_entry)
        {
            esp_zb_ieee_addr_t coordinator;
            if (ptr->config.skipCoordinator)
                esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), coordinator);

            auto *pNext = table_info->record;
            while(pNext)
            {
                if (pNext->dst_addr_mode == ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED)
                {
                    if (!ptr->config.skipCoordinator || zb::ieee_addr{pNext->dst_address.addr_long} != zb::ieee_addr{coordinator})
                    {
                        if (!(ptr->config.on_entry)(pNext, ptr->config.pCtx))
                        {
                            //done
                            return;
                        }
                    }
                }
                pNext = pNext->next;
            }
        }

        if ((table_info->index + table_info->count) < table_info->total)
        {
            //next chunk
            esp_zb_zdo_mgmt_bind_param_t cmd_req;
            cmd_req.dst_addr = ptr->dstAddr;
            cmd_req.start_index = table_info->index + table_info->count;

            //FMT_PRINT("Sending request to {:x} to get binds\n", cmd_req.dst_addr);
            esp_zb_zdo_binding_table_req(&cmd_req, OnGetBindTableChunk, ptr.release());
        }else if (ptr->config.on_end)
            (ptr->config.on_end)(table_info, ptr->config.pCtx);
    }
}
