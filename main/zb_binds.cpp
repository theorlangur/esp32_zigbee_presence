#include "zigbee.hpp"
#include "zb_binds.hpp"
#include "zb_helpers/zbh_types.hpp"

namespace zb
{
    namespace{
        static constexpr esp_zb_zcl_cluster_id_t g_RelevantBoundClusters[] = {ESP_ZB_ZCL_CLUSTER_ID_ON_OFF};
        static bool IsRelevant(esp_zb_zcl_cluster_id_t id)
        {
            for(auto _i : g_RelevantBoundClusters)
                if (_i == id)
                    return true;
            return false;
        }
    }

    BindInfoPool g_BindInfoPool;
    void BindInfo::Do()
    {
        switch(m_State)
        {
            case State::New:
                {
                    TransitTo(State::VerifyBinds);
                    return Do();
                }
                break;
            case State::VerifyBinds:
                {
                    return GetBindTable();
                }
                break;
            case State::SendBindToMeReq:
                {
                }
                break;
            case State::CheckConfigureReport:
                {
                }
                break;
            case State::SendConfigureReport:
                {
                }
                break;
            case State::FailedToConfigure:
                {
                    //we're done
                }
                break;
        }
    }

    void BindInfo::TransitTo(State s)
    {
        m_State = s;
    }

    void BindInfo::SendBindRequest()
    {
        esp_zb_zdo_bind_req_param_t bind_req;
        bind_req.req_dst_addr = m_ShortAddr;
        std::memcpy(bind_req.src_address, m_IEEE, sizeof(esp_zb_ieee_addr_t));
        bind_req.src_endp = m_EP;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        esp_zb_get_long_address(bind_req.dst_address_u.addr_long);
        bind_req.dst_endp = PRESENCE_EP;
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;

        esp_zb_zdo_device_bind_req(&bind_req, OnBindRequestResult, this);
    }

    void BindInfo::OnBindRequestResult(esp_zb_zdp_status_t zdo_status, void *user_ctx)
    {
        //TODO: implement
    }

    void BindInfo::OnGetBindTableFailed(esp_zb_zdp_status_t status)
    {
        TransitTo(State::FailedToConfigure);
        m_ReportConfigured = false;
        m_BoundToMe = false;
    }

    void BindInfo::OnGetBindTableChunk(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx)
    {
        BindInfo *pBind = (BindInfo *)user_ctx;
        if (!g_BindInfoPool.IsValid(pBind)) return;//we're dead

        if (esp_zb_zdp_status_t(table_info->status) != esp_zb_zdp_status_t::ESP_ZB_ZDP_STATUS_SUCCESS)
            return pBind->OnGetBindTableFailed(esp_zb_zdp_status_t(table_info->status));

        esp_zb_ieee_addr_t me;
        esp_zb_get_long_address(me);
        auto *pNext = table_info->record;
        while(pNext)
        {
            if (pNext->dst_addr_mode == ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED)
            {
                if (zb::ieee_addr{pNext->dst_address.addr_long} == zb::ieee_addr{me})
                {
                    //ok, it's me
                    //right cluster?
                    if (IsRelevant(esp_zb_zcl_cluster_id_t(pNext->cluster_id)))//got it. at least one
                    {
                        //no need to search further
                        //need to check if reporting is properly configured
                        pBind->TransitTo(State::CheckConfigureReport);
                        return pBind->Do();
                    }
                }
            }
            pNext = pNext->next;
        }

        if ((table_info->index + table_info->count) < table_info->total)
        {
            //there are more binds
            esp_zb_zdo_mgmt_bind_param_t cmd_req;
            cmd_req.dst_addr = pBind->m_ShortAddr;
            cmd_req.start_index = table_info->index + table_info->count;

            FMT_PRINT("Sending request (next chunk, index {}) to {:x} to get binds\n", cmd_req.start_index, cmd_req.dst_addr);
            return esp_zb_zdo_binding_table_req(&cmd_req, OnGetBindTableChunk, pBind);
        }
        //we're done and apparently no bind to ourselfs found
        pBind->TransitTo(State::SendBindToMeReq);
        return pBind->Do();
    }

    void BindInfo::GetBindTable()
    {
        esp_zb_zdo_mgmt_bind_param_t cmd_req;
        cmd_req.dst_addr = m_ShortAddr;
        cmd_req.start_index = 0;

        FMT_PRINT("Sending request to {:x} to get binds\n", cmd_req.dst_addr);
        esp_zb_zdo_binding_table_req(&cmd_req, OnGetBindTableChunk, this);
    }
}
