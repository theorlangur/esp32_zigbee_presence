#include "zb_binds.hpp"

namespace zb
{
    BindInfoPool g_BindInfoPool;
    void BindInfo::Do()
    {
        switch(m_State)
        {
            case State::New:
                {
                }
                break;
            case State::VerifyBinds:
                {
                }
                break;
            case State::SendBindToMeReq:
                {
                }
                break;
            case State::SendConfigureReport:
                {
                }
                break;
            case State::FailedToConfigure:
                {
                }
                break;
        }
    }

    void BindInfo::GetBindTable()
    {
        esp_zb_zdo_mgmt_bind_param_t cmd_req;
        cmd_req.dst_addr = m_ShortAddr;
        cmd_req.start_index = 0;

        FMT_PRINT("Sending request to {:x} to get binds\n", cmd_req.dst_addr);
        esp_zb_zdo_binding_table_req(&cmd_req, 
            [](const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx)
            {
                BindInfo *pBind = (BindInfo *)user_ctx;
                if (!g_BindInfoPool.IsValid(pBind)) return;//we're dead


                //TODO: implement
            }, this);
    }
}
