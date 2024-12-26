#include "zigbee.hpp"
#include "zb_binds.hpp"
#include "zb_helpers/zbh_types.hpp"
#include "zb_helpers/zbh_handlers_cmd.hpp"
#include "zb_helpers/zbh_bind_table.hpp"


template<>
struct tools::formatter_t<zb::BindInfo::State>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, zb::BindInfo::State const& p)
    {
        const char *pStr = "<unk>";
        switch(p)
        {
            case zb::BindInfo::State::New: pStr = "New"; break;
            case zb::BindInfo::State::VerifyBinds: pStr = "VerifyBinds"; break;
            case zb::BindInfo::State::SendBindToMeReq: pStr = "SendBindToMeReq"; break;
            case zb::BindInfo::State::CheckConfigureReport: pStr = "CheckConfigureReport"; break;
            case zb::BindInfo::State::SendConfigureReport: pStr = "SendConfigureReport"; break;
            case zb::BindInfo::State::CheckReportingAbility: pStr = "CheckReportingAbility"; break;
            case zb::BindInfo::State::TryReadAttribute: pStr = "TryReadAttribute"; break;
            case zb::BindInfo::State::NonFunctional: pStr = "NonFunctional"; break;
            case zb::BindInfo::State::Functional: pStr = "Functional"; break;
            case zb::BindInfo::State::Unbind: pStr = "Unbind"; break;
        }
        return tools::format_to(std::forward<Dest>(dst), "{}", pStr);
    }
};

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

    void BindInfo::Unbind()
    {
        TransitTo(State::Unbind);
        Do();
    }

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
            case State::VerifyBinds: return GetBindTable();
            case State::SendBindToMeReq: return SendBindRequest();
            case State::CheckConfigureReport: return CheckReportConfiguration();
            case State::SendConfigureReport: return SendReportConfiguration();
            case State::TryReadAttribute: return ReadAttribute();
            case State::CheckReportingAbility: return CheckReportingAbility();
            case State::NonFunctional:
                {
                    //we're done
                }
                break;
            case State::Functional:
                {
                    //we're done
                }
                break;
            case State::Unbind: return SendUnBindRequest();
        }
    }

    void BindInfo::TransitTo(State s)
    {
        if (m_LastTSN != kInvalidTSN)
            ZbCmdSend::Unregister(m_LastTSN);

        m_Timer.Cancel();
        m_ConfigReportNode.RemoveFromList();
        m_ReadReportConfigNode.RemoveFromList();
        m_ReadAttrNode.RemoveFromList();

        m_State = s;
    }

    template<BindInfo::State expectedState>
    void BindInfo::OnTimeout(void* param)
    {
        BindInfo *pBind = static_cast<BindInfo *>(param);
        if (!g_BindInfoPool.IsValid(pBind)) 
        {
            FMT_PRINT("Timeout for {} happened but BindInfo {:x} is dead\n", expectedState, param);
            return;//we're dead
        }
        
        FMT_PRINT("Timeout happened at state {} for {:x}\n", expectedState, pBind->m_ShortAddr);

        if (pBind->m_State != expectedState)
        {
            FMT_PRINT("({:x}-{})Unexpected state: {:x}\n", pBind->m_ShortAddr, expectedState, pBind->m_State);
            return;
        }

        if (pBind->m_LastTSN != kInvalidTSN)
        {
            ZbCmdSend::Unregister(uint8_t(pBind->m_LastTSN));
            pBind->m_LastTSN = kInvalidTSN;
        }
        pBind->m_ReportConfigured = false;

        if (pBind->m_AttemptsLeft--)
        {
            FMT_PRINT("Making another attempt. {} left\n", (int)pBind->m_AttemptsLeft);
            pBind->Do();
            return;
        }
        pBind->TransitTo(State::NonFunctional);
        return;
    }

    template<BindInfo::State expectedState>
    void BindInfo::OnSendStatus(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx)
    {
        BindInfo *pBind = static_cast<BindInfo *>(user_ctx);
        FMT_PRINT("({})Got send status callback for {:x} for TSN {:x}. Status {:x}\n", expectedState, user_ctx, pSendStatus->tsn, pSendStatus->status);
        if (!g_BindInfoPool.IsValid(pBind)) 
        {
            FMT_PRINT("Target BindInfo object is dead\n");
            return;//we're dead
        }

        if (pBind->m_State != State::TryReadAttribute)
        {
            FMT_PRINT("({:x}-{})Unexpected state: {:x}\n", pBind->m_ShortAddr, expectedState, pBind->m_State);
            return;
        }

        pBind->m_LastTSN = kInvalidTSN;
        if (!pBind->m_Timer.IsRunning())//timeout already happened?
        {
            FMT_PRINT("({:x}-{})Timeout has already happened. (should not be reachable)\n", pBind->m_ShortAddr, expectedState);
            return;
        }

        if (pSendStatus->status != ESP_OK)
        {
            FMT_PRINT("({:x})Failed to read attribute\n", pBind->m_ShortAddr);
            pBind->TransitTo(State::NonFunctional);
            return;
        }
    }

    void BindInfo::CheckReportingAbility()
    {
        //TODO: implement
    }

    void BindInfo::OnReport(const esp_zb_zcl_report_attr_message_t *pReport)
    {
        //TODO: react to a report
    }

    void BindInfo::ReadAttribute()
    {
        esp_zb_zcl_read_attr_cmd_t read_req = {};
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;//ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        read_req.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        read_req.zcl_basic_cmd.dst_endpoint = m_EP;
        std::memcpy(read_req.zcl_basic_cmd.dst_addr_u.addr_long, m_IEEE, sizeof(esp_zb_ieee_addr_t));
        //read_req.zcl_basic_cmd.dst_addr_u.addr_long = m_EP;
        read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        uint16_t attributes[] = { ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, };
        read_req.attr_number = std::size(attributes);
        read_req.attr_field = attributes;
        m_LastTSN = esp_zb_zcl_read_attr_cmd_req(&read_req);
        m_ReadAttrNode.RegisterSelf();
        m_Timer.Setup(OnTimeout<State::TryReadAttribute>, this, kTimeout);
        ZbCmdSend::Register(m_LastTSN, OnSendStatus<State::TryReadAttribute>, this);
    }

    BindInfo* BindInfo::ReadAttrRespNode::GetBindInfo()
    {
        uint8_t *pRaw = (uint8_t*)this;
        return (BindInfo*)(pRaw - offsetof(BindInfo, m_ReadAttrNode));
    }

    bool BindInfo::ReadAttrRespNode::Notify(esp_zb_zcl_cmd_read_attr_resp_message_t *pResp)
    {
        BindInfo *pBind = GetBindInfo();
        if (pResp->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
        {
            if (pResp->info.src_address.u.short_addr != pBind->m_ShortAddr)
                return false;
        }else if (pResp->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
        {
            if (ieee_addr{pResp->info.src_address.u.ieee_addr} != ieee_addr{pBind->m_IEEE})
                return false;
        }

        FMT_PRINT("Got read attr response from {}\n", pResp->info.src_address);
        pBind->m_ReadAttrNode.RemoveFromList();
        if (pBind->m_Timer.IsRunning())
        {
            pBind->m_Timer.Cancel();
        }
        else //timer already cancelled, this report is outdated
        {
            FMT_PRINT("Timeout already happened. We should never reach here\n");
            return false;
        }

        bool found = false, value = false;
        auto *pVar = pResp->variables;
        while(pVar)
        {
            if (pVar->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
            {
                FMT_PRINT("({:x})Got attribute {:x}; Status {:x}\n", pBind->m_ShortAddr, pVar->attribute.id, pVar->status);
                if (pVar->status == ESP_ZB_ZCL_STATUS_SUCCESS)
                {
                    found = true;
                    value = *(bool*)pVar->attribute.data.value;
                    break;
                }
            }
            //FMT_PRINT("Attr[{:x}]: status={:x} dir={}\n", pVar->attribute_id, pVar->status, pVar->report_direction);
            pVar = pVar->next;
        }

        if (found)
        {
            FMT_PRINT("({:x})Read Attribute done. Attribute found, value {}. All good.\n", pBind->m_ShortAddr, value);
            pBind->m_ReportConfigured = true;
            pBind->m_InitialValue = value;
            pBind->TransitTo(State::Functional);
        }else
            pBind->TransitTo(State::NonFunctional);
        return true;
    }

    BindInfo* BindInfo::ReadReportConfigNode::GetBindInfo()
    {
        uint8_t *pRaw = (uint8_t*)this;
        return (BindInfo*)(pRaw - offsetof(BindInfo, m_ReadReportConfigNode));
    }

    bool BindInfo::ReadReportConfigNode::Notify(esp_zb_zcl_cmd_read_report_config_resp_message_t *pResp)
    {
        BindInfo *pBind = GetBindInfo();
        if (pResp->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
        {
            if (pResp->info.src_address.u.short_addr != pBind->m_ShortAddr)
                return false;
        }else if (pResp->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
        {
            if (ieee_addr{pResp->info.src_address.u.ieee_addr} != ieee_addr{pBind->m_IEEE})
                return false;
        }

        FMT_PRINT("Got read report config response from {}\n", pResp->info.src_address);
        pBind->m_ReadReportConfigNode.RemoveFromList();
        if (pBind->m_Timer.IsRunning())
        {
            pBind->m_Timer.Cancel();
        }
        else //timer already cancelled, this report is outdated
        {
            FMT_PRINT("Timeout already happened. We should never reach here\n");
            return false;
        }

        bool found = false;
        auto *pVar = pResp->variables;
        while(pVar)
        {
            if (pVar->attribute_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
            {
                FMT_PRINT("({:x})Got attribute {:x}; Status {:x}\n", pBind->m_ShortAddr, pVar->attribute_id, pVar->status);
                if (pVar->status == ESP_ZB_ZCL_STATUS_SUCCESS)
                {
                    found = true;
                    break;
                }
            }
            //FMT_PRINT("Attr[{:x}]: status={:x} dir={}\n", pVar->attribute_id, pVar->status, pVar->report_direction);
            pVar = pVar->next;
        }

        if (found)
        {
            FMT_PRINT("({:x})Attribute reporting config found. All good.\n", pBind->m_ShortAddr);
            pBind->m_ReportConfigured = true;
            //pBind->TransitTo(State::Functional);
            pBind->TransitTo(State::TryReadAttribute);
            pBind->Do();
        }else
        {
            FMT_PRINT("({:x})Attribute reporting config NOT found. Attempting to send a configuration request.\n", pBind->m_ShortAddr);
            pBind->TransitTo(State::SendConfigureReport);
            pBind->Do();
        }
        return true;
    }

    BindInfo* BindInfo::ConfigReportNode::GetBindInfo()
    {
        uint8_t *pRaw = (uint8_t*)this;
        return (BindInfo*)(pRaw - offsetof(BindInfo, m_ConfigReportNode));
    }

    bool BindInfo::ConfigReportNode::Notify(esp_zb_zcl_cmd_config_report_resp_message_t *pResp)
    {
        BindInfo *pBind = GetBindInfo();
        if (pResp->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
        {
            if (pResp->info.src_address.u.short_addr != pBind->m_ShortAddr)
                return false;
        }else if (pResp->info.src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
        {
            if (ieee_addr{pResp->info.src_address.u.ieee_addr} != ieee_addr{pBind->m_IEEE})
                return false;
        }

        FMT_PRINT("({:x})Got config report response from {}\n", pBind->m_ShortAddr, pResp->info.src_address);
        pBind->m_ConfigReportNode.RemoveFromList();
        if (pBind->m_Timer.IsRunning())
        {
            pBind->m_Timer.Cancel();
        }
        else //timer already cancelled, this report is outdated
        {
            FMT_PRINT("{:x}Timeout already happened. We should never reach here\n", pBind->m_ShortAddr);
            return false;
        }

        if (pResp->info.status == esp_zb_zcl_status_t::ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            FMT_PRINT("({:x})Attribute reporting configured. All good.\n", pBind->m_ShortAddr);
            //pBind->m_ReportConfigured = true;
            //pBind->TransitTo(State::Functional);
            pBind->TransitTo(State::TryReadAttribute);
            pBind->Do();
        }else
        {
            FMT_PRINT("({:x})Attribute reporting could not be configured. Status: {:x}\n", pBind->m_ShortAddr, pResp->info.status);
            pBind->TransitTo(State::NonFunctional);
            pBind->Do();
        }
        return true;
    }

    void BindInfo::SendReportConfiguration()
    {
        /* Send "configure report attribute" command to the bound sensor */
        esp_zb_zcl_config_report_cmd_t report_cmd = {};
        report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = m_ShortAddr;
        report_cmd.zcl_basic_cmd.dst_endpoint = m_EP;
        report_cmd.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;

        int16_t report_change = 1;
        esp_zb_zcl_config_report_record_t records[] = {
            {
                .direction = ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
                .attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                .attrType = ESP_ZB_ZCL_ATTR_TYPE_BOOL,
                .min_interval = 0,
                .max_interval = 3600,
                .reportable_change = &report_change,
            },
        };
        report_cmd.record_number = 1;//std::size(records);
        report_cmd.record_field = records;
        m_LastTSN = esp_zb_zcl_config_report_cmd_req(&report_cmd);
        FMT_PRINT("Sent configure report request to {:x}; TSN={:x}.\n", m_ShortAddr, m_LastTSN);
        m_ConfigReportNode.RegisterSelf();
        m_Timer.Setup(OnTimeout<State::SendConfigureReport>, this, kTimeout);
        ZbCmdSend::Register(m_LastTSN, OnSendStatus<State::SendConfigureReport>, this);
        //FMT_PRINT("Sent config report for On/Off; TSN={:x}\n", tsn);
    }

    void BindInfo::CheckReportConfiguration()
    {
        esp_zb_zcl_read_report_config_cmd_t rr = {};
        rr.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        rr.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
        rr.manuf_code = 0;
        rr.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        rr.zcl_basic_cmd.dst_endpoint = m_EP;
        rr.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        rr.zcl_basic_cmd.dst_addr_u.addr_short = m_ShortAddr;
        rr.record_number = 1;
        esp_zb_zcl_attribute_record_t on_off_r;
        on_off_r.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
        on_off_r.report_direction = 0;
        rr.record_field = &on_off_r;
        m_LastTSN = esp_zb_zcl_read_report_config_cmd_req(&rr);
        FMT_PRINT("Sent read configure request to {:x}; TSN={:x}.\n", m_ShortAddr, m_LastTSN);
        m_ReadReportConfigNode.RegisterSelf();
        m_Timer.Setup(OnTimeout<State::CheckConfigureReport>, this, kTimeout);
        ZbCmdSend::Register(m_LastTSN, OnSendStatus<State::CheckConfigureReport>, this);
    }

    void BindInfo::SendUnBindRequest()
    {
        m_AttemptsLeft = kMaxConfigAttempts;
        esp_zb_zdo_bind_req_param_t bind_req = {};
        bind_req.req_dst_addr = m_ShortAddr;
        std::memcpy(bind_req.src_address, m_IEEE, sizeof(esp_zb_ieee_addr_t));
        bind_req.src_endp = m_EP;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        esp_zb_get_long_address(bind_req.dst_address_u.addr_long);
        bind_req.dst_endp = PRESENCE_EP;
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;

        esp_zb_zdo_device_unbind_req(&bind_req, OnUnBindRequestResult, this);
        FMT_PRINT("({:x})Sent un-bind request\n", m_ShortAddr);
    }

    void BindInfo::OnUnBindRequestResult(esp_zb_zdp_status_t zdo_status, void *user_ctx)
    {
        BindInfo *pBind = static_cast<BindInfo *>(user_ctx);
        if (!g_BindInfoPool.IsValid(pBind)) 
        {
            FMT_PRINT("({:x})Got UnBind request result, but BindInfo is dead\n", user_ctx);
            return;//we're dead
        }

        if (pBind->m_State != State::Unbind)
        {
            FMT_PRINT("({:x})(Unbind resp)Unexpected state: {:x}\n", pBind->m_ShortAddr, pBind->m_State);
            return;
        }

        if (zdo_status == ESP_ZB_ZDP_STATUS_TIMEOUT)
        {
            if (pBind->m_AttemptsLeft--)
            {
                FMT_PRINT("({:x})Got Bind request result {:x}. Failed. Starting another attempt (left: {})\n", pBind->m_ShortAddr, zdo_status, (int)pBind->m_AttemptsLeft);
                pBind->m_Timer.Setup(
                        [](void *user_ctx){
                            BindInfo *pBind = static_cast<BindInfo *>(user_ctx);
                            if (!g_BindInfoPool.IsValid(pBind)) 
                            {
                                FMT_PRINT("(Re-try attempt)({:x})(Bind request)BindInfo is dead\n", user_ctx);
                                pBind->SendBindRequest();
                                return;//we're dead
                            }
                        }, pBind, kTimeout);
                return;
            }
        }

        pBind->m_ReportConfigured = false;
        pBind->m_BoundToMe = false;
        FMT_PRINT("({:x})Got UnBind request result {:x}\n", pBind->m_ShortAddr, zdo_status);
        pBind->TransitTo(State::NonFunctional);
        pBind->Do();
    }
    
    void BindInfo::SendBindRequest()
    {
        m_AttemptsLeft = kMaxConfigAttempts;
        esp_zb_zdo_bind_req_param_t bind_req = {};
        bind_req.req_dst_addr = m_ShortAddr;
        std::memcpy(bind_req.src_address, m_IEEE, sizeof(esp_zb_ieee_addr_t));
        bind_req.src_endp = m_EP;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        esp_zb_get_long_address(bind_req.dst_address_u.addr_long);
        bind_req.dst_endp = PRESENCE_EP;
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;

        esp_zb_zdo_device_bind_req(&bind_req, OnBindRequestResult, this);
        FMT_PRINT("({:x})Sent bind request\n", m_ShortAddr);
    }

    void BindInfo::OnBindRequestResult(esp_zb_zdp_status_t zdo_status, void *user_ctx)
    {
        BindInfo *pBind = static_cast<BindInfo *>(user_ctx);
        if (!g_BindInfoPool.IsValid(pBind)) 
        {
            FMT_PRINT("({:x})Got Bind request result, but BindInfo is dead\n", user_ctx);
            return;//we're dead
        }

        if (pBind->m_State != State::SendBindToMeReq)
        {
            FMT_PRINT("({:x})(Bind resp)Unexpected state: {:x}\n", pBind->m_ShortAddr, pBind->m_State);
            return;
        }

        if (zdo_status != ESP_ZB_ZDP_STATUS_SUCCESS)
        {
            if (zdo_status == ESP_ZB_ZDP_STATUS_TIMEOUT)
            {
                if (pBind->m_AttemptsLeft--)
                {
                    FMT_PRINT("({:x})Got Bind request result {:x}. Failed. Starting another attempt (left: {})\n", pBind->m_ShortAddr, zdo_status, (int)pBind->m_AttemptsLeft);
                    pBind->m_Timer.Setup(
                            [](void *user_ctx){
                                BindInfo *pBind = static_cast<BindInfo *>(user_ctx);
                                if (!g_BindInfoPool.IsValid(pBind)) 
                                {
                                    FMT_PRINT("(Re-try attempt)({:x})(Bind request)BindInfo is dead\n", user_ctx);
                                    pBind->SendBindRequest();
                                    return;//we're dead
                                }
                            }, pBind, kTimeout);
                    return;
                }
            }
            FMT_PRINT("({:x})Got Bind request result {:x}. Failed\n", pBind->m_ShortAddr, zdo_status);
            pBind->m_ReportConfigured = false;
            pBind->m_BoundToMe = false;
            pBind->TransitTo(State::NonFunctional);
            return;
        }

        FMT_PRINT("({:x})Got Bind request result: OK\n", pBind->m_ShortAddr);
        //other device did the binding
        //let's configure
        pBind->TransitTo(State::SendConfigureReport);
        pBind->Do();
    }

    void BindInfo::OnGetBindTableFailed(esp_zb_zdp_status_t status)
    {
        if (status == ESP_ZB_ZDP_STATUS_TIMEOUT)
        {
            if (m_AttemptsLeft--)
            {
                FMT_PRINT("({:x})Got Get Bind Table result {:x}. Failed. Starting another attempt (left: {})\n", m_ShortAddr, status, (int)m_AttemptsLeft);
                m_Timer.Setup(
                        [](void *user_ctx){
                            BindInfo *pBind = static_cast<BindInfo *>(user_ctx);
                            if (!g_BindInfoPool.IsValid(pBind)) 
                            {
                                FMT_PRINT("(Re-try attempt)({:x})(Get Bind Table)BindInfo is dead\n", user_ctx);
                                pBind->GetBindTable();
                                return;//we're dead
                            }
                        }, this, kTimeout);
                return;
            }
        }
        m_ReportConfigured = false;
        m_BoundToMe = false;
        TransitTo(State::NonFunctional);
    }

    bool BindInfo::OnBindTableBegin(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx)
    {
        BindInfo *pBind = (BindInfo *)user_ctx;
        if (!g_BindInfoPool.IsValid(pBind)) 
        {
            FMT_PRINT("({:x})Got Bind table chunk, but BindInfo is dead\n", user_ctx);
            return false;//we're dead
        }

        if (pBind->m_State != State::VerifyBinds)
        {
            FMT_PRINT("({:x})(VerifyBinds)Unexpected state: {:x}\n", pBind->m_ShortAddr, pBind->m_State);
            return false;
        }
        return true;
    }

    bool BindInfo::OnGetBindTableChunk(esp_zb_zdo_binding_table_record_t *pNext, void *user_ctx)
    {
        BindInfo *pBind = (BindInfo *)user_ctx;
        if (!g_BindInfoPool.IsValid(pBind)) 
        {
            FMT_PRINT("({:x})Got Bind table chunk, but BindInfo is dead\n", user_ctx);
            return false;//we're dead
        }

        if (zb::ieee_addr{pNext->dst_address.addr_long} == zb::ieee_addr{GetMyIEEE()})
        {
            //ok, it's me
            //right cluster?
            if (IsRelevant(esp_zb_zcl_cluster_id_t(pNext->cluster_id)))//got it. at least one
            {
                //no need to search further
                //need to check if reporting is properly configured
                FMT_PRINT("({:x})Bind table chunk: Found.\n", pBind->m_ShortAddr);
                pBind->TransitTo(State::CheckConfigureReport);
                pBind->Do();
                return false;
            }
        }
        return true;
    }

    void BindInfo::OnBindTableFinished(const esp_zb_zdo_binding_table_info_t *, void *user_ctx)
    {
        BindInfo *pBind = (BindInfo *)user_ctx;
        if (!g_BindInfoPool.IsValid(pBind)) 
        {
            FMT_PRINT("({:x})Got final Bind table chunk, but BindInfo is dead\n", user_ctx);
            return;//we're dead
        }

        FMT_PRINT("({:x})Last bind table chunk: Not Found.\n", pBind->m_ShortAddr);
        //we're done and apparently no bind to ourselfs found
        pBind->TransitTo(State::SendBindToMeReq);
        return pBind->Do();
    }

    void BindInfo::OnBindTableFailure(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx)
    {
        BindInfo *pBind = (BindInfo *)user_ctx;
        if (!g_BindInfoPool.IsValid(pBind)) 
        {
            FMT_PRINT("({:x})Got Bind table chunk, but BindInfo is dead\n", user_ctx);
            return;//we're dead
        }

        if (esp_zb_zdp_status_t(table_info->status) != esp_zb_zdp_status_t::ESP_ZB_ZDP_STATUS_SUCCESS)
        {
            FMT_PRINT("({:x})Bind table chunk failed. Status {:x}\n", pBind->m_ShortAddr, table_info->status);
            return pBind->OnGetBindTableFailed(esp_zb_zdp_status_t(table_info->status));
        }
    }

    void BindInfo::GetBindTable()
    {
        FMT_PRINT("Sending request to {:x} to get binds\n", m_ShortAddr);
        bind_table_iterate(m_ShortAddr, {this, OnGetBindTableChunk, OnBindTableBegin, OnBindTableFinished, OnBindTableFailure});
    }
}
