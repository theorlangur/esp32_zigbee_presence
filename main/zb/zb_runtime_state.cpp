#include "zb_dev_def.hpp"
#include "../colors_def.hpp"

namespace zb
{
    void cmd_failure(void *pCtx, esp_zb_zcl_status_t status_code, esp_err_t e)
    {
        ++g_State.m_Internals.m_IntermediateCmdFailuireCount;
        g_State.m_LastFailedStatus = status_code;
        g_State.m_FailedStatusUpdated = true;
        g_State.m_Internals.m_LastESP_ERR = e;

        if (status_code == ESP_ZB_ZCL_STATUS_TIMEOUT)
        {
            if (pCtx == &g_State.m_OnSender)
            {
                g_State.m_Internals.m_LastTimeoutTSN = g_State.m_OnSender.GetTSN();
                g_State.m_Internals.m_WaitForSendStatus = g_State.m_OnSender.IsWaitingForSendStatus();
                g_State.m_Internals.m_WaitForResponse = g_State.m_OnSender.IsWaitingForCmdResponse();
            }
            if (pCtx == &g_State.m_OffSender)
            {
                g_State.m_Internals.m_LastTimeoutTSN = g_State.m_OffSender.GetTSN();
                g_State.m_Internals.m_WaitForSendStatus = g_State.m_OffSender.IsWaitingForSendStatus();
                g_State.m_Internals.m_WaitForResponse = g_State.m_OffSender.IsWaitingForCmdResponse();
            }
            if (pCtx == &g_State.m_OnTimedSender)
            {
                g_State.m_Internals.m_LastTimeoutTSN = g_State.m_OnTimedSender.GetTSN();
                g_State.m_Internals.m_WaitForSendStatus = g_State.m_OnTimedSender.IsWaitingForSendStatus();
                g_State.m_Internals.m_WaitForResponse = g_State.m_OnTimedSender.IsWaitingForCmdResponse();
            }
        }
    }

    void cmd_total_failure(void *, esp_zb_zcl_status_t status_code, esp_err_t e)
    {
        led::blink_pattern(colors::kBlinkPatternCmdError, colors::kCmdError, duration_ms_t(1000));
        led::blink(false, {});
        ++g_State.m_Internals.m_TotalFailureCount;
        g_State.m_LastFailedStatus = status_code;
        g_State.m_FailedStatusUpdated = true;
        g_State.m_Internals.m_LastESP_ERR = e;
    }

    zb::seq_nr_t send_on_raw(void*)
    {
        esp_zb_zcl_on_off_cmd_t cmd_req{};
        cmd_req.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_ON_ID;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        return esp_zb_zcl_on_off_cmd_req(&cmd_req);
    }

    zb::seq_nr_t send_off_raw(void*)
    {
        esp_zb_zcl_on_off_cmd_t cmd_req{};
        cmd_req.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        return esp_zb_zcl_on_off_cmd_req(&cmd_req);
    }

    zb::seq_nr_t send_on_timed_raw(void*)
    {
        auto t = g_Config.GetOnOffTimeout();
        esp_zb_zcl_on_off_on_with_timed_off_cmd_t cmd_req{};
        cmd_req.zcl_basic_cmd.src_endpoint = PRESENCE_EP;
        cmd_req.on_off_control = 0;//process unconditionally
        cmd_req.on_time = t * 10;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        return esp_zb_zcl_on_off_on_with_timed_off_cmd_req(&cmd_req);
    }


    /**********************************************************************/
    /* RuntimeState                                                       */
    /**********************************************************************/
    RuntimeState g_State;
    bool RuntimeState::CanSendCommandsToBind() const
    {
        return m_Internals.m_BoundDevices || m_InitialBindsChecking;
    }

    bool RuntimeState::CommandsToBindInFlight() const
    {
        return m_OnSender.IsActive() || m_OffSender.IsActive() || m_OnTimedSender.IsActive();
    }

    uint8_t RuntimeState::GetIlluminance() const
    {
        if (g_Config.GetIlluminanceExternal())
            return m_ExternalIlluminance;
        return g_ld2412.GetMeasuredLight();
    }

    void RuntimeState::StartExternalTimer(esp_zb_user_callback_t cb, uint32_t time)
    {
        FMT_PRINT("Starting external timer for {} ms\n", time);
        m_ExternalRunningTimer.Setup(cb, nullptr, time);
    }

    void RuntimeState::StartLocalTimer(esp_zb_user_callback_t cb, uint32_t time)
    {
        FMT_PRINT("Starting local timer for {} ms\n", time);
        m_RunningTimer.Setup(cb, nullptr, time);
    }

    void RuntimeState::ScheduleBindsChecking()
    {
        m_BindsCheck.Setup([](void *p){
                RuntimeState *pState = (RuntimeState *)p;
                pState->RunBindsChecking();
                }, this, 2000);
    }

    void RuntimeState::RunBindsChecking()
    {
        BindIteratorConfig cfg;
        cfg.on_begin = [](const esp_zb_zdo_binding_table_info_t *table_info, void *pCtx)->bool{
            FMT_PRINT("New own binds check round\n");
            g_State.m_TempNewBinds.clear();
            g_State.m_FoundExisting = 0;
            g_State.m_BindsReportingCapable = g_Config.GetBindReporting();
            FMT_PRINT("Bind report caps: {:x}\n", g_State.m_BindsReportingCapable.GetRaw());
            for(auto &bi : g_State.m_TrackedBinds)
                bi->m_BindChecked = false;
            return true;
        };
        cfg.on_entry = [](esp_zb_zdo_binding_table_record_t *pRec, void *pCtx)->bool{
            auto &newBinds = g_State.m_TempNewBinds;
            uint16_t shortAddr = esp_zb_address_short_by_ieee(pRec->dst_address.addr_long);
            esp_zb_zcl_addr_t addr;
            addr.addr_type = pRec->dst_addr_mode;
            std::memcpy(&addr.u, &pRec->dst_address, sizeof(addr.u));
            FMT_PRINT("(Raw)Bind. Addr={}, short:{:x} to cluster {:x}, ep={}\n", addr, shortAddr, pRec->cluster_id, pRec->dst_endp);
            if (shortAddr != 0xffff && RuntimeState::IsRelevant(esp_zb_zcl_cluster_id_t(pRec->cluster_id)))//got it. at least one
            {
                auto existingI = g_State.m_TrackedBinds.find(zb::ieee_addr{pRec->dst_address.addr_long}, &BindInfo::m_IEEE);
                if (existingI == g_State.m_TrackedBinds.end())
                {
                    FMT_PRINT("Bind. Addr={}, short:{:x} to cluster {:x}, ep={}\n", addr, shortAddr, pRec->cluster_id, pRec->dst_endp);
                    FMT_PRINT("This is a new bind\n");
                    auto r = newBinds.emplace_back(pRec->dst_address.addr_long, shortAddr);
                    if (r)
                    {
                        BindInfoPtr &ptr = *r;
                        if (ptr)
                        {
                            BindInfo &bi = *ptr;
                            bi.m_BindChecked = true;
                            bi.m_EP = pRec->dst_endp;
                            bi.m_AttemptsLeft = BindInfo::kMaxConfigAttempts;
                            if (g_State.m_InitialBindsChecking)
                            {
                                auto repCapable = g_State.m_BindsReportingCapable.Get(newBinds.size() - 1);
                                if (repCapable == TriState::Undefined)
                                {
                                    bi.m_CheckReporting = true;
                                    FMT_PRINT("{:x}: Requesting reporting check as this one is saved as undefined\n", shortAddr);
                                }
                                else if (repCapable == TriState::False)
                                {
                                    FMT_PRINT("{:x}: Skipping check as this one is saved as non-functional\n", shortAddr);
                                    //no need to do any checks
                                    bi.Failed();//put into failed state, skip all the checks
                                }
                            }
                            else
                            {
                                FMT_PRINT("Requesting reporting check for {:x}\n", shortAddr);
                                bi.m_CheckReporting = true;
                            }
                            bi.Do();//start the whole thing
                        }
                    }
                }else
                {
                    FMT_PRINT("This is a existing bind\n");
                    ++g_State.m_FoundExisting;
                    (*existingI)->m_BindChecked = true;
                    (*existingI)->m_EP = pRec->dst_endp;
                    (*existingI)->RunCheckIfRequested();
                }
                //++foundBinds;
            }else
            {
                FMT_PRINT("Bind. Addr={}, invalid short:{:x} to cluster {:x}, ep={}\n", addr, shortAddr, pRec->cluster_id, pRec->dst_endp);
                if (shortAddr == 0xffff)
                {
                    //request another one
                    g_State.m_NeedBindsChecking = true;
                }
            }
            return true;
        };
        cfg.on_error = [](const esp_zb_zdo_binding_table_info_t *pTable, void *pCtx){
            //arm the next timer
            FMT_PRINT("Binds check error: {:x}; Next round\n", pTable->status);
            g_State.m_NeedBindsChecking = true;
        };
        cfg.on_end = [](const esp_zb_zdo_binding_table_info_t *pTable, void *pCtx){
            uint8_t newStates = 0;
            uint8_t newValidity = 0;
            if (g_State.m_FoundExisting != g_State.m_TrackedBinds.size())
            {
                TriState8Array newReportingStates;
                int nextOldIdx = 0, nextNewIdx = 0;
                for(auto i = g_State.m_TrackedBinds.begin(), e = g_State.m_TrackedBinds.end(); i != e; ++i, ++nextOldIdx)
                {
                    if (!(*i)->m_BindChecked)
                    {
                        (*i)->Unbind();
                        g_State.m_BindsToCleanup.push_back(std::move(*i));
                        g_State.m_TrackedBinds.erase(i--);
                        e = g_State.m_TrackedBinds.end();
                    }
                    else
                    {
                        newReportingStates.Set(nextNewIdx, g_State.m_BindsReportingCapable.Get(nextOldIdx));
                        newStates |= (g_State.m_BindStates & (1 << nextOldIdx)) >> (nextNewIdx - nextOldIdx);
                        newValidity |= (g_State.m_ValidBinds & (1 << nextOldIdx)) >> (nextNewIdx - nextOldIdx);
                        ++nextNewIdx;
                    }
                }
                if (g_State.m_BindsReportingCapable.GetRaw() != newReportingStates.GetRaw())
                {
                    FMT_PRINT("After iterating binds: updating info about report capabilities: from {:x} to {:x}\n", g_State.m_BindsReportingCapable.GetRaw(), newReportingStates.GetRaw());
                    g_State.m_BindsReportingCapable = newReportingStates;
                    g_Config.SetBindReporting(g_State.m_BindsReportingCapable);
                }
            }else
            {
                newStates = g_State.m_BindStates;
                newValidity = g_State.m_ValidBinds;
            }

            for(auto &bi : g_State.m_TempNewBinds)
                g_State.m_TrackedBinds.push_back(std::move(bi));

            g_State.m_TempNewBinds.clear();
            g_State.m_BindStates = newStates;
            g_State.m_ValidBinds = newValidity;
            g_State.m_Internals.m_BoundDevices = g_State.m_TrackedBinds.size();
            g_State.m_InitialBindsChecking = false;
        };
        FMT_PRINT("Initiating own binds iteration\n");
        bind_table_iterate(esp_zb_get_short_address(), cfg);
    }

    void RuntimeState::RunService()
    {
        ZbAlarm::check_death_count();

        for(auto i = m_BindsToCleanup.begin(); i != m_BindsToCleanup.end(); ++i)
        {
            if ((*i)->GetState() == BindInfo::State::NonFunctional)
            {
                FMT_PRINT("Bind cleanup: {:x}\n", (*i)->m_ShortAddr);
                m_BindsToCleanup.erase(i--);
            }
        }

        auto prevValidBinds = m_ValidBinds;
        auto prevBindStates = m_BindStates;
        [[maybe_unused]]auto prevReportCaps = m_BindsReportingCapable;
        bool updateReportingCapsInConfig = false;
        //update validity of the binds
        for(size_t i = 0, n = m_TrackedBinds.size(); i < n; ++i)
        {
            auto &bi = m_TrackedBinds[i];
            auto s = bi->GetState();
            if (s == BindInfo::State::Functional)
            {
                m_ValidBinds |= 1 << i;
                if (bi->m_Initial)
                {
                    bi->m_Initial = false;
                    m_BindStates = (m_BindStates & ~(1 << i)) | (bi->m_InitialValue << i);
                }

                if (bi->m_CheckReporting)
                {
                    FMT_PRINT("Bind {:x} Is has functional reporting\n", bi->m_ShortAddr);
                    updateReportingCapsInConfig = true;   
                    m_BindsReportingCapable.Set(i, TriState::True);
                }
            }
            else
            {
                m_ValidBinds &= ~(1 << i);
                if (s == BindInfo::State::NonFunctional && bi->m_CheckReporting && bi->m_BoundToMe)
                {
                    FMT_PRINT("Bind {:x} Is has non-functional reporting\n", bi->m_ShortAddr);
                    updateReportingCapsInConfig = true;   
                    m_BindsReportingCapable.Set(i, TriState::False);
                }
            }

            if (s == BindInfo::State::Functional || s == BindInfo::State::NonFunctional)
                bi->m_CheckReporting = false;
        }

        m_Internals.m_ConfiguredReports = m_ValidBinds;

        if (!CommandsToBindInFlight())
        {
            m_Internals.Update();
            if (g_State.m_FailedStatusUpdated)
            {
                g_State.m_FailedStatusUpdated = false;
                if (auto status = g_FailureStatus.Set((uint16_t)g_State.m_LastFailedStatus); !status)
                {
                    FMT_PRINT("Failed to set failure status {:x}\n", (int)status.error());
                }
            }

            if (m_NeedBindsChecking)
            {
                m_NeedBindsChecking = false;
                ScheduleBindsChecking();
            }

            {
                static uint16_t g_LastIlluminance = 0xffff;
                auto currentIll = g_State.GetIlluminance();
                if (currentIll != g_LastIlluminance)
                {
                    g_LastIlluminance = currentIll;
                    if (auto status = g_LD2412EngineeringLight.Set(currentIll); !status)
                    {
                        FMT_PRINT("Failed to set measured light attribute with error {:x}\n", (int)status.error());
                    }
                }
            }
        }

        if (prevValidBinds != m_ValidBinds)
        {
            FMT_PRINT("Valid binds changed: from {:x} to {:x}\n", prevValidBinds, m_ValidBinds);
        }
        if (prevBindStates != m_BindStates)
        {
            FMT_PRINT("Bind states changed: from {:x} to {:x}\n", prevBindStates, m_BindStates);
        }

        if (updateReportingCapsInConfig)
        {
            FMT_PRINT("Updating info about report capabilities: from {:x} to {:x}\n", prevReportCaps.GetRaw(), m_BindsReportingCapable.GetRaw());
            g_Config.SetBindReporting(m_BindsReportingCapable);
        }

        static ZbAlarm rep{"InternalsReporting"};
        rep.Setup([](void *p){
                RuntimeState *pState = (RuntimeState *)p;
                pState->RunService();
                }, this, 1000);
    }
}
