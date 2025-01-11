#ifndef ZBH_CMD_SENDER_HPP_
#define ZBH_CMD_SENDER_HPP_

#include "zbh_alarm.hpp"
#include "zbh_handlers_cmd.hpp"

namespace zb
{

    template<uint16_t ClusterId, int CmdId, int Retries>
    struct CmdWithRetries: NonMovable, NonCopyable
    {
        static constexpr uint32_t kCmdResponseWait = 700;//ms
        static constexpr uint16_t kInvalidSeqNr = kInvalidTSN;
        using cmd_sender_t = zb::seq_nr_t(*)(void*);
        using cmd_callback_t = void(*)(void*);
        using cmd_fail_callback_t = void(*)(void*, esp_zb_zcl_status_t, esp_err_t e_err);

        cmd_sender_t m_CmdSender;
        cmd_callback_t m_OnSuccess;
        cmd_fail_callback_t m_OnFail;
        cmd_fail_callback_t m_OnIntermediateFail;
        void *m_pUserCtx;
        ZbAlarm m_WaitResponseTimer{"m_WaitResponseTimer"};
        int m_RetriesLeft = Retries;
        int m_FailureCount = 0;
        ZbCmdResponse::Node m_ResponseNode;
        ZbCmdSend::Node m_SendStatusNode;

        CmdWithRetries(cmd_sender_t sender
                , cmd_callback_t on_success = nullptr
                , cmd_fail_callback_t on_fail = nullptr
                , cmd_fail_callback_t on_intermediate_fail = nullptr
                , void *pUserCtx = nullptr):
            m_CmdSender(sender)
            , m_OnSuccess(on_success)
            , m_OnFail(on_fail)
            , m_OnIntermediateFail(on_intermediate_fail)
            , m_pUserCtx(pUserCtx)
        {
            m_ResponseNode.cluster_id = ClusterId;
            m_ResponseNode.user_ctx = this;
            m_ResponseNode.cb = OnCmdResponse;
            m_ResponseNode.tsn = kInvalidSeqNr;

            m_SendStatusNode.user_ctx = this;
            m_SendStatusNode.cb = OnSendStatus;
            m_SendStatusNode.tsn = kInvalidSeqNr;
        }

        bool IsActive() const { return m_ResponseNode.m_pList || m_SendStatusNode.m_pList; }

        zb::seq_nr_t SendRaw() 
        { 
            if (m_CmdSender)
                return m_CmdSender(m_pUserCtx);
            return (zb::seq_nr_t)kInvalidSeqNr;
        }

        void Send(int _CmdId = -1)
        {
            if (IsActive())//another command is already in flight
            {
                ESP_ERROR_CHECK(ESP_FAIL);
                if (m_OnFail) 
                    m_OnFail(m_pUserCtx, ESP_ZB_ZCL_STATUS_DUPE_EXISTS, ESP_OK);
                return;//TODO: log? inc failure count?
            }

            m_RetriesLeft = Retries;
            if (_CmdId != -1)
                m_ResponseNode.cmd_id = _CmdId;
            else
                m_ResponseNode.cmd_id = CmdId;


            if constexpr (Retries) //register response
                SendAgain();
            else
                SendRaw();
            }

            void SendAgain()
            {
                SetSeqNr(SendRaw());
                m_ResponseNode.tsn = m_SendStatusNode.tsn;
                m_ResponseNode.RegisterSelf();

                m_WaitResponseTimer.Setup(OnTimer, this, kCmdResponseWait);
            }

            zb::seq_nr_t GetTSN() const { return m_SendStatusNode.tsn; }
            bool IsWaitingForSendStatus() const { return m_SendStatusNode.m_pList != nullptr; }
            bool IsWaitingForCmdResponse() const { return m_ResponseNode.m_pList != nullptr; }
    private:
        void SetSeqNr(uint16_t nr = kInvalidSeqNr)
        {
            m_SendStatusNode.tsn = nr;
            if (nr != kInvalidSeqNr)
                m_SendStatusNode.RegisterSelf();
            else
                m_SendStatusNode.RemoveFromList();
        }

        void OnFailed()
        {
            m_WaitResponseTimer.Cancel();
            m_ResponseNode.RemoveFromList();
            m_SendStatusNode.RemoveFromList();
            SetSeqNr();
            if (m_OnFail) 
                m_OnFail(m_pUserCtx, esp_zb_zcl_status_t::ESP_ZB_ZCL_STATUS_FAIL, ESP_OK);
        }

        static void OnCmdResponse(uint8_t cmd_id, esp_zb_zcl_status_t status_code, esp_zb_zcl_cmd_info_t *pInfo, void *user_ctx)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)user_ctx;
            if (IsCoordinator(pInfo->src_address))
            {
                FMT_PRINT("Response from coordinator on Cmd {:x}; status: {:x}\n", pCmd->m_ResponseNode.cmd_id, (int)status_code);
                return;
            }

            pCmd->m_ResponseNode.RemoveFromList();
            pCmd->m_SendStatusNode.RemoveFromList();
            pCmd->SetSeqNr();//reset

#ifndef NDEBUG
            {
                using clock_t = std::chrono::system_clock;
                auto now = clock_t::now();
                auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
                FMT_PRINT("{} Response on Cmd {:x} from {}; status: {:x}\n", _n, pCmd->m_ResponseNode.cmd_id, pInfo->src_address, (int)status_code);
            }
#endif
            if (status_code != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ++pCmd->m_FailureCount;
                if (pCmd->m_OnIntermediateFail)
                    pCmd->m_OnIntermediateFail(pCmd->m_pUserCtx, status_code, ESP_OK);

                FMT_PRINT("Cmd {:x} failed with status: {:x}\n", pCmd->m_ResponseNode.cmd_id, (int)status_code);
                if (!pCmd->m_RetriesLeft)
                {
                    //failure
                    FMT_PRINT("Cmd {:x} completely failed\n", pCmd->m_ResponseNode.cmd_id);
                    pCmd->OnFailed();
                    return;
                }
                //try again
                --pCmd->m_RetriesLeft;
                FMT_PRINT("Retry: Cmd {:x} (left: {})\n", pCmd->m_ResponseNode.cmd_id, pCmd->m_RetriesLeft);
                pCmd->SendAgain();
                return;
            }

            //all good
            pCmd->m_WaitResponseTimer.Cancel();
            if (pCmd->m_OnSuccess)
                (pCmd->m_OnSuccess)(pCmd->m_pUserCtx);
        }

        static void OnSendStatus(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)user_ctx;
            auto status_code = pSendStatus->status;

#ifndef NDEBUG
            {
                using clock_t = std::chrono::system_clock;
                auto now = clock_t::now();
                auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
                FMT_PRINT("{} Send Cmd {:x} seqNr={:x} to {}; status: {:x}\n", _n, pCmd->m_ResponseNode.cmd_id, pSendStatus->tsn, pSendStatus->dst_addr, (int)status_code);
            }
#endif
            if (status_code != ESP_OK)
            {
                pCmd->m_ResponseNode.RemoveFromList();
                ++pCmd->m_FailureCount;
                if (pCmd->m_OnIntermediateFail)
                    pCmd->m_OnIntermediateFail(pCmd->m_pUserCtx, ESP_ZB_ZCL_STATUS_FAIL, status_code);

                FMT_PRINT("Cmd {:x} failed with status: {:x}\n", pCmd->m_ResponseNode.cmd_id, (int)status_code);
                if (!pCmd->m_RetriesLeft)
                {
                    //failure
                    FMT_PRINT("Cmd {:x} completely failed\n", pCmd->m_ResponseNode.cmd_id);
                    pCmd->OnFailed();
                    return;
                }
                //try again
                --pCmd->m_RetriesLeft;
                FMT_PRINT("Retry: Cmd {:x} (left: {})\n", pCmd->m_ResponseNode.cmd_id, pCmd->m_RetriesLeft);
                pCmd->SendAgain();
                return;
            }
            //re-add self, if there are more binds to wait for
            pCmd->m_SendStatusNode.RegisterSelf();
            //sending was ok, now we expect a response
        }

        static void OnTimer(void *p)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)p;
            ++pCmd->m_FailureCount;
            if (pCmd->m_OnIntermediateFail)
                pCmd->m_OnIntermediateFail(pCmd->m_pUserCtx, ESP_ZB_ZCL_STATUS_TIMEOUT, ESP_ERR_TIMEOUT);
            pCmd->SetSeqNr();//reset
            pCmd->m_ResponseNode.RemoveFromList();

            if (pCmd->m_RetriesLeft)
            {
                //report timeout
                --pCmd->m_RetriesLeft;
                FMT_PRINT("Retry after timeout: Cmd {:x} (left: {})\n", pCmd->m_ResponseNode.cmd_id, pCmd->m_RetriesLeft);
                pCmd->SendAgain();
                return;
            }

            FMT_PRINT("Cmd {:x} timed out and no retries left\n", pCmd->m_ResponseNode.cmd_id);
            pCmd->OnFailed();
        }
    };
}
#endif
