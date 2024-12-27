#ifndef ZBH_CMD_SENDER_HPP_
#define ZBH_CMD_SENDER_HPP_

#include "zbh_alarm.hpp"
#include "zbh_handlers_cmd.hpp"

namespace zb
{
    bool is_coordinator(esp_zb_zcl_addr_t &addr);

    template<uint16_t ClusterId, int CmdId, int Retries>
    struct CmdWithRetries
    {
        static constexpr uint32_t kCmdResponseWait = 700;//ms
        static constexpr uint16_t kInvalidSeqNr = 0xffff;
        using cmd_sender_t = zb::seq_nr_t(*)(void*);
        using cmd_callback_t = void(*)(void*);
        using cmd_fail_callback_t = void(*)(void*, esp_zb_zcl_status_t);

        cmd_sender_t m_CmdSender;
        cmd_callback_t m_OnSuccess;
        cmd_fail_callback_t m_OnFail;
        cmd_fail_callback_t m_OnIntermediateFail;
        void *m_pUserCtx;
        ZbAlarm m_WaitResponseTimer{"m_WaitResponseTimer"};
        uint16_t m_SeqNr = kInvalidSeqNr;
        int m_RetriesLeft = Retries;
        int m_FailureCount = 0;
        int m_CmdId = CmdId;

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
        }

        zb::seq_nr_t SendRaw() 
        { 
            if (m_CmdSender)
                return m_CmdSender(m_pUserCtx);
            return (zb::seq_nr_t)kInvalidSeqNr;
        }

        void Send(int _CmdId = -1)
        {
            if (m_SeqNr != kInvalidSeqNr)//another command is already in flight
            {
                ESP_ERROR_CHECK(ESP_FAIL);
                if (m_OnFail) 
                    m_OnFail(m_pUserCtx, ESP_ZB_ZCL_STATUS_DUPE_EXISTS);
                return;//TODO: log? inc failure count?
            }

            m_RetriesLeft = Retries;
            if (_CmdId != -1)
                m_CmdId = _CmdId;

            if constexpr (Retries) //register response
                SendAgain();
            else
                SendRaw();
            }

            void SendAgain()
            {
                SetSeqNr(SendRaw());
                m_SendCallbackProcessed = m_RespCallbackProcessed = false;
                ZbCmdResponse::Register(ClusterId, m_CmdId, &OnCmdResponse, this);
                m_WaitResponseTimer.Setup(OnTimer, this, kCmdResponseWait);
            }
    private:
        void SetSeqNr(uint16_t nr = kInvalidSeqNr)
        {
            if (m_SeqNr != kInvalidSeqNr)
                ZbCmdSend::Unregister(m_SeqNr);
            m_SeqNr = nr;
            if (m_SeqNr != kInvalidSeqNr)
                ZbCmdSend::Register(m_SeqNr, &OnSendStatus, this);
        }
        bool m_SendCallbackProcessed = false;
        bool m_RespCallbackProcessed = false;
        void OnFailed()
        {
            m_WaitResponseTimer.Cancel();
            m_SendCallbackProcessed = m_RespCallbackProcessed = false;
            SetSeqNr();
            ZbCmdResponse::Unregister(ClusterId, m_CmdId);
            if (m_OnFail) 
                m_OnFail(m_pUserCtx, esp_zb_zcl_status_t::ESP_ZB_ZCL_STATUS_FAIL);
        }

        static bool OnCmdResponse(uint8_t cmd_id, esp_zb_zcl_status_t status_code, esp_zb_zcl_cmd_info_t *pInfo, void *user_ctx)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)user_ctx;
            if (is_coordinator(pInfo->src_address))
            {
                FMT_PRINT("Response from coordinator on Cmd {:x}; status: {:x}\n", pCmd->m_CmdId, (int)status_code);
                return true;//keep response handler
            }

            pCmd->m_RespCallbackProcessed = true;
            pCmd->SetSeqNr();//reset

#ifndef NDEBUG
            {
                using clock_t = std::chrono::system_clock;
                auto now = clock_t::now();
                auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
                FMT_PRINT("{} Response on Cmd {:x} from {}; status: {:x}\n", _n, pCmd->m_CmdId, pInfo->src_address, (int)status_code);
            }
#endif
            if (status_code != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ++pCmd->m_FailureCount;
                if (pCmd->m_OnIntermediateFail)
                    pCmd->m_OnIntermediateFail(pCmd->m_pUserCtx, esp_zb_zcl_status_t(status_code));

                FMT_PRINT("Cmd {:x} failed with status: {:x}\n", pCmd->m_CmdId, (int)status_code);
                if (!pCmd->m_RetriesLeft)
                {
                    //failure
                    FMT_PRINT("Cmd {:x} completely failed\n", pCmd->m_CmdId);
                    pCmd->OnFailed();
                    return false;//no need to keep.
                }
                //try again
                --pCmd->m_RetriesLeft;
                FMT_PRINT("Retry: Cmd {:x} (left: {})\n", pCmd->m_CmdId, pCmd->m_RetriesLeft);
                pCmd->SendAgain();
                return true;//leave registered
            }

            //all good
            pCmd->m_WaitResponseTimer.Cancel();
            if (pCmd->m_OnSuccess)
                (pCmd->m_OnSuccess)(pCmd->m_pUserCtx);
            return false;
        }

        static void OnSendStatus(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)user_ctx;
            auto status_code = pSendStatus->status;
            if (is_coordinator(pSendStatus->dst_addr))
            {
                FMT_PRINT("Response from coordinator on Cmd {:x}; status: {:x}\n", pCmd->m_CmdId, (int)status_code);
                return;//skipping coordinator
            }
            pCmd->m_SendCallbackProcessed = true;
            if (pCmd->m_RespCallbackProcessed)//we got already a resp callback?
            {
                //nevermind then
                FMT_PRINT("Send Cmd {:x} with status {:x}, But already got response earlier\n", pCmd->m_CmdId, status_code);
                return;
            }

#ifndef NDEBUG
            {
                using clock_t = std::chrono::system_clock;
                auto now = clock_t::now();
                auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
                FMT_PRINT("{} Send Cmd {:x} seqNr={:x} to {}; status: {:x}\n", _n, pCmd->m_CmdId, pSendStatus->tsn, pSendStatus->dst_addr, (int)status_code);
            }
#endif
            if (status_code != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ++pCmd->m_FailureCount;
                if (pCmd->m_OnIntermediateFail)
                    pCmd->m_OnIntermediateFail(pCmd->m_pUserCtx, ESP_ZB_ZCL_STATUS_FAIL);

                FMT_PRINT("Cmd {:x} failed with status: {:x}\n", pCmd->m_CmdId, (int)status_code);
                if (!pCmd->m_RetriesLeft)
                {
                    //failure
                    FMT_PRINT("Cmd {:x} completely failed\n", pCmd->m_CmdId);
                    pCmd->OnFailed();
                    return;
                }
                //try again
                --pCmd->m_RetriesLeft;
                FMT_PRINT("Retry: Cmd {:x} (left: {})\n", pCmd->m_CmdId, pCmd->m_RetriesLeft);
                pCmd->SendAgain();
                return;
            }
            //sending was ok, now we expect a response
        }

        static void OnTimer(void *p)
        {
            CmdWithRetries *pCmd = (CmdWithRetries *)p;
            ++pCmd->m_FailureCount;
            pCmd->SetSeqNr();//reset
            ZbCmdResponse::Unregister(ClusterId, pCmd->m_CmdId);
            if (pCmd->m_OnIntermediateFail)
                pCmd->m_OnIntermediateFail(pCmd->m_pUserCtx, ESP_ZB_ZCL_STATUS_TIMEOUT);

            if (pCmd->m_RetriesLeft)
            {
                //report timeout
                    --pCmd->m_RetriesLeft;
                    FMT_PRINT("Retry after timeout: Cmd {:x} (left: {})\n", pCmd->m_CmdId, pCmd->m_RetriesLeft);
                    pCmd->SendAgain();
                    return;
                }

                FMT_PRINT("Cmd {:x} timed out and no retries left\n", pCmd->m_CmdId);
                pCmd->OnFailed();
        }
    };
}
#endif
