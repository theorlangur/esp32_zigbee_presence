#include "zbh_handlers_cmd.hpp"

namespace zb
{
    LinkedListT<ZbCmdResponse::Node> ZbCmdResponse::g_CmdResponseList;

    void ZbCmdResponse::Node::RegisterSelf()
    {
        g_CmdResponseList += *this;
    }

    LinkedListT<ZbCmdSend::Node> ZbCmdSend::g_SendStatusList;
    void ZbCmdSend::Node::RegisterSelf()
    {
        g_SendStatusList += *this;
    }

    void ZbCmdSend::handler(esp_zb_zcl_command_send_status_message_t message)
    {
        for(Node *pN : g_SendStatusList)
        {
            if (pN->tsn == message.tsn)
            {
                pN->RemoveFromList();
                if (pN->cb)
                    pN->cb(&message, pN->user_ctx);
                return;
            }
        }

        FMT_PRINT("[task={}; lock={}]Send status(no callback): seqNr={}; dst={},  status={:x}\n", (const char*)pcTaskGetName(nullptr), APILock::g_State, message.tsn, message.dst_addr, message.status);
    }
}
