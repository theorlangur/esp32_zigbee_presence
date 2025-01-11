#include "zbh_apsde_binds.hpp"

namespace zb
{
    static BindUnbind_Handler *g_pFirst = nullptr;

    static std::optional<bool> apsde_bind_callback(esp_zb_apsde_data_ind_t ind)
    {
        if (ind.asdu_length == sizeof(APSME_BindUnbindReq) + 1)
        {
            auto cmd = APSME_Commands(ind.cluster_id);//this will have a command id
            switch(cmd)
            {
                case APSME_Commands::Bind:
                case APSME_Commands::Unbind:
                    if (g_pFirst)
                    {
                        APSME_BindUnbindReq *pReq = (APSME_BindUnbindReq *)(ind.asdu + 1);
                        auto pN = g_pFirst;
                        while(pN)
                        {
                            if (
                                (pN->cluster_id == kAnyCluster || pN->cluster_id == pReq->cluster_id)
                                && (pN->src_ep == kAnyEP || pN->src_ep == pReq->src_ep)
                                && (pN->dst_ep == kAnyEP || pN->dst_ep == pReq->dst_ep)
                               )
                                pN->cb(cmd, *pReq);
                            pN = pN->pNext;
                        }
                        return false;
                    }
                    break;
                default:
                    break;
            }
        }
        return std::nullopt;
    }

    static APSDE_Handler g_APSME_BindHandler{.cb = apsde_bind_callback};


    BindUnbind_Handler::~BindUnbind_Handler()
    {
        Remove();
    }

    void BindUnbind_Handler::Add()
    {
        if (pNext != this)
            return;

        g_APSME_BindHandler.Add();

        pNext = g_pFirst;
        g_pFirst = this;
    }

    void BindUnbind_Handler::Remove()
    {
        if (pNext == this)
            return;

        BindUnbind_Handler *pPrev = nullptr;
        BindUnbind_Handler *pN = g_pFirst;
        while(pN)
        {
            if (pN == this)
            {
                if (pPrev)
                    pPrev->pNext = pNext;
                else
                    g_pFirst = pNext;
                pNext = this;
                break;
            }
            pPrev = pN;
            pN = pN->pNext;
        }
    }
}
