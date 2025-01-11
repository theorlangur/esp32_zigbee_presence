#include "zbh_apsde.hpp"
#include "nwk/esp_zigbee_nwk.h"

namespace zb
{
    constinit static APSDE_Handler *g_pFirst = nullptr;

    bool generic_apsde_data_indication_callback(esp_zb_apsde_data_ind_t ind)
    {
        if (ind.dst_short_addr == esp_zb_get_short_address() && ind.status == 0)
        {
            auto pN = g_pFirst;
            while(pN)
            {
                if (auto r = pN->cb(ind))//if non-empty optional is returned - we're done processing
                    return *r;
                pN = pN->pNext;
            }
        }
        return false;
    }

    APSDE_Handler::~APSDE_Handler()
    {
        Remove();
    }

    void APSDE_Handler::Add()
    {
        if (pNext != this)
        {
            //already in the list
            return;
        }

        pNext = g_pFirst;
        g_pFirst = this;
    }

    void APSDE_Handler::Remove()
    {
        if (pNext == this)
            return;//not in the list
        APSDE_Handler *pPrev = nullptr;
        APSDE_Handler *pN = g_pFirst;
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

    void setup_generic_absde_data_indication_handling()
    {
        esp_zb_aps_data_indication_handler_register(generic_apsde_data_indication_callback);
    }
}
