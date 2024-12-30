#include "zb_dev_def.hpp"

namespace zb
{
    /**********************************************************************/
    /* LD2412 Component                                                   */
    /**********************************************************************/
    ld2412::Component g_ld2412;//THE presence sensor component

    /**********************************************************************/
    /* Storable data                                                      */
    /**********************************************************************/
    //storable configuration
    LocalConfig g_Config;



    /**********************************************************************/
    /* Internals                                                          */
    /**********************************************************************/
    void Internals::Update()
    {
        static uint32_t g_LastSet = 0xffffffff;
        uint32_t newVal = GetVal();
        if (newVal != g_LastSet)
        {
            g_LastSet = newVal;
            if (auto status = g_Internals.Set(newVal); !status)
            {
                FMT_PRINT("Failed to set internals in update {:x}\n", (int)status.error());
            }
        }
    }
}
