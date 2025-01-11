#ifndef ZBH_APSDE_HPP_
#define ZBH_APSDE_HPP_
#include "lib_misc_helpers.hpp"
#include "aps/esp_zigbee_aps.h"
#include <optional>

namespace zb
{
    using absde_indication_callback_t = std::optional<bool>(*)(esp_zb_apsde_data_ind_t ind);
    struct APSDE_Handler: NonCopyable, NonMovable
    {
        absde_indication_callback_t cb;
        APSDE_Handler *pNext = this;//indication that we're not in the list

        ~APSDE_Handler();
        void Add();
        void Remove();
    };

    void setup_generic_absde_data_indication_handling();
}
#endif
