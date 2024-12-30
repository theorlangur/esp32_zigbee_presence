#ifndef ZB_DEV_DEF_CONST_HPP_
#define ZB_DEV_DEF_CONST_HPP_

#include "../zb_helpers/zbh_helpers.hpp"

//#define ENABLE_ENGINEERING_ATTRIBUTES

namespace zb
{
    constexpr uint8_t PRESENCE_EP = 1;
    static constexpr const uint16_t CLUSTER_ID_LD2412 = kManufactureSpecificCluster;
    constexpr uint32_t kDelayedAttrChangeTimeout = 200;
}
#endif
