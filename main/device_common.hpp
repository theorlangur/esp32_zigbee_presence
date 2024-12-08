#ifndef DEVICE_COMMON_HPP_
#define DEVICE_COMMON_HPP_
#include <cstdint>
#include "ld2412_component.hpp"

namespace zb
{
    enum class OnOffMode: uint8_t
    {
        OnOff = 0,
        OnOnly = 1,
        OffOnly = 2,
        TimedOn = 3,
        TimedOnLocal = 4,
        Nothing = 5,
    };
}
#endif
