#ifndef DEVICE_COMMON_HPP_
#define DEVICE_COMMON_HPP_
#include <cstdint>
#include "periph/ld2412_component.hpp"

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

    enum class TriState: uint16_t
    {
        False     = 0,
        True      = 1,
        Undefined = 2
    };

    struct TriState8Array
    {
        TriState m0 : 2 = TriState::Undefined;
        TriState m1 : 2 = TriState::Undefined;
        TriState m2 : 2 = TriState::Undefined;
        TriState m3 : 2 = TriState::Undefined;
        TriState m4 : 2 = TriState::Undefined;
        TriState m5 : 2 = TriState::Undefined;
        TriState m6 : 2 = TriState::Undefined;
        TriState m7 : 2 = TriState::Undefined;

        TriState Get(size_t i) const
        { 
            if (i >= 8) return TriState::Undefined;
            const uint16_t *pV = (const uint16_t *)this;
            return TriState((*pV >> (i * 2)) & 0x03);
        }

        void Set(size_t i, TriState s) const
        { 
            if (i >= 8) return;
            uint16_t *pV = (uint16_t *)this;
            *pV = (*pV & ~(0x03 << (i * 2))) | (uint16_t(s) << (i * 2));
        }

        uint16_t GetRaw() const { return *(const uint16_t *)this; }
    };
}
#endif
