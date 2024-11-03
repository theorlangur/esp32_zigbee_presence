#ifndef DEVICE_COMMON_HPP_
#define DEVICE_COMMON_HPP_
#include <cstdint>

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

    enum class PresenceDetectionMode: uint8_t
    {
        Combined = 0,//PIR or mmWave to detect, PIR and mmWave both cleared to clear
        mmWaveOnly = 1,
        PIROnly = 2,
        PIRDriven = 3,//occupancy detected only when PIR reports the detection, cleared when both are clear
    };
}
#endif
