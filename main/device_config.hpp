#ifndef DEVICE_CONFIG_HPP_
#define DEVICE_CONFIG_HPP_

#include "esp_err.h"
#include "device_common.hpp"

namespace zb
{
    struct LocalConfig
    {
        static constexpr uint32_t kActualStreamingVersion = 1;

        uint32_t m_Version = kActualStreamingVersion;
        uint16_t m_OnOffTimeout = 10;//seconds
        OnOffMode m_OnOffMode = OnOffMode::TimedOnLocal;
        PresenceDetectionMode m_PresenceDetectionMode = PresenceDetectionMode::Combined;
        //simple vs energy mode config?

        esp_err_t on_start();
        esp_err_t on_change();
        void on_end();
    };
}
#endif
