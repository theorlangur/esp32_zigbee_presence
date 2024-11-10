#ifndef DEVICE_CONFIG_HPP_
#define DEVICE_CONFIG_HPP_

#include "esp_err.h"
#include "device_common.hpp"

namespace zb
{
    struct LocalConfig
    {
        static constexpr uint32_t kActualStreamingVersion = 1;

    private:
        uint32_t m_Version = kActualStreamingVersion;
        uint16_t m_OnOffTimeout = 10;//seconds
        OnOffMode m_OnOffMode = OnOffMode::TimedOnLocal;
        PresenceDetectionMode m_PresenceDetectionMode = PresenceDetectionMode::Combined;
        //simple vs energy mode config?
    public:
        auto GetVersion() const { return m_Version; }
        auto GetOnOffTimeout() const { return m_OnOffTimeout; }
        auto GetOnOffMode() const { return m_OnOffMode; }
        auto GetPresenceDetectionMode() const { return m_PresenceDetectionMode; }

        void SetVersion(uint32_t v);
        void SetOnOffTimeout(uint16_t v);
        void SetOnOffMode(OnOffMode v);
        void SetPresenceDetectionMode(PresenceDetectionMode v);

        void FactoryReset();

        esp_err_t on_start();
        esp_err_t on_change();
        void on_end();
    };
}
#endif
