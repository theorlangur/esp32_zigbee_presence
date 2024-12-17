#ifndef DEVICE_CONFIG_HPP_
#define DEVICE_CONFIG_HPP_

#include "esp_err.h"
#include "device_common.hpp"

namespace zb
{
    struct LocalConfig
    {
        static constexpr uint32_t kActualStreamingVersion = 1;
        static constexpr uint8_t kMaxIlluminance = 255;

        struct PresenceDetectionMode
        {
            uint8_t m_Edge_mmWave      : 1 = 1;
            uint8_t m_Edge_PIRInternal : 1 = 1;
            uint8_t m_Edge_External    : 1 = 1;
            uint8_t m_Keep_mmWave      : 1 = 1;
            uint8_t m_Keep_PIRInternal : 1 = 1;
            uint8_t m_Keep_External    : 1 = 1;
        };
    private:
        uint32_t m_Version = kActualStreamingVersion;
        uint16_t m_OnOffTimeout = 10;//seconds
        OnOffMode m_OnOffMode = OnOffMode::TimedOnLocal;
        PresenceDetectionMode m_PresenceDetectionMode;
        LD2412::SystemMode m_LD2412Mode = LD2412::SystemMode::Energy;
        uint8_t m_IlluminanceThreshold = kMaxIlluminance; //Illuminance<=Threashold -> active, sending on/off commands
        uint16_t m_ExternalOnOffTimeout = 1;
        uint16_t m_Restarts = 0;
    public:
        auto GetVersion() const { return m_Version; }
        auto GetOnOffTimeout() const { return m_OnOffTimeout; }
        auto GetOnOffMode() const { return m_OnOffMode; }
        auto GetPresenceDetectionMode() const { return m_PresenceDetectionMode; }
        auto GetLD2412Mode() const { return m_LD2412Mode; }
        auto GetIlluminanceThreshold() const { return m_IlluminanceThreshold; }
        auto GetExternalOnOffTimeout() const { return m_ExternalOnOffTimeout; }
        auto GetRestarts() const { return m_Restarts; }

        void SetVersion(uint32_t v);
        void SetOnOffTimeout(uint16_t v);
        void SetOnOffMode(OnOffMode v);
        void SetPresenceDetectionMode(PresenceDetectionMode v);
        void SetLD2412Mode(LD2412::SystemMode v);
        void SetIlluminanceThreshold(uint8_t v);
        void SetExternalOnOffTimeout(uint16_t v);

        void FactoryReset();

        esp_err_t on_start();
        esp_err_t on_change();
        void on_end();
    };
}
#endif
