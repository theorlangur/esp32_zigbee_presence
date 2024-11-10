#include "device_config.hpp"
#include "esp_log.h"
#include <sys/stat.h>
#include "esp_littlefs.h"
#include "generic_helpers.hpp"

static const char *TAG = "esp_littlefs";
namespace zb
{
    static const char *kBasePath = "/littlefs";
    static const char *kConfigFilePath = "/littlefs/config.dat";
    static const char *kParitionLabel = "zb_config";
    esp_err_t LocalConfig::on_start()
    {
        ESP_LOGI(TAG, "Initializing LittleFS");

        esp_vfs_littlefs_conf_t conf = {
            .base_path = kBasePath,
            .partition_label = kParitionLabel,
            .format_if_mount_failed = true,
            .dont_mount = false,
        };

        esp_err_t ret = esp_vfs_littlefs_register(&conf);

        if (ret != ESP_OK) {
            if (ret == ESP_FAIL) {
                ESP_LOGE(TAG, "Failed to mount or format filesystem");
            } else if (ret == ESP_ERR_NOT_FOUND) {
                ESP_LOGE(TAG, "Failed to find LittleFS partition");
            } else {
                ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
            }
            return ret;
        }

        struct stat st;
        if (stat(kConfigFilePath, &st) == 0) {
            //exists
            FILE *f = fopen(kConfigFilePath, "rb");
            if (!f)
            {
                ESP_LOGE(TAG, "Failed to open existin file %s for binary reading", kConfigFilePath);
                return ESP_FAIL;
            }
            ScopeExit cleanup = [&]{ if (f) fclose(f);};
            if (size_t r = fread(this, 1, sizeof(m_Version), f); r != sizeof(m_Version))
            {
                ESP_LOGE(TAG, "Failed to read version from existing config file %s", kConfigFilePath);
                ESP_LOGE(TAG, "Read: %d; File size: %d", r, (size_t)st.st_size);
                fclose(f);
                f = nullptr;
                return on_change();//we must always write the up-to-date version after conversion
            }

            if (m_Version == kActualStreamingVersion)
            {
                constexpr const size_t kRestToRead = sizeof(LocalConfig) - sizeof(m_Version);
                if (size_t r = fread(&m_Version + 1, 1, kRestToRead, f); r != kRestToRead)
                {
                    ESP_LOGE(TAG, "Failed to read rest from existing config file %s (read: %d)", kConfigFilePath, r);
                    return ESP_ERR_INVALID_SIZE;
                }
                return ESP_OK;
            }else
            {
                //here be dragons. I mean streaming conversion
                fclose(f);
                f = nullptr;
                return on_change();//we must always write the up-to-date version after conversion
            }
        }else //doesn't exist. create new
            return on_change();
    }

    esp_err_t LocalConfig::on_change()
    {
        FILE *f = fopen(kConfigFilePath, "wb");
        if (!f)
        {
            ESP_LOGE(TAG, "Failed to open for writing file %s", kConfigFilePath);
            return ESP_ERR_NOT_FOUND;
        }
        ScopeExit cleanup = [&]{fclose(f);};
        if (size_t r = fwrite(this, 1, sizeof(LocalConfig), f); r != sizeof(LocalConfig))
        {
            ESP_LOGE(TAG, "Failed to write config file %s (written: %d)", kConfigFilePath, r);
            return ESP_ERR_INVALID_SIZE;
        }
        return ESP_OK;
    }

    void LocalConfig::on_end()
    {
        esp_vfs_littlefs_unregister(kParitionLabel);
        ESP_LOGI(TAG, "LittleFS unmounted");
    }

    void LocalConfig::SetVersion(uint32_t v)
    {
        m_Version = v;
        on_change();
    }

    void LocalConfig::SetOnOffTimeout(uint16_t v)
    {
        m_OnOffTimeout = v;
        on_change();
    }

    void LocalConfig::SetOnOffMode(OnOffMode v)
    {
        m_OnOffMode = v;
        on_change();
    }

    void LocalConfig::SetPresenceDetectionMode(PresenceDetectionMode v)
    {
        m_PresenceDetectionMode = v;
        on_change();
    }

    void LocalConfig::FactoryReset()
    {
        esp_littlefs_format(kParitionLabel);
        *this = {};
        on_change();
        on_end();
    }
}
