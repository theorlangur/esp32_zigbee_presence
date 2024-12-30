#include "zb_dev_def.hpp"

namespace zb
{
    static ZbAlarmExt16 g_DelayedExternalUpdate;

    static void update_external_attributes()
    {
        g_ExternalOnOff.Set(g_State.m_LastPresenceExternal);
        esp_zb_zcl_occupancy_sensing_occupancy_t val = g_State.m_LastPresence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
        if (auto status = g_OccupancyState.Set(val); !status)
        {
            FMT_PRINT("Failed to set occupancy attribute with error {:x}\n", (int)status.error());
        }
    };

    /**********************************************************************/
    /* On/Off server cluster commands for external sensor                 */
    /**********************************************************************/
    void on_external_on_timer_finished(void* param)
    {
        FMT_PRINT("External timer finished\n");
        g_State.m_LastPresenceExternal = false;
        //g_ExternalOnOff.Report(g_DestCoordinator);
        if (update_presence_state())
        {
            if (send_on_off(g_State.m_LastPresence))
                g_DelayedExternalUpdate.Setup(update_external_attributes, kDelayedAttrChangeTimeout);
            else
                update_external_attributes();
        }else
        {
            if (auto status = g_ExternalOnOff.Set(g_State.m_LastPresenceExternal); !status)
            {
                FMT_PRINT("Failed to set on/off state attribute with error {:x}\n", (int)status.error());
            }
        }
    }

    esp_err_t cmd_on_off_external_on()
    {
        FMT_PRINT("Switching external on/off ON\n");
        g_State.m_LastPresenceExternal = true;
        if (auto et = g_Config.GetExternalOnOffTimeout(); et > 0)
            g_State.StartExternalTimer(&on_external_on_timer_finished, et * 1000);
        else
            g_State.m_ExternalRunningTimer.Cancel();

        if (update_presence_state())
        {
            if (send_on_off(g_State.m_LastPresence))
                g_DelayedExternalUpdate.Setup(update_external_attributes, kDelayedAttrChangeTimeout);
            else
                update_external_attributes();
        }else
        {
            g_ExternalOnOff.Set(g_State.m_LastPresenceExternal);
        }
        return ESP_OK;
    }

    esp_err_t cmd_on_off_external_off()
    {
        FMT_PRINT("Switching external on/off OFF {}...\n");
        g_State.m_LastPresenceExternal = false;
        g_State.m_ExternalRunningTimer.Cancel();
        if (update_presence_state())
        {
            if (send_on_off(g_State.m_LastPresence))
                g_DelayedExternalUpdate.Setup(update_external_attributes, kDelayedAttrChangeTimeout);
            else
                update_external_attributes();
        }else
            g_ExternalOnOff.Set(g_State.m_LastPresenceExternal);
        return ESP_OK;
    }

    std::optional<OnWithTimedOffPayload> OnWithTimedOffPayload::from(const esp_zb_zcl_custom_cluster_command_message_t *message)
    {
        if (message->data.size == 5)
        {
            OnWithTimedOffPayload r;
            std::memcpy(&r.on_off_ctrl, (const uint8_t*)message->data.value, 1);
            std::memcpy(&r.on_time, (const uint8_t*)message->data.value + 1, 2);
            std::memcpy(&r.off_wait_time, (const uint8_t*)message->data.value + 3, 2);
            return r;
        }
        return std::nullopt;
    }

    esp_err_t cmd_on_off_external_on_with_timed_off(OnWithTimedOffPayload const& data)
    {
        FMT_PRINT("Switching external on/off ON with params: ctrl: {}; on time: {} (deci-seconds); off wait time: {} (deci-seconds)\n", data.on_off_ctrl, data.on_time, data.off_wait_time);
        g_State.m_LastPresenceExternal = true;
        g_State.m_ExternalRunningTimer.Cancel();
        if (data.on_time > 0 && data.on_time < 0xfffe)
        {
            //(re-)start timer
            uint32_t t = data.on_time * 100;
            if (auto et = g_Config.GetExternalOnOffTimeout(); et > 0)
                t = et * 1000;
            g_State.StartExternalTimer(&on_external_on_timer_finished, t);
        }
        if (update_presence_state())
        {
            if (send_on_off(g_State.m_LastPresence))
                g_DelayedExternalUpdate.Setup(update_external_attributes, kDelayedAttrChangeTimeout);
            else
                update_external_attributes();
        }else
            g_ExternalOnOff.Set(g_State.m_LastPresenceExternal);
        return ESP_OK;
    }

    void update_external_presence(bool _new)
    {
        if (g_State.m_LastPresenceExternal != _new)
        {
            FMT_PRINT("Switching external to {}\n", _new);
            g_State.m_LastPresenceExternal = _new;

            if (g_State.m_LastPresenceExternal)
            {
                if (auto et = g_Config.GetExternalOnOffTimeout(); et > 0)
                    g_State.StartExternalTimer(&on_external_on_timer_finished, et * 1000);
                else
                    g_State.m_ExternalRunningTimer.Cancel();
            }else
                g_State.m_ExternalRunningTimer.Cancel();

            if (update_presence_state())
            {
                if (send_on_off(g_State.m_LastPresence))
                    g_DelayedExternalUpdate.Setup(update_external_attributes, kDelayedAttrChangeTimeout);
                else
                    update_external_attributes();
            }else
                g_ExternalOnOff.Set(g_State.m_LastPresenceExternal);
        }
    }

    esp_err_t ias_zone_state_change(const void *_m)
    {
        esp_zb_zcl_ias_zone_status_change_notification_message_t *message = (esp_zb_zcl_ias_zone_status_change_notification_message_t *)_m;
        FMT_PRINT("ias zone status change from {}: to {:x}; extended: {:x}; delay: {:x}; zone id {:x};\n"
                , message->info.src_address
                , message->zone_status
                , message->extended_status
                , message->delay
                , message->zone_id
                );

        update_external_presence(message->zone_status & 1);
        return ESP_OK;
    }

}
