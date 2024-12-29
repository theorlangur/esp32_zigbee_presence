#include "zb_dev_def_cmd.hpp"
#include "zb_dev_def.hpp"

namespace zb
{
    /**********************************************************************/
    /* Commands                                                           */
    /**********************************************************************/
    esp_err_t ld2412_cmd_restart()
    {
        FMT_PRINT("Restarting...\n");
        g_ld2412.Restart();
        return ESP_OK;
    }

    esp_err_t ld2412_cmd_factory_reset()
    {
        FMT_PRINT("Factory resetting...\n");
        g_ld2412.FactoryReset();
        g_Config.FactoryReset();
        esp_zb_factory_reset();//this resets/reboots the device
        return ESP_OK;
    }

    esp_err_t ld2412_cmd_reset_energy_stat()
    {
        FMT_PRINT("Resetting collected energy stat...\n");
        g_ld2412.ResetEnergyStatistics();
        return ESP_OK;
    }

    esp_err_t ld2412_cmd_switch_bluetooth(bool on)
    {
        FMT_PRINT("Switching bluetooth {}...\n", on);
        g_ld2412.SwitchBluetooth(on);
        return ESP_OK;
    }

    esp_err_t cmd_recheck_binds()
    {
        FMT_PRINT("Got a request to re-check binds...\n");
        for(auto &i : g_State.m_TrackedBinds)
        {
            if (i->IsPassive())
                i->m_CheckReporting = true;
        }
        g_State.m_NeedBindsChecking = true;
        return ESP_OK;
    }

    static const ZbCmdHandler g_Commands[] = {
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_RESTART, &ld2412_cmd_restart>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_FACTORY_RESET, &ld2412_cmd_factory_reset>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_RESET_ENERGY_STAT, &ld2412_cmd_reset_energy_stat>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, LD2412_CMD_BLUETOOTH, &ld2412_cmd_switch_bluetooth, bool>{},
        CmdDescr<PRESENCE_EP, CLUSTER_ID_LD2412, CMD_RECHECK_BINDS, &cmd_recheck_binds>{},
        CmdDescr<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_ID, &cmd_on_off_external_on>{},
        CmdDescr<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID, &cmd_on_off_external_off>{},
        CmdDescr<PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CMD_ON_OFF_ON_WITH_TIMED_OFF_ID, &cmd_on_off_external_on_with_timed_off, OnWithTimedOffPayload>{},
        {} //last, terminating one
    };

    const ZbCmdHandlingDesc g_CommandsDesc{
        /*default*/[](const esp_zb_zcl_custom_cluster_command_message_t *message)->esp_err_t
        {
            FMT_PRINT("Unknown command: ep {:x}; cluster {:x}; cmd {:x}; data size {:x}\n",
                    message->info.dst_endpoint,
                    message->info.cluster,
                    message->info.command.id,
                    message->data.size
                    );
            return ESP_OK;
        },
        g_Commands
    };


}
