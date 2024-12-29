#ifndef ZB_DEV_DEF_CMD_HPP_
#define ZB_DEV_DEF_CMD_HPP_

#include "zb_dev_def_const.hpp"

namespace zb
{
    /**********************************************************************/
    /* Commands IDs                                                       */
    /**********************************************************************/
    static constexpr const uint8_t LD2412_CMD_RESTART = 0;
    static constexpr const uint8_t LD2412_CMD_FACTORY_RESET = 1;
    static constexpr const uint8_t LD2412_CMD_RESET_ENERGY_STAT = 2;
    static constexpr const uint8_t LD2412_CMD_BLUETOOTH = 3;
    static constexpr const uint8_t CMD_RECHECK_BINDS = 4;

    /**********************************************************************/
    /* Commands                                                           */
    /**********************************************************************/
    esp_err_t ld2412_cmd_restart();
    esp_err_t ld2412_cmd_factory_reset();
    esp_err_t ld2412_cmd_reset_energy_stat();
    esp_err_t ld2412_cmd_switch_bluetooth(bool on);
    esp_err_t cmd_recheck_binds();
}
#endif
