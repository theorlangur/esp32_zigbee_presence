#ifndef ZB_MAIN_HPP_
#define ZB_MAIN_HPP_
#include <cstdint>
#include "../lib/linked_list.hpp"
#include "../zb_helpers/zbh_handlers_list.hpp"
#include "zcl/esp_zigbee_zcl_command.h"

namespace zb
{

    void setup();

    struct ReadConfigResponseNode: GenericActionNode<ReadConfigResponseNode, esp_zb_zcl_cmd_read_report_config_resp_message_t>{};
    struct ConfigReportResponseNode: GenericActionNode<ConfigReportResponseNode, esp_zb_zcl_cmd_config_report_resp_message_t>{};
    struct ReadAttrResponseNode: GenericActionNode<ReadAttrResponseNode, esp_zb_zcl_cmd_read_attr_resp_message_t>{};
}
#endif
