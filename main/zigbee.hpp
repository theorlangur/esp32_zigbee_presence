#ifndef ZIGBEE_H_
#define ZIGBEE_H_
#include <cstdint>
#include "lib/linked_list.hpp"
#include "zcl/esp_zigbee_zcl_command.h"

namespace zb
{
    constexpr uint8_t PRESENCE_EP = 1;

    void setup();


    struct ReadConfigResponseNode: Node
    {
        //return 'true' - was handled, no need to keep iterating
        virtual bool Notify(esp_zb_zcl_cmd_read_report_config_resp_message_t *pResp) = 0;
    };
    void RegisterReadConfigResponseHandler(ReadConfigResponseNode &n);

    struct ConfigReportResponseNode: Node
    {
        //return 'true' - was handled, no need to keep iterating
        virtual bool Notify(esp_zb_zcl_cmd_config_report_resp_message_t *pResp) = 0;
    };
    void RegisterConfigReportResponseHandler(ConfigReportResponseNode &n);

    const esp_zb_ieee_addr_t& GetMyIEEE();
}
#endif
