#include "zb_dev_def.hpp"
#include "esp_check.h"

namespace zb
{
    /**********************************************************************/
    /* ReportAttributeHandler's                                           */
    /**********************************************************************/
    static const ReportAttributeHandler g_ReportHandlers[] = {
        ReportAttributeHandler(kAnyEP, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
            [](const esp_zb_zcl_report_attr_message_t *pReport)->esp_err_t
            {
                //external occupancy report. treating as external presence
                esp_zb_zcl_occupancy_sensing_occupancy_t *pOcc = (esp_zb_zcl_occupancy_sensing_occupancy_t *)pReport->attribute.data.value;
                FMT_PRINT("Got external occupancy report: from {}; State={}\n", pReport->src_address, *pOcc);
                update_external_presence(*pOcc == esp_zb_zcl_occupancy_sensing_occupancy_t::ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED);
                return ESP_OK;
            }),
        ReportAttributeHandler(kAnyEP, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
            [](const esp_zb_zcl_report_attr_message_t *pReport)->esp_err_t
            {
                //external occupancy report. treating as external presence
                uint16_t *pVal = (uint16_t *)pReport->attribute.data.value;
                FMT_PRINT("Got external illuminance report: from {}; value={}\n", pReport->src_address, *pVal);
                if (g_Config.GetIlluminanceExternal())
                {
                    g_State.m_ExternalIlluminance = (*pVal) >> 8;
                    if (auto status = g_LD2412EngineeringLight.Set(g_State.m_ExternalIlluminance); !status)
                    {
                        FMT_PRINT("Failed to set measured light attribute with error {:x}\n", (int)status.error());
                    }
                }else
                {
                    FMT_PRINT("Not configured to use external illuminance. Ignored\n");
                }
                return ESP_OK;
            }),
        ReportAttributeHandler(kAnyEP, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
            [](const esp_zb_zcl_report_attr_message_t *pReport)->esp_err_t
            {
                FMT_PRINT("Got report on on/off attribute from {}; State={}\n", pReport->src_address, *(bool*)pReport->attribute.data.value);

                auto bindIt = g_State.m_TrackedBinds.end();
                if (pReport->src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT)
                    bindIt = g_State.m_TrackedBinds.find(pReport->src_address.u.short_addr, &BindInfo::m_ShortAddr);
                else if (pReport->src_address.addr_type == ESP_ZB_ZCL_ADDR_TYPE_IEEE)
                    bindIt = g_State.m_TrackedBinds.find(ieee_addr{pReport->src_address.u.ieee_addr}, &BindInfo::m_IEEE);

                if (bindIt != g_State.m_TrackedBinds.end())
                {
                    switch((*bindIt)->GetState())
                    {
                    case BindInfo::State::CheckReportingAbility:
                        (*bindIt)->OnReport(pReport);
                        return ESP_OK;
                    case BindInfo::State::Functional:
                        break;
                    default:
                        return ESP_OK;//anything else but functional is ignored
                    }
                    size_t idx = bindIt - g_State.m_TrackedBinds.begin();
                    FMT_PRINT("Found a bind info at index {}\n", idx);
                    if (g_State.m_ValidBinds & (1 << idx))
                    {
                        g_State.m_BindStates &= ~(1 << idx);
                        bool *pVal = (bool *)pReport->attribute.data.value;
                        FMT_PRINT("New state of the bind info: {}\n", *pVal);
                        g_State.m_BindStates |= (int(*pVal) << idx);
                        FMT_PRINT("New binds state: {:x}\n", g_State.m_BindStates);

                        if (!(g_State.m_BindStates & g_State.m_ValidBinds) && g_State.m_LastPresence)//we still have the presence
                        {
                            FMT_PRINT("No controlled bound devices are on. Permitting triggering\n");
                            //re-arm, so that we can again detect and react
                            g_State.m_TriggerAllowed = true;
                        }
                    }else
                    {
                        FMT_PRINT("BindInfo has an invalid state\n");
                    }
                }else
                {
                    //not found. Unbind?
                }

                return ESP_OK;
            }
            ),

        {}//last one
    };

    const ReportAttributesHandlingDesc g_ReportHandlingDesc = {
        /*default*/[](const esp_zb_zcl_report_attr_message_t *message)->esp_err_t
        {
            esp_err_t ret = ESP_OK;

            ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
            ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                                message->status);
            ESP_LOGI(TAG, "Received report message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d), data type(%d)", message->dst_endpoint, message->cluster,
                     message->attribute.id, message->attribute.data.size, message->attribute.data.type);

            return ret;
        }
        ,g_ReportHandlers
    };

}
