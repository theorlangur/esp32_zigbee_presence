#ifndef ZBH_HANDLERS_REPORT_HPP_
#define ZBH_HANDLERS_REPORT_HPP_

#include "zbh_attributes.hpp"

namespace zb
{
    /**********************************************************************/
    /* ReportAttributeHandler                                             */
    /**********************************************************************/
    template<class AttrType>
    using typed_report_attribute_handler = esp_err_t(*)(AttrType const& value, const esp_zb_zcl_report_attr_message_t *message);

    using typeless_report_attribute_handler = esp_err_t (*)(const esp_zb_zcl_report_attr_message_t *message);

    template<class AttrValueType, typed_report_attribute_handler<AttrValueType> TypedHandler>
    esp_err_t generic_zb_report_handler(const esp_zb_zcl_report_attr_message_t *message)
    {
        return TypedHandler(*(AttrValueType*)message->attribute.data.value, message);
    }

    template<class ZclAttrType, typed_report_attribute_handler<typename ZclAttrType::AttrType> h>
    struct ReportDescr{};

    struct ReportAttributeHandler
    {
        template<class ZclAttrType, typed_report_attribute_handler<typename ZclAttrType::AttrType> h>
        ReportAttributeHandler(ReportDescr<ZclAttrType, h>): 
            ep(ZclAttrType::MY_EP)
            , cluster_id(ZclAttrType::MY_CLUSTER_ID)
            , attribute_id(ZclAttrType::MY_ATTRIBUTE_ID)
            , handler(&generic_zb_report_handler<typename ZclAttrType::AttrType, h>)
        {}
        ReportAttributeHandler(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, auto h): 
            ep(ep)
            , cluster_id(cluster_id)
            , attribute_id(attr_id)
            , handler(h)
        {}
        ReportAttributeHandler() = default;

        uint8_t ep;
        uint16_t cluster_id;
        uint16_t attribute_id;
        typeless_report_attribute_handler handler = nullptr;
    };

    struct ReportAttributesHandlingDesc
    {
        const typeless_report_attribute_handler defaultHandler = nullptr;
        ReportAttributeHandler const * const pHandlers = nullptr;
    };

    template<ReportAttributesHandlingDesc const &reportHandlers>
    esp_err_t report_attr_cb(const void *message)
    {
        auto *pReport = (esp_zb_zcl_report_attr_message_t *)message;
#if defined(ZB_DBG)
        {
            auto sz = pReport->attribute.data.size;
            FMT_PRINT("Report Attr: type: {:x}; size: {}\n", (int)pReport->attribute.data.type, sz);
            uint8_t *pData = (uint8_t *)pReport->attribute.data.value;
            for(int i = 0; i < pReport->attribute.data.size; ++i)
                FMT_PRINT(" {:x}", pData[i]);
            FMT_PRINT("\n");
        }
#endif

        auto *pFirst = reportHandlers.pHandlers;
        while(pFirst && pFirst->handler)
        {
            if ((pFirst->ep == kAnyEP || pReport->dst_endpoint == pFirst->ep)
                    && (pFirst->cluster_id == kAnyCluster || pReport->cluster == pFirst->cluster_id)
                    && (pFirst->attribute_id == kAnyAttribute || pReport->attribute.id == pFirst->attribute_id)
               )
                return pFirst->handler(pReport);
            ++pFirst;
        }
        if (reportHandlers.defaultHandler)
            return reportHandlers.defaultHandler(pReport);
        return ESP_OK;
    }
}

#endif
