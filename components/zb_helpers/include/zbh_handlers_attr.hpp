#ifndef ZBH_HANDLERS_ATTR_HPP_
#define ZBH_HANDLERS_ATTR_HPP_

#include "zbh_attributes.hpp"

namespace zb
{
    /**********************************************************************/
    /* SetAttributeHandler                                                */
    /**********************************************************************/
    template<class AttrType>
    using typed_set_attribute_handler = esp_err_t(*)(AttrType const& value, const esp_zb_zcl_set_attr_value_message_t *message);

    using typeless_set_attribute_handler = esp_err_t (*)(const esp_zb_zcl_set_attr_value_message_t *message);

    template<class AttrValueType, typed_set_attribute_handler<AttrValueType> TypedHandler>
    esp_err_t generic_zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
    {
        return TypedHandler(*(AttrValueType*)message->attribute.data.value, message);
    }

    template<class ZclAttrType, typed_set_attribute_handler<typename ZclAttrType::AttrType> h>
    struct AttrDescr{};

    struct SetAttributeHandler
    {
        template<class ZclAttrType, typed_set_attribute_handler<typename ZclAttrType::AttrType> h>
        SetAttributeHandler(AttrDescr<ZclAttrType, h>): 
            ep(ZclAttrType::MY_EP)
            , cluster_id(ZclAttrType::MY_CLUSTER_ID)
            , attribute_id(ZclAttrType::MY_ATTRIBUTE_ID)
            , handler(&generic_zb_attribute_handler<typename ZclAttrType::AttrType, h>)
        {}
        SetAttributeHandler(uint8_t ep, uint16_t cluster_id, uint16_t attr_id, auto h): 
            ep(ep)
            , cluster_id(cluster_id)
            , attribute_id(attr_id)
            , handler(h)
        {}
        SetAttributeHandler() = default;

        uint8_t ep;
        uint16_t cluster_id;
        uint16_t attribute_id;
        typeless_set_attribute_handler handler = nullptr;
    };

    struct SetAttributesHandlingDesc
    {
        const typeless_set_attribute_handler defaultHandler = nullptr;
        SetAttributeHandler const * const pHandlers = nullptr;
    };

    template<SetAttributesHandlingDesc const &attributeHandlers>
    esp_err_t set_attr_value_cb(const void *message)
    {
        auto *pSetAttr = (esp_zb_zcl_set_attr_value_message_t *)message;
#if defined(ZB_DBG)
        {
            auto sz = pSetAttr->attribute.data.size;
            FMT_PRINT("Set Attr: type: {:x}; size: {}\n", (int)pSetAttr->attribute.data.type, sz);
            uint8_t *pData = (uint8_t *)pSetAttr->attribute.data.value;
            for(int i = 0; i < pSetAttr->attribute.data.size; ++i)
                FMT_PRINT(" {:x}", pData[i]);
            FMT_PRINT("\n");
        }
#endif

        auto *pFirst = attributeHandlers.pHandlers;
        while(pFirst && pFirst->handler)
        {
            if ((pFirst->ep == kAnyEP || pSetAttr->info.dst_endpoint == pFirst->ep)
                    && (pFirst->cluster_id == kAnyCluster || pSetAttr->info.cluster == pFirst->cluster_id)
                    && (pFirst->attribute_id == kAnyAttribute || pSetAttr->attribute.id == pFirst->attribute_id)
               )
                return pFirst->handler(pSetAttr);
            ++pFirst;
        }
        if (attributeHandlers.defaultHandler)
            return attributeHandlers.defaultHandler(pSetAttr);
        return ESP_OK;
    }
}

#endif
