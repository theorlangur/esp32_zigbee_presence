#ifndef ZB_HELPERS_HPP_
#define ZB_HELPERS_HPP_

#include "esp_zigbee_core.h"
#include <cstring>
#include <string_view>
#include <span>
#include <expected>
#include <utility>
#include "generic_helpers.hpp"

namespace zb
{
    inline constexpr const uint8_t kAnyEP = 0xff;
    inline constexpr const uint16_t kAnyCluster = 0xffff;
    inline constexpr const uint16_t kAnyAttribute = 0xffff;

    inline constexpr const uint16_t kManufactureSpecificCluster = 0xfc00;

    struct ZigbeeStrView
    {
        char *pStr;

        operator void*() { return pStr; }
        uint8_t size() const { return pStr[0]; }
        std::string_view sv() const { return {pStr + 1, pStr[0]}; }
    };

    struct ZigbeeStrRef
    {
        char sz;

        operator void*() { return this; }
        uint8_t size() const { return sz; }
        std::string_view sv() const { return {&sz + 1, sz}; }
    };

    template<size_t N>
    struct ZigbeeStrBuf: public ZigbeeStrRef
    {
        char data[N];
    };

    struct ZigbeeOctetRef
    {
        uint8_t sz;

        operator void*() { return this; }
        uint8_t size() const { return sz; }
        std::span<const uint8_t> sv() const { return {&sz + 1, sz}; }
    };

    template<size_t N>
    struct ZigbeeOctetBuf: public ZigbeeOctetRef
    {
        uint8_t data[N];
    };

    template<size_t N>
    struct ZigbeeStr
    {
        char name[N];
        operator void*() { return name; }
        size_t size() const { return N - 1; }
        std::string_view sv() const { return {name + 1, N - 1}; }
        ZigbeeStrView zsv() const { return {name}; }
        ZigbeeStrRef& zsv_ref() { return *(ZigbeeStrRef*)name; }
    };


    template<size_t N>
    constexpr ZigbeeStr<N> ZbStr(const char (&n)[N])
    {
        static_assert(N < 255, "String too long");
        return [&]<size_t...idx>(std::index_sequence<idx...>){
            return ZigbeeStr<N>{.name={N-1, n[idx]...}};
        }(std::make_index_sequence<N-1>());
    }

    struct APILock
    {
        APILock() { esp_zb_lock_acquire(portMAX_DELAY); }
        ~APILock() { esp_zb_lock_release(); }
    };

    struct DestAddr
    {
        esp_zb_zcl_address_mode_t mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;         /*!< APS addressing mode constants refer to esp_zb_zcl_address_mode_t */
        esp_zb_addr_u addr;

        constexpr DestAddr() = default;
        constexpr DestAddr(uint16_t shortAddr): 
            mode(ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT)
            , addr{.addr_short = shortAddr}
        {}

        DestAddr(esp_zb_ieee_addr_t ieeeAddr): 
            mode(ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT)
        {
            std::memcpy(addr.addr_long, ieeeAddr, sizeof(esp_zb_ieee_addr_t));
        }
    };
    constexpr static DestAddr g_DestCoordinator{uint16_t(0)};

    template<typename T>
    struct TypeDescr;

    template<> struct TypeDescr<uint8_t>  { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_U8; };
    template<> struct TypeDescr<uint16_t> { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_U16; };
    template<> struct TypeDescr<uint32_t> { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_U32; };
    template<> struct TypeDescr<uint64_t> { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_U64; };
    template<> struct TypeDescr<int8_t>   { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_S8; };
    template<> struct TypeDescr<int16_t>  { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_S16; };
    template<> struct TypeDescr<int32_t>  { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_S32; };
    template<> struct TypeDescr<int64_t>  { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_S64; };
    template<> struct TypeDescr<float>    { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_SINGLE; };
    template<> struct TypeDescr<double>   { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_DOUBLE; };

    template<class Str> requires std::is_base_of_v<ZigbeeStrRef, Str>
    struct TypeDescr<Str> { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING; };
    template<class Data>  requires std::is_base_of_v<ZigbeeOctetRef, Data>
    struct TypeDescr<Data> { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING; };

    template<typename T>
    struct TypeInitDefault { 
        static T default_value() { return {}; } 
        static void* to_void(T&v) { return &v; } 
    };

    template<typename T> requires std::is_same_v<T, ZigbeeStrRef> || std::is_same_v<T, ZigbeeOctetRef>
    struct TypeInitDefault<T> 
    { 
        template<class dummy=void>
        static ZigbeeStrRef default_value() { static_assert(std::is_same_v<dummy,void>, "Cannot use ZigbeeStr/OctetRef directly. Must be a limit"); return {}; } 
        static void* to_void(ZigbeeStrRef&v) { return v; } 
    };

    template<size_t N>
    struct MaxDefaultForStr { 
        static ZigbeeStrBuf<N> default_value() { return {N, {0}}; } 
        static void* to_void(ZigbeeStrRef&v) { return v; } 
    };

    template<size_t N>
    struct MaxDefaultForOctet { 
        static ZigbeeOctetBuf<N> default_value() { return {N, {0}}; } 
        static void* to_void(ZigbeeOctetRef&v) { return v; } 
    };

    enum class Access: std::underlying_type_t<esp_zb_zcl_attr_access_t>
    {
        Read = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
        Write = ESP_ZB_ZCL_ATTR_ACCESS_WRITE_ONLY,
        Report = ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        RW = Read | Write,
        RWP = Read | Write | Report,
    };

    inline Access operator|(Access a1, Access a2) { return Access(std::to_underlying(a1) | std::to_underlying(a2)); }

    template<uint8_t EP, uint16_t ClusterID, uint8_t Role, uint16_t Attr, typename T, typename Default = TypeInitDefault<T>>
    struct ZclAttributeAccess
    {
        constexpr static const auto MY_EP = EP;
        constexpr static const auto MY_CLUSTER_ID = ClusterID;
        constexpr static const auto MY_ATTRIBUTE_ID = Attr;
        using AttrType = T;

        using ZCLResult = std::expected<esp_zb_zcl_status_t, esp_zb_zcl_status_t>;
        using ESPResult = std::expected<esp_err_t, esp_err_t>;
        ZCLResult Set(const T &v, bool dbg = false)
        {
            if (dbg)
            {
                FMT_PRINT("Setting EP:{:x} Cluster:{:x} Attr:{:x} Val ptr:{:x}\n", EP, ClusterID, Attr, (size_t)&v);
                if constexpr (std::is_convertible_v<T&, ZigbeeStrRef&>)
                {
                    uint8_t sz = *(uint8_t*)&v;
                    FMT_PRINT("Str: sz:{:x}\n", *(uint8_t*)&v);
                    char *pStr = (char*)&v + 1;
                    for(uint8_t i = 0; i < sz; ++i)
                    {
                        FMT_PRINT("{} ", pStr[i]);
                    }
                    FMT_PRINT("\n");
                }
            }
            auto status = esp_zb_zcl_set_attribute_val(EP, ClusterID, Role, Attr, (void*)&v, false);
            if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
                return std::unexpected(status);
            return ESP_ZB_ZCL_STATUS_SUCCESS;
        }

        ESPResult Report(DestAddr addr = {})
        {
            esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
            report_attr_cmd.address_mode = addr.mode;
            report_attr_cmd.zcl_basic_cmd.dst_addr_u = addr.addr; //coordinator
            report_attr_cmd.zcl_basic_cmd.src_endpoint = EP;
            report_attr_cmd.zcl_basic_cmd.dst_endpoint = {};
            report_attr_cmd.attributeID = Attr;
            report_attr_cmd.cluster_role = Role;
            report_attr_cmd.clusterID = ClusterID;
            if (auto r = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd); r != ESP_OK)
                return std::unexpected(r);
            return ESP_OK;
        }

        static auto AddToCluster(esp_zb_attribute_list_t *custom_cluster, Access a)
        {
            constexpr static const auto MY_TYPE_ID = TypeDescr<T>::kID;
            auto def = Default::default_value();
            return esp_zb_custom_cluster_add_custom_attr(custom_cluster, MY_ATTRIBUTE_ID, MY_TYPE_ID, std::to_underlying(a), Default::to_void(def));
        }
    };

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

    template<SetAttributesHandlingDesc const * const pAttributeHandlers>
    esp_err_t generic_zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
    {
        switch (callback_id) {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            if constexpr (pAttributeHandlers)
            {
                auto *pSetAttr = (esp_zb_zcl_set_attr_value_message_t *)message;
                auto *pFirst = pAttributeHandlers->pHandlers;
                while(pFirst && pFirst->handler)
                {
                    if ((pFirst->ep == kAnyEP || pSetAttr->info.dst_endpoint == pFirst->ep)
                     && (pFirst->cluster_id == kAnyCluster || pSetAttr->info.cluster == pFirst->cluster_id)
                     && (pFirst->attribute_id == kAnyAttribute || pSetAttr->attribute.id == pFirst->attribute_id)
                       )
                        return pFirst->handler(pSetAttr);
                    ++pFirst;
                }
                if (pAttributeHandlers->defaultHandler)
                    return pAttributeHandlers->defaultHandler(pSetAttr);
            }
            break;
        default:
            //ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
            break;
        }
        return ESP_OK;
    }
}
#endif
