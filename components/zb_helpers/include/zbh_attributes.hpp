#ifndef ZBH_ATTRIBUTES_HPP_
#define ZBH_ATTRIBUTES_HPP_

#include "zbh_types.hpp"

namespace zb
{
    inline constexpr const uint8_t kAnyEP = 0xff;
    inline constexpr const uint16_t kAnyCluster = 0xffff;
    inline constexpr const uint16_t kAnyAttribute = 0xffff;
    inline constexpr const uint16_t kAnyCmd = 0xff;

    inline constexpr const uint16_t kManufactureSpecificCluster = 0xfc00;

    template<typename T>
    struct TypeDescr;

    template<typename T> requires std::is_enum_v<T>
    struct TypeDescr<T>  
    { 
        static_assert(sizeof(T) <= 2, "Only 8/16 bits enums are supported");
        static constexpr const esp_zb_zcl_attr_type_t kID = sizeof(T) == 1 ? ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM : ESP_ZB_ZCL_ATTR_TYPE_16BIT_ENUM; 
    };
    template<> struct TypeDescr<bool>     { static constexpr const esp_zb_zcl_attr_type_t kID = ESP_ZB_ZCL_ATTR_TYPE_BOOL; static_assert(sizeof(bool) == 1); };
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
#ifndef NDEBUG
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
#endif
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
            //report_attr_cmd.cluster_role = Role;
            report_attr_cmd.clusterID = ClusterID;
            if (auto r = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd); r != ESP_OK)
                return std::unexpected(r);
            return ESP_OK;
        }

        static auto AddToCluster(esp_zb_attribute_list_t *custom_cluster, Access a)
        {
            auto def = Default::default_value();
            return AddToCluster(custom_cluster, a, def);
        }

        static auto AddToCluster(esp_zb_attribute_list_t *custom_cluster, Access a, auto def)
        {
            constexpr static const auto MY_TYPE_ID = TypeDescr<T>::kID;
            return esp_zb_custom_cluster_add_custom_attr(custom_cluster, MY_ATTRIBUTE_ID, MY_TYPE_ID, std::to_underlying(a), Default::to_void(def));
        }
    };

    template<uint8_t EP, uint16_t ClusterID>
    struct ZclServerCluster
    {
        template<uint16_t Attr, typename T, typename Default = TypeInitDefault<T>>
        using Attribute = ZclAttributeAccess<EP, ClusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, Attr, T, Default>;
    };

    template<uint8_t EP, uint16_t ClusterID>
    struct ZclClientCluster
    {
        template<uint16_t Attr, typename T, typename Default = TypeInitDefault<T>>
        using Attribute = ZclAttributeAccess<EP, ClusterID, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE, Attr, T, Default>;
    };
}

#endif
