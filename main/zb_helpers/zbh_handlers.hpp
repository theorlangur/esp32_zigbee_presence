#ifndef ZBH_HANDLERS_HPP_
#define ZBH_HANDLERS_HPP_

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

    /**********************************************************************/
    /* ZbCmdHandler                                                       */
    /**********************************************************************/
    using typeless_cmd_handler = esp_err_t (*)(const esp_zb_zcl_custom_cluster_command_message_t *message);
    struct ZbCmdHandler;
    struct ZbCmdHandlingDesc
    {
        const typeless_cmd_handler defaultHandler = nullptr;
        ZbCmdHandler const * const pHandlers = nullptr;
    };

    template<class TargetType, auto CmdLambda>
    esp_err_t generic_zb_cmd_handler(const esp_zb_zcl_custom_cluster_command_message_t *message)
    {
        if constexpr (requires{ CmdLambda(*message); })//raw message access has prio
            return CmdLambda(*message);
        else if constexpr (requires{ CmdLambda(message->data); })//raw access but data only
            return CmdLambda(message->data);
        else if constexpr (requires{ CmdLambda(message->data.value, message->data.size); })//raw access but directly void* and size
            return CmdLambda(message->data.value, message->data.size);
        else if constexpr (requires{ CmdLambda(); })
            return CmdLambda();//no data expected
        else if constexpr (requires(TargetType const& m){ CmdLambda(m); })//case for accepting TargetType by const ref
        {
            if constexpr (requires{ { TargetType::from(message) }->std::convertible_to<std::optional<TargetType>>; })//case when TargetType can convert to self
            {
                if (auto r = TargetType::from(message); r)
                    return CmdLambda(*r);
                else
                    return ESP_ERR_INVALID_ARG;
            }
            else//case for a direct casting
            {
                if (sizeof(TargetType) <= message->data.size)
                    return CmdLambda(*static_cast<const TargetType*>(message->data.value));
                else
                    return ESP_ERR_INVALID_ARG;
            }
        }
        else if constexpr (requires{ TargetType::decompose(message); })//advanced tuple-based invocation
        {
            using TupleResult = std::remove_cvref_t<decltype(TargetType::decompose(message))>;
            return []<class OptionalTuple, size_t... idx>(OptionalTuple const& t, std::index_sequence<idx...>)
            {
                if (t)
                {
                    auto const& tup = *t;
                    return CmdLambda(std::get<idx>(tup)...);
                }else
                    return ESP_ERR_INVALID_ARG;
            }(TargetType::decompose(message), std::make_index_sequence<std::tuple_size_v<TupleResult>>());
        }else
        {
            static_assert(sizeof(CmdLambda) == 0, "Invalid callback");
        }
        return ESP_FAIL;
    }

    template<uint8_t _ep, uint16_t _cluster, uint8_t _cmd, auto h, class TargetType = void>
    struct CmdDescr{};

    struct ZbCmdHandler
    {
        template<uint8_t _ep, uint16_t _cluster, uint8_t _cmd, auto h, class TargetType>
        ZbCmdHandler(CmdDescr<_ep,_cluster,_cmd, h, TargetType>): 
            ep(_ep)
            , cluster_id(_cluster)
            , cmd_id(_cmd)
            , handler(&generic_zb_cmd_handler<TargetType, h>)
        {}
        ZbCmdHandler() = default;

        uint8_t ep;
        uint16_t cluster_id;
        uint8_t cmd_id;
        typeless_cmd_handler handler = nullptr;
    };

    struct ZbCmdResponse
    {
        //returns true - auto remove from registered callbacks, false - leave in callbacks
        using cmd_resp_handler_t = bool(*)(uint8_t cmd_id, esp_zb_zcl_status_t status_code, esp_zb_zcl_cmd_info_t *, void *user_ctx);
        struct RespHandlers
        {
            cmd_resp_handler_t h;
            void *user_ctx;
        };
        RespHandlers m_RegisteredResponseCallbacks[256] = {};
        uint16_t m_ClusterId = 0xffff;

        static constexpr size_t kMaxClusters = 2;
        static ZbCmdResponse g_CmdResponseRegistry[kMaxClusters];
    public:
        static ZbCmdResponse* FindOrAquireRegistry(uint16_t clusterId)
        {
            for(auto &r : g_CmdResponseRegistry)
            {
                if (r.m_ClusterId == clusterId)
                    return &r;
                if (r.m_ClusterId == 0xffff)
                {
                    //free
                    r.m_ClusterId = clusterId;
                    return &r;
                }
            }
            return nullptr;
        }
        static ZbCmdResponse* FindRegistry(uint16_t clusterId)
        {
            for(auto &r : g_CmdResponseRegistry)
            {
                if (r.m_ClusterId == clusterId)
                    return &r;
            }
            return nullptr;
        }

        static RespHandlers Register(uint16_t clusterId, uint8_t cmd, cmd_resp_handler_t cb, void *user_ctx)
        {
            auto *pR = FindOrAquireRegistry(clusterId);
            if (!pR) return {};

            auto prev = pR->m_RegisteredResponseCallbacks[cmd];
            pR->m_RegisteredResponseCallbacks[cmd] = {cb, user_ctx};
            return prev;
        }

        static RespHandlers Unregister(uint16_t clusterId, uint8_t cmd)
        {
            auto *pR = FindRegistry(clusterId);
            if (pR)
            {
                auto prev = pR->m_RegisteredResponseCallbacks[cmd];
                pR->m_RegisteredResponseCallbacks[cmd] = {nullptr, nullptr};
                return prev;
            }
            return {};
        }
    };
    inline ZbCmdResponse ZbCmdResponse::g_CmdResponseRegistry[kMaxClusters];

    using seq_nr_t = uint8_t;
    struct ZbCmdSend
    {
        //returns true - auto remove from registered callbacks, false - leave in callbacks
        using cmd_send_handler_t = void(*)(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx);
        struct SendHandlers
        {
            cmd_send_handler_t h;
            void *user_ctx;
        };
        inline static SendHandlers g_RegisteredSendCallbacks[256] = {};
    public:
        static SendHandlers Register(uint8_t seqNr, cmd_send_handler_t cb, void *user_ctx)
        {
            auto prev = g_RegisteredSendCallbacks[seqNr];
            g_RegisteredSendCallbacks[seqNr] = {cb, user_ctx};
            return prev;
        }

        static void Unregister(uint8_t seqNr)
        {
            g_RegisteredSendCallbacks[seqNr] = {nullptr, nullptr};
        }

        static void handler(esp_zb_zcl_command_send_status_message_t message)
        {
            auto &entry = g_RegisteredSendCallbacks[message.tsn];
            if (entry.h)
            {
                auto e = entry;
                entry = {nullptr, nullptr};
                e.h(&message, e.user_ctx);
            }else
            {
                FMT_PRINT("[task={}; lock={}]Send status(no callback): seqNr={}; dst={},  status={:x}\n", (const char*)pcTaskGetName(nullptr), APILock::g_State, message.tsn, message.dst_addr, message.status);
            }
        }
    };


    /**********************************************************************/
    /* generic_zb_action_handler                                          */
    /**********************************************************************/
    template<
          SetAttributesHandlingDesc const * const pAttributeHandlers = nullptr
        , ZbCmdHandlingDesc const * const pCmdHandlers = nullptr
        >
    esp_err_t generic_zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
    {
        switch (callback_id) {
        case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
            {
                if constexpr (pCmdHandlers)
                {
                    auto *pCmdMsg = (esp_zb_zcl_custom_cluster_command_message_t *)message;
#if defined(ZB_DBG)
                {
                    FMT_PRINT("Cmd: for EP {:x} Cluster {:x} Cmd: {:x}\n", pCmdMsg->info.dst_endpoint, pCmdMsg->info.cluster, pCmdMsg->info.command.id);
                    FMT_PRINT("Data size: {}\n", pCmdMsg->data.size);
                    if (pCmdMsg->data.size)
                    {
                        uint8_t *pData = (uint8_t *)pCmdMsg->data.value;
                        for(int i = 0; i < pCmdMsg->data.size; ++i)
                            FMT_PRINT(" {:x}", pData[i]);
                        FMT_PRINT("\n");
                    }
                }
#endif
                    auto *pFirst = pCmdHandlers->pHandlers;
                    while(pFirst && pFirst->handler)
                    {
                        if ((pFirst->ep == kAnyEP || pCmdMsg->info.dst_endpoint == pFirst->ep)
                         && (pFirst->cluster_id == kAnyCluster || pCmdMsg->info.cluster == pFirst->cluster_id)
                         && (pFirst->cmd_id == kAnyCmd || pCmdMsg->info.command.id == pFirst->cmd_id)
                           )
                            return pFirst->handler(pCmdMsg);
                        ++pFirst;
                    }
                    if (pCmdHandlers->defaultHandler)
                        return pCmdHandlers->defaultHandler(pCmdMsg);
                }
            }
            break;
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            if constexpr (pAttributeHandlers)
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
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
            {
                auto *pCmdRespMsg = (esp_zb_zcl_cmd_default_resp_message_t *)message;
                auto *pR = ZbCmdResponse::FindRegistry(pCmdRespMsg->info.cluster);
                auto cb = pR ? pR->m_RegisteredResponseCallbacks[pCmdRespMsg->resp_to_cmd] : ZbCmdResponse::RespHandlers{nullptr, nullptr};
                if (cb.h)
                {
                    if (!cb.h(pCmdRespMsg->resp_to_cmd, pCmdRespMsg->status_code, &pCmdRespMsg->info, cb.user_ctx))
                        pR->m_RegisteredResponseCallbacks[pCmdRespMsg->resp_to_cmd] = {nullptr, nullptr};
                }else
                {
#ifndef NDEBUG
                    using clock_t = std::chrono::system_clock;
                    auto now = clock_t::now();
                    auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
                    FMT_PRINT("{} Response to Zigbee command ({:x}) with no callback to handle. Status: {:x}; From: addr={:x}; ep={}; cluster={:x}\n"
                            , _n, pCmdRespMsg->resp_to_cmd
                            , (uint16_t)pCmdRespMsg->status_code
                            , pCmdRespMsg->info.src_address
                            , pCmdRespMsg->info.src_endpoint
                            , pCmdRespMsg->info.cluster
                            );
#endif
                }
            }
            break;
        default:
            {
#ifndef NDEBUG
            //using clock_t = std::chrono::system_clock;
            //auto now = clock_t::now();
            //auto _n = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
            FMT_PRINT("task={}; lock={} Receive Zigbee action({:x}) callback\n", (const char*)pcTaskGetName(nullptr), APILock::g_State, (int)callback_id);
#endif
            }
            break;
        }
        return ESP_OK;
    }
}

#endif
