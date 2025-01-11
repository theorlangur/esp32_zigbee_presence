#ifndef ZBH_HANDLERS_CMD_HPP_
#define ZBH_HANDLERS_CMD_HPP_

#include "zbh_types.hpp"
#include "zbh_handlers.hpp"
#include "lib_linked_list.hpp"

namespace zb
{
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
    using seq_nr_t = uint16_t;
    static constexpr seq_nr_t kInvalidTSN = 0xffff;

    struct ZbCmdResponse
    {
        using cmd_resp_handler_t = void(*)(uint8_t cmd_id, esp_zb_zcl_status_t status_code, esp_zb_zcl_cmd_info_t *, void *user_ctx);
        struct Node: ::Node
        {
            uint16_t cluster_id;
            uint8_t cmd_id;
            seq_nr_t tsn;
            cmd_resp_handler_t cb;
            void *user_ctx;

            void RegisterSelf();
        };

        static LinkedListT<Node> g_CmdResponseList;
    };


    /**********************************************************************/
    /* ZbCmdSend                                                          */
    /**********************************************************************/
    struct ZbCmdSend
    {
        //returns true - auto remove from registered callbacks, false - leave in callbacks
        using cmd_send_handler_t = void(*)(esp_zb_zcl_command_send_status_message_t *pSendStatus, void *user_ctx);
        struct Node: ::Node
        {
            seq_nr_t tsn;
            cmd_send_handler_t cb;
            void *user_ctx;
            bool skipCoordinator = true;

            void RegisterSelf();
        };

        static LinkedListT<Node> g_SendStatusList;
    public:
        static void handler(esp_zb_zcl_command_send_status_message_t message);
    };


    /**********************************************************************/
    /* Action handlers                                                    */
    /**********************************************************************/
    //ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID
    template<ZbCmdHandlingDesc const & cmdHandlers>
    esp_err_t cmd_custom_cluster_req_cb(const void *message)
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
        auto *pFirst = cmdHandlers.pHandlers;
        while(pFirst && pFirst->handler)
        {
            if ((pFirst->ep == kAnyEP || pCmdMsg->info.dst_endpoint == pFirst->ep)
                    && (pFirst->cluster_id == kAnyCluster || pCmdMsg->info.cluster == pFirst->cluster_id)
                    && (pFirst->cmd_id == kAnyCmd || pCmdMsg->info.command.id == pFirst->cmd_id)
               )
                return pFirst->handler(pCmdMsg);
            ++pFirst;
        }
        if (cmdHandlers.defaultHandler)
            return cmdHandlers.defaultHandler(pCmdMsg);
        return ESP_OK;
    }

    //ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID
    inline esp_err_t cmd_response_action_handler(const void *message)
    {
        auto *pCmdRespMsg = (esp_zb_zcl_cmd_default_resp_message_t *)message;
        for(ZbCmdResponse::Node *pN : ZbCmdResponse::g_CmdResponseList)
        {
            if (pN->cluster_id == pCmdRespMsg->info.cluster 
                && pN->cmd_id == pCmdRespMsg->resp_to_cmd
                && pCmdRespMsg->info.header.tsn == pN->tsn
                )
            {
                //
                if (pN->cb)
                {
                    pN->cb(pCmdRespMsg->resp_to_cmd, pCmdRespMsg->status_code, &pCmdRespMsg->info, pN->user_ctx);
                }else
                {
                    //?
                }
                return ESP_OK;
            }
        }

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
        return ESP_OK;
    }
}

#endif
