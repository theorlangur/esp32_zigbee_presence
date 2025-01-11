#ifndef ZBH_HANDLERS_LIST_HPP_
#define ZBH_HANDLERS_LIST_HPP_

#include "zbh_types.hpp"
#include "lib_linked_list.hpp"

namespace zb
{
    //node-list subscription-based action handler
    template<class N>
    concept action_handler_node_c = requires(N *pNode)
    { 
        typename N::message_t;
        { pNode->Notify((typename N::message_t*)nullptr) }->std::same_as<bool>;
    };

    template<class Self, class _message_t>
    struct GenericActionNode: Node
    {
        using message_t = _message_t;
        //return 'true' - was handled, no need to keep iterating
        virtual bool Notify(message_t *pResp) = 0;

        void RegisterSelf() { Register(static_cast<Self&>(*this)); }

        static void Register(Self &n)
        {
            g_List += n;
        }
        static LinkedListT<Self> g_List;
    };
    template<class Self, class _message_t>
    LinkedListT<Self> GenericActionNode<Self, _message_t>::g_List;


    template<action_handler_node_c N>
    esp_err_t generic_node_list_handler(const void *message)
    {
        typename N::message_t *pResp = (typename N::message_t *)message;
        //FMT_PRINT("Config Report Response:\n");
        for(auto *pNode : N::g_List)
        {
            if (pNode->Notify(pResp))
                break;
        }
        return ESP_OK;
    }
}

#endif
