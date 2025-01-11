#ifndef ZBH_APSDE_BINDS_HPP_
#define ZBH_APSDE_BINDS_HPP_
#include "zbh_apsde.hpp"
#include "zbh_attributes.hpp"

namespace zb
{
    enum class APSME_Commands: uint8_t
    {
        Bind = 0x21,
        Unbind = 0x22,
    };

    struct APSME_BindUnbindReq
    {
        esp_zb_ieee_addr_t src;
        uint8_t src_ep;
        uint16_t cluster_id;
        uint8_t dst_addr_mode;//only 0x03 is interesting
        esp_zb_ieee_addr_t dst;
        uint8_t dst_ep;
    }__attribute__((packed));

    using bind_unbind_callback_t = void(*)(APSME_Commands cmd, APSME_BindUnbindReq &req);
    struct BindUnbind_Handler: NonCopyable, NonMovable
    {
        bind_unbind_callback_t cb;
        uint16_t cluster_id = kAnyCluster;
        uint8_t src_ep = kAnyEP;
        uint8_t dst_ep = kAnyEP;
        BindUnbind_Handler *pNext = this;//indication that we're not in the list

        ~BindUnbind_Handler();
        void Add();
        void Remove();
    };
}
#endif
