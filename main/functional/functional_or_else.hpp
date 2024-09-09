#ifndef FUNCTIONAL_OR_ELSE_H_
#define FUNCTIONAL_OR_ELSE_H_
#include "functional_base.hpp"
namespace functional
{
    template<class V>
        struct or_else_t
        {
            using functional_block_t = void;
            V t;
        };

    template<class V>
        auto or_else(V &&f)
        {
            return or_else_t{std::move(f)};
        }


    template<class ExpVal, class ExpErr, class V>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, or_else_t<V> &&def)->std::expected<ExpVal, ExpErr>
        {
            if (!e)
                return std::move(def).t;
            else
                return std::move(e);
        }
}
#endif
