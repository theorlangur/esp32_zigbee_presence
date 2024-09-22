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
            const char *pContext = "";

            template<class Dummy>
            auto operator()(Dummy &&) const
            {
                return t;
            }
        };

    template<class V>
        auto or_else(V &&f, const char *pCtx = "")
        {
            return or_else_t{std::move(f), pCtx};
        }


    template<class ExpVal, class ExpErr, class V>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, or_else_t<V> const&def)->std::expected<ExpVal, ExpErr>
        {
            if (!e)
            {
                if constexpr (kPrintContextOnError)
                    printf("or_else(%s) passing an error to error handling\n", def.pContext);
                return def(e);
            }
            else
                return std::move(e);
        }
}
#endif
