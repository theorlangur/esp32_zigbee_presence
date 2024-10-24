#ifndef FUNCTIONAL_TRANSFORM_ERROR_H_
#define FUNCTIONAL_TRANSFORM_ERROR_H_
#include "functional_base.hpp"
namespace functional
{
    template<class T>
        struct transform_error_t
        {
            using functional_block_t = void;
            using Callback = T;
            T t;
            const char *pContext = "";

            template<class ExpVal>
            auto operator()(ExpVal &&v) const
            {
                static_assert(false, "Makes no sense to call it directly. It converts an error");
            }
        };

    template<class T>
        auto transform_error(T &&f, const char *pCtx = "")
        {
            return transform_error_t{std::forward<T>(f), pCtx};
        }

    template<class ExpVal, class ExpErr, class TransformErrV>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, transform_error_t<TransformErrV> const&tr_err)->std::expected<ExpVal, std::invoke_result_t<TransformErrV, ExpErr&>>
        {
            if (!e)
            {
                if constexpr (kPrintContextOnError)
                    printf("transform_error(%s) transforming an error\n", tr_err.pContext);
                return std::unexpected(tr_err.t(e.error()));
            }

            return e.value();
        }
}
#endif
