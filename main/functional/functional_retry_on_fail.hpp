#ifndef FUNCTIONAL_RETRY_ON_FAIL_H_
#define FUNCTIONAL_RETRY_ON_FAIL_H_
#include "functional_base.hpp"
namespace functional
{
    template<class CB>
        struct retry_on_fail_t
        {
            using functional_block_t = void;
            using Callback = CB;
            CB t;
            int attempts;
            const char *pContext = "";

            template<class ExpVal>
            auto operator()(ExpVal &&v) const
            {
                using namespace internals;
                for(int i = 0; i < attempts - 1; ++i)
                {
                    if (auto te = invoke_continuation_lval(std::forward<ExpVal>(v), *this); te)
                        return te;
                }
                return invoke_continuation_lval(std::forward<ExpVal>(v), *this);
            }
        };
    template<class CB>
        auto retry_on_fail(int attempts, CB &&f, const char *pCtx = "")
        {
            return retry_on_fail_t{std::move(f), attempts};
        }

    template<class CB, class ErrorHandler>
        struct retry_on_fail_err_handle_t: retry_on_fail_t<CB>
    {
        retry_on_fail_err_handle_t(CB &&f, int attempts, ErrorHandler &&e, const char *pCtx):
            retry_on_fail_t<CB>{std::move(f), attempts, pCtx},
            err{std::move(e)}
        {}
        ErrorHandler err;


        template<class ExpVal>
        auto operator()(ExpVal &&v) const
        {
            using namespace internals;
            constexpr auto arity = get_arity_of<ErrorHandler>();

            for(int i = 0; i < (this->attempts - 1); ++i)
            {
                if (auto te = invoke_continuation_lval(std::forward<ExpVal>(v), *this); te)
                    return te;
                else
                { 
                    if constexpr (arity == 1)
                    {
                        if (!this->err(te.error()))
                            return te;//we stop iterating and return error
                    }else if constexpr (arity == 0)
                    {
                        if (!this->err())
                            return te;//we stop iterating and return error
                    }else
                        static_assert(arity == 1, "Error handler must accept either 1 parameter (error) or nothing");
                }
            }
            return invoke_continuation_lval(std::forward<ExpVal>(v), *this);
        }
    };

    template<class CB, class ErrorHandler>
        auto retry_on_fail(int attempts, CB &&f, ErrorHandler &&err, const char *pCtx = "")
        {
            return retry_on_fail_err_handle_t{std::move(f), attempts, std::move(err), pCtx};
        }


    template<class ExpVal, class ExpErr, class V>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, retry_on_fail_t<V> const&def)->std::expected<ExpVal, ExpErr>
        {
            using namespace internals;
            using ret_type_t = std::remove_cvref_t<decltype(def(e.value()))>;
            using ret_error_type_t = typename ret_type_t::error_type;

            if (!e)
            {
                if constexpr (kPrintContextOnError)
                    printf("retry_on_fail(%s) condition returned an error\n", def.pContext);
                if constexpr (std::is_same_v<ret_type_t, std::expected<ExpVal,ExpErr>>)
                    return std::move(e);
                else
                    return ret_type_t(std::unexpected(ret_error_type_t(e.error())));
            }

            return def(e.value());
        }

    template<class ExpVal, class ExpErr, class V, class E>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, retry_on_fail_err_handle_t<V, E> const&def)
        {
            using namespace internals;
            using ret_type_t = std::remove_cvref_t<decltype(def(e.value()))>;
            using ret_error_type_t = typename ret_type_t::error_type;

            if (!e)
            {
                if constexpr (kPrintContextOnError)
                    printf("retry_on_fail_err_handle(%s) condition returned an error\n", def.pContext);
                if constexpr (std::is_same_v<ret_type_t, std::expected<ExpVal,ExpErr>>)
                    return std::move(e);
                else
                    return ret_type_t(std::unexpected(ret_error_type_t(e.error())));
            }

            return def(e.value());
        }
}
#endif
