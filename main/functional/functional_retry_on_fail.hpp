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
        };
    template<class CB>
        auto retry_on_fail(int attempts, CB &&f)
        {
            return retry_on_fail_t{std::move(f), attempts};
        }

    template<class CB, class ErrorHandler>
        struct retry_on_fail_err_handle_t: retry_on_fail_t<CB>
    {
        retry_on_fail_err_handle_t(CB &&f, int attempts, ErrorHandler &&e):
            retry_on_fail_t<CB>{std::move(f), attempts},
            err{std::move(e)}
        {}
        ErrorHandler err;
    };
    template<class CB, class ErrorHandler>
        auto retry_on_fail(int attempts, CB &&f, ErrorHandler &&err)
        {
            return retry_on_fail_err_handle_t{std::move(f), attempts, std::move(err)};
        }


    template<class ExpVal, class ExpErr, class V>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, retry_on_fail_t<V> &&def)->std::expected<ExpVal, ExpErr>
        {
            using namespace internals;
            if (!e)
                return std::move(e);

            for(int i = 0; i < (def.attempts - 1); ++i)
            {
                if (auto te = invoke_continuation_lval(e, def); te)
                    return te;
            }
            return invoke_continuation_lval(e, def);
        }

    template<class ExpVal, class ExpErr, class V, class E>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, retry_on_fail_err_handle_t<V, E> &&def)->std::expected<ExpVal, ExpErr>
        {
            using namespace internals;
            if (!e)
                return std::move(e);

            constexpr auto arity = get_arity_of<E>();

            for(int i = 0; i < (def.attempts - 1); ++i)
            {
                if (auto te = invoke_continuation_lval(e, def); te)
                    return te;
                else
                { 
                    if constexpr (arity == 1)
                    {
                        if (!def.err(te.error()))
                            return te;//we stop iterating and return error
                    }else if constexpr (arity == 0)
                    {
                        if (!def.err())
                            return te;//we stop iterating and return error
                    }else
                        static_assert(arity == 1, "Error handler must accept either 1 parameter (error) or nothing");
                }
            }
            return invoke_continuation_lval(e, def);
        }
}
#endif
