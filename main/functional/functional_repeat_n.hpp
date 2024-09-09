#ifndef FUNCTIONAL_REPEAT_N_H_
#define FUNCTIONAL_REPEAT_N_H_
#include "functional_base.hpp"

namespace functional
{
    struct dummy_loop_ctx_t {};
    template<class CB, class Ctx>
        struct repeat_n_t
        {
            using functional_block_t = void;
            using Callback = CB;
            CB t;
            int n;
            Ctx ctx;

            template<class ExpVal>
            auto operator()(ExpVal &&v)
            {
                using namespace internals;
                for(int i = 0; i < (n - 1); ++i)
                {
                    if (auto te = invoke_continuation_lval(std::forward<ExpVal>(v), *this, i, ctx); !te)
                        return te;
                }
                return invoke_continuation_lval(std::forward<ExpVal>(v), *this, n - 1, ctx);
            }
        };
    template<class CB, class LoopCtx = dummy_loop_ctx_t>
        auto repeat_n(int n, CB &&f, LoopCtx ctx={})
        {
            return repeat_n_t{std::move(f), n, std::move(ctx)};
        }


    template<class ExpVal, class ExpErr, class V, class Ctx>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, repeat_n_t<V, Ctx> &&def)
        {
            using namespace internals;
            using ret_type_t = ret_type_continuation_lval_t<ExpVal, decltype(def), int>;
            using ret_err_type_t = typename ret_type_t::error_type;
            if (!e)
                return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));

            return def(e.value());
        }
}
#endif
