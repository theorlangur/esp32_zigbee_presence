#ifndef FUNCTIONAL_REPEAT_WHILE_H_
#define FUNCTIONAL_REPEAT_WHILE_H_

#include "functional_base.hpp"

namespace functional
{

    namespace internals
    {
        template<class F, class Ctx>
            concept can_call_with_ctx = requires(F &&f, Ctx &ctx) { f(ctx); };

        template<class F, class Ctx>
            concept can_call_with_ctx_and_it = requires(F &&f, Ctx &ctx, int n) { f(ctx, n); };

        template<class F>
            concept can_call_with_it = requires(F &&f, int n) { f(n); };
    }

    template<class Err>
    using WhileResult = std::expected<bool, Err>;

    template<class While, class CB, class Default, class Ctx>
        struct repeat_while_t
        {
            using functional_block_t = void;
            using Callback = CB;
            While w;
            CB t;
            Default d;
            mutable Ctx ctx;
            const char *pContext = "";

            template<class ExpVal>
            auto operator()(ExpVal &&v) const
            {
                using namespace internals;
                using ret_type_t = ret_type_continuation_lval_t<ExpVal, decltype(*this), decltype(ctx)>;

                int iterations = 0;
                auto while_condition = [](auto &d){
                    if constexpr (can_call_with_ctx<While,Ctx>)
                        return d.w(d.ctx);
                    else
                        return d.w();
                };
                auto invoke_default = [&](auto &d){
                    if constexpr (can_call_with_ctx_and_it<Default,Ctx>)
                        return d.d(d.ctx, iterations);
                    else if constexpr (can_call_with_ctx<Default,Ctx>)
                        return d.d(d.ctx);
                    else if constexpr (can_call_with_it<Default>)
                        return d.d(iterations);
                    else
                        return d.d();
                };
                while(true)
                {
                    if (auto te = while_condition(*this); !te)
                    {
                        if constexpr (kPrintContextOnError)
                            printf("repeat_while(%s) condition returned an error\n", pContext);
                        return ret_type_t(std::unexpected(te.error()));
                    }
                    else if (!te.value())
                        break;
                    if (auto te = invoke_continuation_lval(std::forward<ExpVal>(v), *this, ctx); !te)
                    {
                        if constexpr (kPrintContextOnError)
                            printf("repeat_while(%s) iteration returned an error\n", pContext);
                        return te;
                    }else
                    {
                        ++iterations;
                    }
                }
                return invoke_default(*this);
            }
        };

    template<class While, class CB, class Default, class LoopCtx = dummy_loop_ctx_t>
        auto repeat_while(While &&w, CB &&f, Default &&d, LoopCtx ctx={}, const char *pCtx = "")
        {
            return repeat_while_t{std::move(w), std::move(f), std::move(d), std::move(ctx), pCtx};
        }


    template<class ExpVal, class ExpErr, class W, class V, class D, class Ctx>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, repeat_while_t<W,V,D,Ctx> const &def)
        {
            using namespace internals;
            using ret_type_t = ret_type_continuation_lval_t<ExpVal, decltype(def), decltype(def.ctx)>;
            using ret_err_type_t = typename ret_type_t::error_type;
            if (!e)
            {
                if constexpr (kPrintContextOnError)
                    printf("repeat_while(%s) passing an error\n", def.pContext);
                return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));
            }

            return def(e.value());
        }
}
#endif
