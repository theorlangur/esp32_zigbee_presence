#ifndef FUNCTIONAL_REPEAT_WHILE_H_
#define FUNCTIONAL_REPEAT_WHILE_H_

#include "functional_base.hpp"

namespace functional
{

    namespace internals
    {
        template<class F, class Ctx>
            concept can_call_with_ctx = requires(F &&f, Ctx &ctx) { f(ctx); };
    }

    template<class While, class CB, class Default, class Ctx>
        struct repeat_while_t
        {
            using functional_block_t = void;
            using Callback = CB;
            While w;
            CB t;
            Default d;
            Ctx ctx;

            template<class ExpVal>
            auto operator()(ExpVal &&v)
            {
                using namespace internals;
                using ret_type_t = ret_type_continuation_lval_t<ExpVal, decltype(*this), decltype(ctx)>;

                alignas(ret_type_t) uint8_t resMem[sizeof(ret_type_t)];
                ret_type_t *pRes = nullptr;
                auto while_condition = [](auto &d){
                    if constexpr (can_call_with_ctx<While,Ctx>)
                        return d.w(d.ctx);
                    else
                        return d.w();
                };
                auto invoke_default = [](auto &d){
                    if constexpr (can_call_with_ctx<Default,Ctx>)
                        return d.d(d.ctx);
                    else
                        return d.d();
                };
                while(true)
                {
                    if (auto te = while_condition(*this); !te)
                        return ret_type_t(std::unexpected(te.error()));
                    else if (!te.value())
                        break;
                    if (auto te = invoke_continuation_lval(std::forward<ExpVal>(v), *this, ctx); !te)
                        return te;
                    else
                    {
                        if (!pRes) pRes = new (resMem) ret_type_t(std::move(te));
                        else *pRes = std::move(te);
                    }
                }
                if (!pRes)
                    return invoke_default(*this);

                return *pRes;
            }
        };

    template<class While, class CB, class Default, class LoopCtx = dummy_loop_ctx_t>
        auto repeat_while(While &&w, CB &&f, Default &&d, LoopCtx ctx={})
        {
            return repeat_while_t{std::move(w), std::move(f), std::move(d), std::move(ctx)};
        }


    template<class ExpVal, class ExpErr, class W, class V, class D, class Ctx>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, repeat_while_t<W,V,D,Ctx> &&def)
        {
            using namespace internals;
            using ret_type_t = ret_type_continuation_lval_t<ExpVal, decltype(def), decltype(def.ctx)>;
            using ret_err_type_t = typename ret_type_t::error_type;
            if (!e)
                return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));

            return def(e.value());
        }
}
#endif
