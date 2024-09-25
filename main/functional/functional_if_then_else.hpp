#ifndef FUNCTIONAL_IF_THEN_ELSE_H_
#define FUNCTIONAL_IF_THEN_ELSE_H_

#include "functional_base.hpp"

namespace functional
{
    namespace internals
    {
        struct dummy_else_t
        {
            using i_am_dummy = void;
        };
    };

    template<class If>
    struct WrapIf
    {
        using Callback = If;
        If &t;
    };

    template<class Else>
    struct WrapElse
    {
        using Callback = Else;
        Else &t;
    };

    template<class If, class Then, class Else>
    struct if_then_else_t
    {
        using functional_block_t = void;
        using Callback = Then;
        If _if;
        Then t;
        Else _else;
        const char *pContext = "";


        template<class ExpVal>
        auto operator()(ExpVal &&v) const
        {
            using namespace internals;
            using ret_type_if_t = std::remove_cvref_t<ret_type_continuation_lval_t<ExpVal, WrapIf<If>>>;
            using ret_type_then_t = std::remove_cvref_t<ret_type_continuation_lval_t<ExpVal, if_then_else_t<If,Then,Else>>>;

            WrapIf _wif{_if};
            bool condition;
            if constexpr (has_expected_interface<ret_type_if_t>)
            {
                static_assert(std::is_convertible_v<ret_type_if_t, ret_type_then_t>, "If and Then return types must be compatible");
                if (auto te = invoke_continuation_lval(std::forward<ExpVal>(v), _wif); !te)
                {
                    if constexpr (kPrintContextOnError)
                        printf("if_then_else(%s) returned an error\n", pContext);
                    return te;
                }
                else
                    condition = te.value();
            }else
                condition = invoke_continuation_lval(std::forward<ExpVal>(v), _wif);

            if (condition)
                return invoke_continuation_lval(std::forward<ExpVal>(v), *this);

            if constexpr (requires{ typename Else::i_am_dummy; })
            {
                if constexpr (has_expected_interface<ret_type_then_t> && supports_tuple_interface<std::remove_cvref_t<ExpVal>>)
                {
                    if constexpr (!supports_tuple_interface<typename ret_type_then_t::value_type>)
                        return ret_type_then_t(std::get<0>(std::forward<ExpVal>(v)));
                    else if constexpr (std::tuple_size_v<typename ret_type_then_t::value_type> == 1)
                        return ret_type_then_t(std::get<0>(std::forward<ExpVal>(v)));
                    else
                        return ret_type_then_t(std::forward<ExpVal>(v));
                }else
                    return ret_type_then_t(std::forward<ExpVal>(v));
            }
            else
            {
                using ret_type_else_t = std::remove_cvref_t<ret_type_continuation_lval_t<ExpVal, WrapElse<Else>>>;
                WrapElse _welse{_else};
                static_assert(std::is_convertible_v<ret_type_else_t, ret_type_then_t>, "If and Then return types must be compatible");
                return invoke_continuation_lval(std::forward<ExpVal>(v), _welse);
            }
        }
    };

    template<class If, class Then, class Else>
    auto if_then_else(If &&_if, Then &&_then, Else &&_else, const char *pCtx = "")
    {
        if constexpr (std::is_convertible_v<If,bool>)
            return if_then_else_t{[&]()->bool{ return _if; }, std::forward<Then>(_then), std::forward<Else>(_else), pCtx};
        else
            return if_then_else_t{std::forward<If>(_if), std::forward<Then>(_then), std::forward<Else>(_else), pCtx};
    }

    template<class If, class Then>
    auto if_then(If &&_if, Then &&_then, const char *pCtx = "")
    {
        if constexpr (std::is_convertible_v<If,bool>)
            return if_then_else_t{[&]()->bool{ return _if; }, std::forward<Then>(_then), internals::dummy_else_t{}, pCtx};
        else
            return if_then_else_t{std::forward<If>(_if), std::forward<Then>(_then), internals::dummy_else_t{}, pCtx};
    }

    template<class ExpVal, class ExpErr, class If, class Then, class Else>
    auto operator|(std::expected<ExpVal, ExpErr> &&e, if_then_else_t<If,Then,Else> const&cond)
    {
        using namespace internals;
        using ret_type_t = std::remove_cvref_t<decltype(cond(e.value()))>;
        if constexpr (!std::is_same_v<ret_type_t, void>)
        {
            using ret_err_type_t = typename ret_type_t::error_type;
            if (!e)
            {
                if constexpr (std::is_same_v<ret_type_t, std::expected<ExpVal,ExpErr>>)
                {
                    if constexpr (kPrintContextOnError)
                        printf("if_then_else(%s) passing an error\n", cond.pContext);
                    return std::move(e);
                }
                else
                {
                    if constexpr (kPrintContextOnError)
                        printf("if_then_else(%s) passing an error\n", cond.pContext);
                    return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));
                }
            }

            return cond(e.value());
        }else
        {
            if (!!e)
                cond(e.value());
            return e;
        }
    }
};
#endif
