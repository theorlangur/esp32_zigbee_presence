#ifndef FUNCTIONAL_IF_THEN_ELSE_H_
#define FUNCTIONAL_IF_THEN_ELSE_H_

#include "functional_base.hpp"

namespace functional
{
    namespace internals
    {
        template<class C>
        concept has_expected_interface = requires(C e)
        {
            e.error();
            e.value();
            {e}->std::convertible_to<bool>;
        };

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
        using Callback = WrapElse;
        WrapElse &t;
    };

    template<class If, class Then, class Else>
    struct if_then_else_t
    {
        using functional_block_t = void;
        using Callback = Then;
        If _if;
        Then t;
        Else _else;


        template<class ExpVal>
        auto operator()(ExpVal &&v)
        {
            using namespace internals;
            using ret_type_if_t = ret_type_continuation_lval_t<ExpVal, WrapIf<If>>;
            using ret_type_then_t = ret_type_continuation_lval_t<ExpVal, if_then_else_t<If,Then,Else>>;

            WrapIf _wif{_if};
            bool condition;
            if constexpr (has_expected_interface<ret_type_if_t>)
            {
                static_assert(std::is_convertible_v<ret_type_if_t, ret_type_then_t>, "If and Then return types must be compatible");
                if (auto te = invoke_continuation_lval(std::forward<ExpVal>(v), _wif); !te)
                    return te;
                else
                    condition = te.value();
            }else
                condition = invoke_continuation_lval(std::forward<ExpVal>(v), _wif);

            if (condition)
                return invoke_continuation_lval(std::forward<ExpVal>(v), *this);

            if constexpr (requires{ typename Else::i_am_dummy; })
                return ret_type_then_t(std::forward<ExpVal>(v));
            else
            {
                using ret_type_else_t = ret_type_continuation_lval_t<ExpVal, WrapElse<Else>>;
                WrapElse _welse{_else};
                static_assert(std::is_convertible_v<ret_type_else_t, ret_type_then_t>, "If and Then return types must be compatible");
                return invoke_continuation_lval(std::forward<ExpVal>(v), _welse);
            }
        }
    };

    template<class If, class Then, class Else>
    auto if_then_else(If &&_if, Then &&_then, Else &&_else)
    {
        if constexpr (std::is_convertible_v<If,bool>)
            return if_then_else_t{[&]()->bool{ return _if; }, std::forward<Then>(_then), std::forward<Else>(_else)};
        else
            return if_then_else_t{std::forward<If>(_if), std::forward<Then>(_then), std::forward<Else>(_else)};
    }

    template<class If, class Then>
    auto if_then(If &&_if, Then &&_then)
    {
        if constexpr (std::is_convertible_v<If,bool>)
            return if_then_else_t{[&]()->bool{ return _if; }, std::forward<Then>(_then), internals::dummy_else_t{}};
        else
            return if_then_else_t{std::forward<If>(_if), std::forward<Then>(_then), internals::dummy_else_t{}};
    }

    template<class ExpVal, class ExpErr, class If, class Then, class Else>
    auto operator|(std::expected<ExpVal, ExpErr> &&e, if_then_else_t<If,Then,Else> &&cond)
    {
        using namespace internals;
        using ret_type_t = std::remove_cvref_t<decltype(cond(e.value()))>;
        if constexpr (!std::is_same_v<ret_type_t, void>)
        {
            using ret_err_type_t = typename ret_type_t::error_type;
            if (!e)
            {
                if constexpr (std::is_same_v<ret_type_t, std::expected<ExpVal,ExpErr>>)
                    return std::move(e);
                else
                    return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));
            }

            return cond(e.value());
        }else
        {
            if (e)
                cond(e.value());
            return e;
        }
    }
};
#endif
