#ifndef FUNCTIONAL_COMBO_H_
#define FUNCTIONAL_COMBO_H_

#include "functional_base.hpp"

namespace functional
{
    namespace internals
    {
        template<class C>
            concept is_functional_block = requires{ typename std::remove_cvref_t<C>::functional_block_t; };

        template<class C>
        struct is_expected_type
        {
            static constexpr const bool value = false;
        };

        template<class V, class E>
        struct is_expected_type<std::expected<V,E>>
        {
            static constexpr const bool value = true;
        };

        template<class C>
        constexpr bool is_expected_type_v = is_expected_type<C>::value;
    }

    template<class T>
        struct combo_t
        {
            using functional_block_t = void;
            using Callback = T;
            static const constexpr size_t my_arity = 1;
            T t;
            template<class ExpVal>
            auto operator()(ExpVal &&v) const
            {
                return t(std::forward<ExpVal>(v));
            }
        };

    template<class Block1, class Block2> requires internals::is_functional_block<Block1> && internals::is_functional_block<Block2>
        auto operator|(Block1 &&b1, Block2 &&b2)
        {
            return combo_t{[b1=std::move(b1), b2=std::move(b2)]<class E>(E &&e) {
                if constexpr (internals::is_expected_type_v<E>)
                    return std::move(e) | b1 | b2;
                else
                    return b1(std::forward<E>(e)) | b2;
            }
            };
        }

    template<class ExpVal, class ExpErr, class CB>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, combo_t<CB> const&def)
        {
            using namespace internals;
            using ret_type_t = ret_type_continuation_lval_t<decltype(e.value()), decltype(def), decltype(e)>;
            using ret_err_type_t = typename ret_type_t::error_type;
            if (!e)
                return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));
            return def(std::move(e));
        }
}
#endif
