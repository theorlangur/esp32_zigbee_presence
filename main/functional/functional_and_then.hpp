#ifndef FUNCTIONAL_AND_THEN_H_
#define FUNCTIONAL_AND_THEN_H_

#include "functional_base.hpp"

namespace functional
{
    template<class T>
        struct and_then_t
        {
            using functional_block_t = void;
            using Callback = T;
            T t;

            template<class ExpVal>
            auto operator()(ExpVal &&v) const
            {
                using namespace internals;
                using PureVal = std::remove_cvref_t<ExpVal>;
                constexpr const std::size_t arity = get_arity_of<T>();
                if constexpr (can_skip_first_n_args<T, PureVal, 1>())
                {
                    //destruct into separate and skip 1st argument
                    return [&]<std::size_t... idx>(std::index_sequence<idx...>){
                        using namespace std;
                        return t(get<idx + 1>(std::forward<ExpVal>(v))...);
                    }(std::make_index_sequence<arity>{});
                }else if constexpr (arity > 1)
                {
                    static_assert(arity <= std::tuple_size_v<PureVal>, "Too many arguments");
                    //destruct into separate
                    return [&]<std::size_t... idx>(std::index_sequence<idx...>){
                        using namespace std;
                        return t(get<idx>(std::forward<ExpVal>(v))...);
                    }(std::make_index_sequence<arity>{});
                }else if constexpr(arity == 1)
                    return t(std::forward<ExpVal>(v));
                else
                    return t();
            }
        };

    template<class T>
        auto and_then(T &&f)
        {
            return and_then_t{std::forward<T>(f)};
        }

    template<class ExpVal, class ExpErr, class AndThenV>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> const& cont)
        {
            using namespace internals;
            using ret_type_t = std::remove_cvref_t<decltype(cont(e.value()))>;
            if constexpr (!std::is_same_v<ret_type_t, void>)
            {
                using ret_err_type_t = typename ret_type_t::error_type;
                if (!e)
                    return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));

                return cont(e.value());
            }else
            {
                if (e)
                    cont(e.value());
                return e;
            }
        }
};
#endif
