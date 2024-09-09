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
        };

    template<class T>
        auto and_then(T &&f)
        {
            return and_then_t{std::forward<T>(f)};
        }

    template<class ExpVal, class ExpErr, class AndThenV>
        auto invoke_continuation(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> &&cont)
        {
            using namespace internals;
            constexpr const size_t arity = get_arity_of<AndThenV>();
            if constexpr (can_skip_first_n_args<AndThenV, ExpVal, 1>())
            {
                //destruct into separate and skip 1st argument
                return [&]<std::size_t... idx>(std::index_sequence<idx...>){
                    using namespace std;
                    decltype(auto) v = std::move(e).value();
                    return cont.t(get<idx + 1>(v)...);
                }(std::make_index_sequence<arity>{});
            }else if constexpr (arity > 1)
            {
                static_assert(arity <= std::tuple_size_v<ExpVal>, "Too many arguments");
                //destruct into separate
                return [&]<std::size_t... idx>(std::index_sequence<idx...>){
                    using namespace std;
                    decltype(auto) v = std::move(e).value();
                    return cont.t(get<idx>(v)...);
                }(std::make_index_sequence<arity>{});
            }else if constexpr(arity == 1)
                return cont.t(std::move(e).value());
            else
                return cont.t();
        }

        template<class Exp, class Cont>
            using ret_type_continuation_t = decltype(invoke_continuation(std::declval<Exp>(), std::declval<Cont>()));


    template<class ExpVal, class ExpErr, class AndThenV>
        auto operator|(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> &&cont)
        {
            using namespace internals;
            using ret_type_t = ret_type_continuation_t<decltype(e), decltype(cont)>;
            using ret_err_type_t = typename ret_type_t::error_type;
            if (!e)
                return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));

            return invoke_continuation(std::move(e), std::move(cont));
        }
};
#endif
