#ifndef FUNCTIONAL_HELPERS_HPP_
#define FUNCTIONAL_HELPERS_HPP_

#include <expected>
#include <utility>

template<class T>
struct and_then_t
{
    T t;
};

template<class T>
auto and_then(T &&f)
{
    return and_then_t{std::forward<T>(f)};
}

template<class T>
struct transform_error_t
{
    T t;
};

template<class T>
auto transform_error(T &&f)
{
    return transform_error_t{std::forward<T>(f)};
}

template<class V>
struct or_else_t
{
    V t;
};

template<class V>
auto or_else(V &&f)
{
    return or_else_t{std::move(f)};
}

template <typename T> struct get_arity : get_arity<decltype(&T::operator())> {};
template <typename R, typename... Args> struct get_arity<R(*)(Args...)> : std::integral_constant<unsigned, sizeof...(Args)> {};
// Possibly add specialization for variadic functions
// Member functions:
template <typename R, typename C, typename... Args> struct get_arity<R(C::*)(Args...)> : std::integral_constant<unsigned, sizeof...(Args)> {};
template <typename R, typename C, typename... Args> struct get_arity<R(C::*)(Args...) const> : std::integral_constant<unsigned, sizeof...(Args)> {};

template<class ExpVal, class ExpErr, class AndThenV>
auto invoke_continuation(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> &&cont)
{
    if constexpr (get_arity<AndThenV>::value > 1)
    {
        static_assert(get_arity<AndThenV>::value <= std::tuple_size_v<ExpVal>, "Too many arguments");
        //destruct into separate
        return [&]<std::size_t... idx>(std::index_sequence<idx...>){
            using namespace std;
            decltype(auto) v = std::move(e).value();
            return cont.t(get<idx>(v)...);
        }(std::make_index_sequence<get_arity<AndThenV>::value>{});
    }else if constexpr(get_arity<AndThenV>::value == 1)
        return cont.t(std::move(e).value());
    else
        return cont.t();
}

template<class ExpVal, class ExpErr, class AndThenV>
auto operator|(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> &&cont)->decltype(invoke_continuation(std::declval<decltype(e)>(), std::declval<decltype(cont)>()))
{
    if (!e)
        return std::unexpected(std::move(e).error());

    return invoke_continuation(std::move(e), std::move(cont));
}

template<class ExpVal, class ExpErr, class TransformErrV>
auto operator|(std::expected<ExpVal, ExpErr> &&e, transform_error_t<TransformErrV> &&tr_err)->std::expected<ExpVal, std::invoke_result_t<TransformErrV, ExpErr&>>
{
    if (!e)
        return std::unexpected(tr_err.t(e.error()));

    return e.value();
}

template<class ExpVal, class ExpErr, class V>
auto operator|(std::expected<ExpVal, ExpErr> &&e, or_else_t<V> &&def)->std::expected<ExpVal, ExpErr>
{
    if (!e)
        return std::move(def).t;
    else
        return e;
}

#endif
