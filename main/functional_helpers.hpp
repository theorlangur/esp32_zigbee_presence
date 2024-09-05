#ifndef FUNCTIONAL_HELPERS_HPP_
#define FUNCTIONAL_HELPERS_HPP_

#include <expected>
#include <utility>

template<class T>
struct and_then_t
{
    using Callback = T;
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
    using Callback = T;
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

template<class CB>
struct retry_on_fail_t
{
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

template<class CB>
struct repeat_n_t
{
    using Callback = CB;
    CB t;
    int n;
};
template<class CB>
auto repeat_n(int n, CB &&f)
{
    return repeat_n_t{std::move(f), n};
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

template<class ExpVal, class ExpErr, class Cont, class... Args>
auto invoke_continuation_lval(std::expected<ExpVal, ExpErr> &e, Cont &cont, Args&&... args)
{
    using CB = typename Cont::Callback;
    constexpr const size_t arity = get_arity<CB>::value;
    constexpr const size_t add_args_count = sizeof...(Args);
    static_assert(arity >= add_args_count, "Callback must accept at least the amount of additional arguments");
    if constexpr ((arity - add_args_count) > 1)
    {
        static_assert(arity <= std::tuple_size_v<ExpVal>, "Too many arguments");
        //destruct into separate
        return [&]<std::size_t... idx>(std::index_sequence<idx...>){
            using namespace std;
            decltype(auto) v = e.value();
            return cont.t(std::forward<Args>(args)..., get<idx>(v)...);
        }(std::make_index_sequence<arity>{});
    }else if constexpr((arity - add_args_count) == 1)
        return cont.t(std::forward<Args>(args)..., e.value());
    else
        return cont.t(std::forward<Args>(args)...);
}

template<class Exp, class Cont>
using ret_type_continuation_t = decltype(invoke_continuation(std::declval<Exp>(), std::declval<Cont>()));

template<class Exp, class Cont, class... Args>
using ret_type_continuation_lval_t = decltype(invoke_continuation_lval(std::declval<Exp&>(), std::declval<Cont&>(), std::declval<Args>()...));

template<class ExpVal, class ExpErr, class AndThenV>
auto operator|(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> &&cont)->ret_type_continuation_t<decltype(e), decltype(cont)>
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
        return std::move(e);
}

template<class ExpVal, class ExpErr, class V>
auto operator|(std::expected<ExpVal, ExpErr> &&e, retry_on_fail_t<V> &&def)->std::expected<ExpVal, ExpErr>
{
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
    if (!e)
        return std::move(e);

    for(int i = 0; i < (def.attempts - 1); ++i)
    {
        if (auto te = invoke_continuation_lval(e, def); te)
            return te;
        else
        { 
            if constexpr (get_arity<E>::value == 1)
            {
                if (!def.err(te.error()))
                    return te;//we stop iterating and return error
            }else if constexpr (get_arity<E>::value == 0)
            {
                if (!def.err())
                    return te;//we stop iterating and return error
            }else
                static_assert(get_arity<E>::value == 1, "Error handler must accept either 1 parameter (error) or nothing");
        }
    }
    return invoke_continuation_lval(e, def);
}

template<class ExpVal, class ExpErr, class V>
auto operator|(std::expected<ExpVal, ExpErr> &&e, repeat_n_t<V> &&def)->ret_type_continuation_lval_t<decltype(e), decltype(def), int>
{
    if (!e)
        return std::move(e);

    for(int i = 0; i < (def.n - 1); ++i)
    {
        if (auto te = invoke_continuation_lval(e, def, i); !te)
            return te;
    }
    return invoke_continuation_lval(e, def, def.n - 1);
}

#endif
