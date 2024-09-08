#ifndef FUNCTIONAL_HELPERS_HPP_
#define FUNCTIONAL_HELPERS_HPP_

#include <expected>
#include <utility>
#include <tuple>

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

template<class T>
struct transform_error_t
{
    using functional_block_t = void;
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
    using functional_block_t = void;
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
    using functional_block_t = void;
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

struct dummy_loop_ctx_t {};
template<class CB, class Ctx>
struct repeat_n_t
{
    using functional_block_t = void;
    using Callback = CB;
    CB t;
    int n;
    Ctx ctx;
};
template<class CB, class LoopCtx = dummy_loop_ctx_t>
auto repeat_n(int n, CB &&f, LoopCtx ctx={})
{
    return repeat_n_t{std::move(f), n, std::move(ctx)};
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
};
template<class While, class CB, class Default, class LoopCtx = dummy_loop_ctx_t>
auto repeat_while(While &&w, CB &&f, Default &&d, LoopCtx ctx={})
{
    return repeat_while_t{std::move(w), std::move(f), std::move(d), std::move(ctx)};
}

struct any_t
{
    any_t(std::size_t);
    template<class T>
    constexpr operator T&&() const noexcept;
    template<class T>
    constexpr operator T&() const noexcept;
    template<class T>
    constexpr operator const T&() const noexcept;
};

template <class T>
struct wrapper {
  const T value;
  static const wrapper<T> report_if_you_see_a_link_error_with_this_object;
};

template <class T>
consteval const T& get_fake_object() noexcept {
  return wrapper<T>::report_if_you_see_a_link_error_with_this_object.value;
}

template<std::size_t N, class C>
consteval bool has_arity_of(C const& f)
{
    return [&]<std::size_t... idx>(std::index_sequence<idx...>){
        return requires { f(any_t(idx)...); };
    }(std::make_index_sequence<N>());
}

template<class C, std::size_t N = 10>
constexpr std::size_t get_arity_of_impl()
{
    if constexpr (N == 0)
    {
        static_assert(requires{get_fake_object<C>()();}, "If this fails then either f is not invokable or arity bigger than max supported");
        return 0;
    }else if constexpr (has_arity_of<N>(get_fake_object<C>()))
        return N;
    else
        return get_arity_of_impl<C, N-1>();
}

template<class C>
constexpr std::size_t get_arity_of()
{
    return get_arity_of_impl<C>();
}


template<class C>
constexpr auto get_return_type()
{
   return []<std::size_t... idx>(std::index_sequence<idx...>)
    {
        return get_fake_object<C>()(any_t(idx)...);
    }(std::make_index_sequence<get_arity_of<C>()>());
}

template<class C>
struct return_type_of
{
    using type = decltype(get_return_type<C>());
};

template<class C>
using return_type_of_t = return_type_of<C>::type;


template<class CB, class V, size_t N, size_t...idx>
concept can_skip_first_n_args_c = requires(CB &cb, V &&v){
        cb(get<idx + N>(v)...);
};

template<class CB, class V, size_t N, size_t... idx>
consteval bool can_skip_first_n_args_impl(std::index_sequence<idx...>)
{
    return can_skip_first_n_args_c<CB, V, N, idx...>;
}

template<class V>
concept supports_tuple_interface = requires{
    std::tuple_size_v<V>;
};

template<class CB, class V, size_t N>
consteval bool can_skip_first_n_args()
{
    constexpr const size_t arity = get_arity_of<CB>();
    if constexpr (arity == 1)
        return can_skip_first_n_args_impl<CB, V, N>(std::make_index_sequence<arity>{});
    else if constexpr (arity > 1 && supports_tuple_interface<V>)
        if constexpr (std::tuple_size_v<V> > arity)
            return can_skip_first_n_args_impl<CB, V, N>(std::make_index_sequence<arity>{});
        else
            return false;
    else
        return false;
}

template<class ExpVal, class ExpErr, class AndThenV>
void check_skip(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> &&cont)
{
    static_assert(can_skip_first_n_args<AndThenV, ExpVal, 1>());
}


template<class ExpVal, class ExpErr, class AndThenV>
auto invoke_continuation(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> &&cont)
{
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

template<class ExpVal, class ExpErr, class Cont, class... Args>
auto invoke_continuation_lval(std::expected<ExpVal, ExpErr> &e, Cont &cont, Args&&... args)
{
    using CB = typename Cont::Callback;
    constexpr const size_t arity = get_arity_of<CB>();
    constexpr const size_t add_args_count = sizeof...(Args);
    //static_assert(arity >= add_args_count, "Callback must accept at least the amount of additional arguments");
    if constexpr ((arity > add_args_count) && (arity - add_args_count) > 1)
    {
        static_assert(arity <= std::tuple_size_v<ExpVal>, "Too many arguments");
        //destruct into separate
        return [&]<std::size_t... idx>(std::index_sequence<idx...>){
            using namespace std;
            decltype(auto) v = e.value();
            return cont.t(std::forward<Args>(args)..., get<idx>(v)...);
        }(std::make_index_sequence<arity>{});
    }else if constexpr((arity > add_args_count) && (arity - add_args_count) == 1)
        return cont.t(std::forward<Args>(args)..., e.value());
    else
    {
        return [&]<std::size_t... idx>(std::index_sequence<idx...>, std::tuple<Args&&...> &&targs){
            using namespace std;
            return cont.t(get<idx>(targs)...);
        }(std::make_index_sequence<arity>{}, std::forward_as_tuple(std::forward<Args>(args)...));
    }
}

template<class Exp, class Cont>
using ret_type_continuation_t = decltype(invoke_continuation(std::declval<Exp>(), std::declval<Cont>()));

template<class Exp, class Cont, class... Args>
using ret_type_continuation_lval_t = decltype(invoke_continuation_lval(std::declval<Exp&>(), std::declval<Cont&>(), std::declval<Args>()...));

template<class ExpVal, class ExpErr, class AndThenV>
auto operator|(std::expected<ExpVal, ExpErr> &&e, and_then_t<AndThenV> &&cont)
{
    using ret_type_t = ret_type_continuation_t<decltype(e), decltype(cont)>;
    using ret_err_type_t = typename ret_type_t::error_type;
    if (!e)
        return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));

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

    constexpr auto arity = get_arity_of<E>();

    for(int i = 0; i < (def.attempts - 1); ++i)
    {
        if (auto te = invoke_continuation_lval(e, def); te)
            return te;
        else
        { 
            if constexpr (arity == 1)
            {
                if (!def.err(te.error()))
                    return te;//we stop iterating and return error
            }else if constexpr (arity == 0)
            {
                if (!def.err())
                    return te;//we stop iterating and return error
            }else
                static_assert(arity == 1, "Error handler must accept either 1 parameter (error) or nothing");
        }
    }
    return invoke_continuation_lval(e, def);
}

template<class ExpVal, class ExpErr, class V, class Ctx>
auto operator|(std::expected<ExpVal, ExpErr> &&e, repeat_n_t<V, Ctx> &&def)
{
    using ret_type_t = ret_type_continuation_lval_t<decltype(e), decltype(def), int>;
    using ret_err_type_t = typename ret_type_t::error_type;
    if (!e)
        return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));

    for(int i = 0; i < (def.n - 1); ++i)
    {
        if (auto te = invoke_continuation_lval(e, def, i, def.ctx); !te)
            return te;
    }
    return invoke_continuation_lval(e, def, def.n - 1, def.ctx);
}

template<class F, class Ctx>
concept can_call_with_ctx = requires(F &&f, Ctx &ctx) { f(ctx); };

template<class ExpVal, class ExpErr, class W, class V, class D, class Ctx>
auto operator|(std::expected<ExpVal, ExpErr> &&e, repeat_while_t<W,V,D,Ctx> &&def)
{
    using ret_type_t = ret_type_continuation_lval_t<decltype(e), decltype(def), decltype(def.ctx)>;
    using ret_err_type_t = typename ret_type_t::error_type;
    if (!e)
        return ret_type_t(std::unexpected(ret_err_type_t{std::move(e).error()}));

    alignas(ret_type_t) uint8_t resMem[sizeof(ret_type_t)];
    ret_type_t *pRes = nullptr;
    auto while_condition = [](auto &d){
        if constexpr (can_call_with_ctx<W,Ctx>)
            return d.w(d.ctx);
        else
            return d.w();
    };
    auto invoke_default = [](auto &d){
        if constexpr (can_call_with_ctx<D,Ctx>)
            return d.d(d.ctx);
        else
            return d.d();
    };
    while(true)
    {
        if (auto te = while_condition(def); !te)
            return ret_type_t(std::unexpected(te.error()));
        else if (!te.value())
            break;
        if (auto te = invoke_continuation_lval(e, def, def.ctx); !te)
            return te;
        else
        {
            if (!pRes) pRes = new (resMem) ret_type_t(std::move(te));
            else *pRes = std::move(te);
        }
    }
    if (!pRes)
        return invoke_default(def);
    
    return *pRes;
}

#endif
