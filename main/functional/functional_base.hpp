#ifndef FUNCTIONAL_BASE_H_
#define FUNCTIONAL_BASE_H_

#include <expected>
#include <utility>
#include <tuple>


namespace functional
{
    namespace internals{
        template <class T>
            struct wrapper {
                const T value;
                static const wrapper<T> report_if_you_see_a_link_error_with_this_object;
            };

        template <class T>
            consteval const T& get_fake_object() noexcept {
                return wrapper<T>::report_if_you_see_a_link_error_with_this_object.value;
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
                using CR = std::remove_cvref_t<C>;
                if constexpr (requires{ typename CR::Callback; })
                {
                    if constexpr (requires{ CR::my_arity; })
                        return CR::my_arity;
                    else
                        return get_arity_of<typename CR::Callback>();
                }
                else
                    return get_arity_of_impl<CR>();
            }


        template<class C>
            consteval auto get_return_type()
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

        template<class C>
        concept has_expected_interface = requires(C e)
        {
            e.error();
            e.value();
            {!e}->std::convertible_to<bool>;
        };


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
                std::tuple_size<V>::value;
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

        template<class ExpVal, class Cont, class... Args>
            auto invoke_continuation_lval(ExpVal &&v, Cont &cont, Args&&... args)
            {
                //using CB = typename Cont::Callback;
                constexpr const size_t arity = get_arity_of<Cont>();
                constexpr const size_t add_args_count = sizeof...(Args);
                //static_assert(arity >= add_args_count, "Callback must accept at least the amount of additional arguments");
                if constexpr ((arity > add_args_count) && (arity - add_args_count) > 1)
                {
                    static_assert(arity <= std::tuple_size_v<ExpVal>, "Too many arguments");
                    //destruct into separate
                    return [&]<std::size_t... idx>(std::index_sequence<idx...>){
                        using namespace std;
                        return cont.t(std::forward<Args>(args)..., get<idx>(v)...);
                    }(std::make_index_sequence<arity>{});
                }else if constexpr((arity > add_args_count) && (arity - add_args_count) == 1)
                    return cont.t(std::forward<Args>(args)..., std::forward<ExpVal>(v));
                else
                {
                    return [&]<std::size_t... idx>(std::index_sequence<idx...>, std::tuple<Args&&...> &&targs){
                        using namespace std;
                        return cont.t(get<idx>(targs)...);
                    }(std::make_index_sequence<arity>{}, std::forward_as_tuple(std::forward<Args>(args)...));
                }
            }

        template<class Val, class Cont, class... Args>
            using ret_type_continuation_lval_t = decltype(invoke_continuation_lval(std::declval<Val&>(), std::declval<Cont&>(), std::declval<Args>()...));

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

    struct dummy_t 
    {
            using functional_block_t = void;
    };
    template<class ExpVal, class ExpErr>
    auto operator|(std::expected<ExpVal, ExpErr> &&e, dummy_t cont)
    {
        return e;
    }
}

#endif
