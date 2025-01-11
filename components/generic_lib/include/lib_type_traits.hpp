#ifndef LIB_TYPE_TRAITS_HPP_
#define LIB_TYPE_TRAITS_HPP_

#include <cstring>
#include <type_traits>
#include <expected>

template<class T>
using deref_t = std::remove_cvref_t<decltype(*std::declval<T>())>;

template<class T>
concept simple_destructible_t = std::is_trivially_destructible_v<T>;// || requires { typename T::can_relocate; };

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
constexpr bool is_expected_type_v = is_expected_type<std::remove_cvref_t<C>>::value;

#endif
