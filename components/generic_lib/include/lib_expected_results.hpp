#ifndef LIB_EXPECTED_RESULTS_HPP_
#define LIB_EXPECTED_RESULTS_HPP_

#include "esp_err.h"
#include <chrono>
#include "lib_formatter.hpp"

#define CHECK_STACK(sz) /*\
    {\
      auto t = xTaskGetCurrentTaskHandleForCore(0);\
      auto stackMark = uxTaskGetStackHighWaterMark(t);\
      printf("(%s) stackMark: %d\n", pcTaskGetName(NULL), stackMark);\
      if (stackMark <= sz)\
        {\
          assert(stackMark > sz);\
        }\
    }*/

struct Err
{
    const char *pLocation = "";
    esp_err_t code = ESP_OK;
};

template<>
struct tools::formatter_t<Err>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, Err const& e)
    {
        return tools::format_to(std::forward<Dest>(dst), "::Err\\{at {}: {}}", e.pLocation, esp_err_to_name(e.code));
    }
};

template<class Ref, class Val>
struct RetValT
{
    Ref r;
    Val v;
};

namespace std{
    template<class Ref, class Val>
    struct tuple_size<RetValT<Ref,Val>>: std::integral_constant<std::size_t, 2> {};

    template<class Ref, class Val>
    struct tuple_element<0, RetValT<Ref,Val>>
    {
        using type = Ref;
    };

    template<class Ref, class Val>
    struct tuple_element<1, RetValT<Ref,Val>>
    {
        using type = Val;
    };

    template<std::size_t idx, class Ref, class Val>
    auto& get(RetValT<Ref,Val> &rv)
    {
        if constexpr (idx == 0)
            return rv.r;
        else if constexpr (idx == 1)
            return rv.v;
    }

    template<std::size_t idx, class Ref, class Val>
    auto& get(RetValT<Ref,Val> const& rv)
    {
        if constexpr (idx == 0)
            return rv.r;
        else if constexpr (idx == 1)
            return rv.v;
    }
};

#define CALL_ESP_EXPECTED(location, f) \
    if (auto err = f; err != ESP_OK) \
        return std::unexpected(Err{location, err})

#endif
