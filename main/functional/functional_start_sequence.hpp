#ifndef FUNCTIONAL_START_SEQUENCE_H_
#define FUNCTIONAL_START_SEQUENCE_H_

#include "functional_base.hpp"
namespace functional
{
    template<class... Args>
        struct start_t
        {
            std::tuple<Args...> args;
        };

    template<class... Args>
        auto start_sequence(Args&&... args) 
        { 
            return start_t{std::forward_as_tuple(std::forward<Args>(args)...)}; 
        }

    template<class Block, class... Args>
        auto operator|(start_t<Args...> &&s, Block &&def)
        {
            //actually call the thing
            return [&]<std::size_t...idx>(std::index_sequence<idx...>){ 
                return def(std::move(s.args)); 
            }(std::make_index_sequence<sizeof...(Args)>());
        }
}

#endif
