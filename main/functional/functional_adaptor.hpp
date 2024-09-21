#ifndef FUNCTIONAL_ADAPTOR_H_
#define FUNCTIONAL_ADAPTOR_H_

#include "functional_base.hpp"

namespace functional
{
    struct adapt_ident_t
    {
        auto operator()(auto &&a) const
        {
            return std::move(a);
        }
    };
    template<class NewExpected, class ConvertValue, class ConvertError>
    struct adapt_t
    {
        using functional_block_t = void;
        ConvertValue v;
        ConvertError e;

        template<class ExpVal>
        NewExpected operator()(ExpVal &&v) const
        {
            if constexpr (internals::is_expected_type_v<std::remove_cvref_t<ExpVal>>)
            {
                if (!!v)
                {
                    //printf("Adapting a value\n");
                    return this->v(v.value());
                }
                else
                {
                    //printf("Adapting an error\n");
                    return std::unexpected(this->e(v.error()));
                }
            }
            else
                return this->v(v);
        }
    };

    template<class NewExpected, class ConvertValue, class ConvertError>
    auto adapt_to(ConvertValue v, ConvertError e)
    {
        return adapt_t<NewExpected,ConvertValue,ConvertError>{std::move(v), std::move(e)};
    }

    template<class NewExpected, class ConvertValue>
    auto adapt_to_value(ConvertValue v)
    {
        return adapt_t<NewExpected, ConvertValue ,adapt_ident_t>{std::move(v), adapt_ident_t{}};
    }

    template<class NewExpected, class ConvertError>
    auto adapt_to_error(ConvertError e)
    {
        return adapt_t<NewExpected, adapt_ident_t, ConvertError>{adapt_ident_t{}, std::move(e)};
    }

    template<class ExpVal, class ExpErr, class NewExpected, class ConvertValue, class ConvertError>
    NewExpected operator|(std::expected<ExpVal, ExpErr> &&e, adapt_t<NewExpected, ConvertValue, ConvertError> const& cont)
    {
        return cont(e);
    }
}
#endif
