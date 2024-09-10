#ifndef UART_FUNCTIONAL_H_
#define UART_FUNCTIONAL_H_

#include "uart.hpp"
#include "functional/functional.hpp"

namespace uart
{
    auto match_bytes(Channel &c, std::span<const uint8_t> bytes)
    {
        using namespace functional;
        struct ctx_t
        {
            Channel &c;
            std::span<const uint8_t> bytes;
            size_t idx = 0;
        };
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[](ctx_t &ctx)->ExpectedCondition{ return ctx.idx < ctx.bytes.size(); },
                /*iteration*/[](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.ReadByte() 
                                | and_then([&](uint8_t b)->ExpectedResult{
                                        if (ctx.bytes[ctx.idx] != b)
                                        {
                                            //printf("Match failed to %s\n", pMatchStr);
                                            return std::unexpected(::Err{"match_bytes", ESP_OK});
                                        }
                                        ++ctx.idx;
                                        return std::ref(ctx.c);
                                  });
                            },
                /*default  */[]()->ExpectedResult{ return std::unexpected(::Err{"match_bytes", ESP_OK}); },
                /*context  */ctx_t{c, bytes}
                );
    }

    template<class... Seqs>
    auto match_any_bytes(Channel &c, Seqs&&... bytes)
    {
        using namespace functional;
        struct ctx_t
        {
            Channel &c;
            std::span<uint8_t> sequences[sizeof...(Seqs)];
            size_t idx = 0;
            int match = -1;
            bool anyValidLeft = true;
        };
        using MatchAnyResult = Channel::RetVal<int>;
        using ExpectedResult = std::expected<MatchAnyResult, Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[](ctx_t &ctx)->ExpectedCondition{ return ctx.anyValidLeft && ctx.match == -1; },
                /*iteration*/[](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.ReadByte() 
                                | and_then([&](uint8_t b)->ExpectedResult{
                                        ctx.anyValidLeft = false;
                                        int match = 0;
                                        for(auto &s : ctx.sequences)
                                        {
                                            if (s.empty())
                                                continue;

                                            if ((s.size() <= ctx.idx) && s[ctx.idx] != b)
                                            {
                                                s = {};
                                            }else if (s.size() == (ctx.idx - 1))
                                            {
                                                //match
                                                ctx.match = match;
                                            }else
                                                ctx.anyValidLeft = true;
                                            ++match;
                                        }

                                        ++ctx.idx;
                                        return MatchAnyResult{std::ref(ctx.c), ctx.match};
                                  });
                            },
                /*default  */[]()->ExpectedResult{ return std::unexpected(::Err{"match_any_bytes", ESP_OK}); },
                /*context  */ctx_t{c, {bytes...}}
                );
    }

    auto match_bytes(Channel &c, const uint8_t *pBytes, uint8_t terminator)
    {
        using namespace functional;
        //printf("match_bytes for: %s\n", pBytes);
        struct ctx_t
        {
            Channel &c;
            const uint8_t* pBytes;
            uint8_t terminator;
        };
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[](ctx_t &ctx)->ExpectedCondition{ return *ctx.pBytes != ctx.terminator; },
                /*iteration*/[](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.ReadByte() 
                                | and_then([&](uint8_t b)->ExpectedResult{
                                        if (*ctx.pBytes != b)
                                        {
                                            //printf("Match failed to %s\nByte num failed: %d\n", (const char*)pOrig, int(ctx.pBytes - pOrig));
                                            return std::unexpected(::Err{"match_bytes", ESP_OK});
                                        }
                                        ++ctx.pBytes;
                                        return std::ref(ctx.c);
                                  });
                            },
                /*default  */[]()->ExpectedResult{ return std::unexpected(::Err{"match_bytes", ESP_OK}); },
                /*context  */ctx_t{c, pBytes, terminator}
                );
    }

    auto match_bytes(Channel &c, const char *pStr)
    {
        return match_bytes(c, (const uint8_t*)pStr, 0);
    }

    template<size_t N>
    auto match_bytes(Channel &c, const char (&arr)[N])
    {
        return match_bytes(c, (const uint8_t*)arr, 0);
    }

    template<size_t N>
    auto match_bytes(Channel &c, const uint8_t (&arr)[N])
    {
        return match_bytes(c, std::span<const uint8_t>(arr, N));
    }

    template<class... BytePtr>
    auto match_any_bytes_term(Channel &c, uint8_t term, BytePtr&&... bytes)
    {
        using namespace functional;
        struct ctx_t
        {
            Channel &c;
            uint8_t term;
            const uint8_t* sequences[sizeof...(BytePtr)];
            size_t idx = 0;
            int match = -1;
            bool anyValidLeft = true;
        };
        using MatchAnyResult = Channel::RetVal<int>;
        using ExpectedResult = std::expected<MatchAnyResult, Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[](ctx_t &ctx)->ExpectedCondition{ return ctx.anyValidLeft && ctx.match == -1; },
                /*iteration*/[](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.ReadByte() 
                                | and_then([&](uint8_t b)->ExpectedResult{
                                        ctx.anyValidLeft = false;
                                        int match = 0;
                                        for(auto &s : ctx.sequences)
                                        {
                                            if (!s)
                                                continue;

                                            if (s[ctx.idx] == b)
                                            {
                                                if (s[ctx.idx + 1] == ctx.term)
                                                {
                                                    ctx.match = match;
                                                    break;
                                                }else
                                                    ctx.anyValidLeft = true;
                                            }else
                                                s = nullptr;
                                            ++match;
                                        }

                                        ++ctx.idx;
                                        return MatchAnyResult{std::ref(ctx.c), ctx.match};
                                  });
                            },
                /*default  */[]()->ExpectedResult{ return std::unexpected(::Err{"match_any_bytes", ESP_OK}); },
                /*context  */ctx_t{c, term, {(const uint8_t*)bytes...}}
                );
    }

    template<class C>
    concept convertible_to_const_char_ptr = requires(C &c) { {c}->std::convertible_to<const char*>; };

    template<class... BytePtr> requires (convertible_to_const_char_ptr<BytePtr> &&...)
    auto match_any_str(Channel &c, BytePtr&&... bytes)
    {
        return match_any_bytes_term(c, 0, std::forward<BytePtr>(bytes)...);
    }

    auto read_until(Channel &c, uint8_t until)
    {
        using namespace functional;
        struct ctx_t
        {
            Channel &c;
            uint8_t until;
        };
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[](ctx_t &ctx)->ExpectedCondition{ return ctx.c.PeekByte() | and_then([&](uint8_t b)->ExpectedCondition{ return b != ctx.until; }); },
                /*iteration*/[](ctx_t &ctx)->ExpectedResult{ return ctx.c.ReadByte() | and_then([&]()->ExpectedResult{ return std::ref(ctx.c); }); },
                /*default  */[](ctx_t &ctx)->ExpectedResult{ return std::ref(ctx.c); },
                /*context  */ctx_t{c, until}
                );
    }

    template<class T>
    auto read_into(Channel &c, T &dst)
    {
        using namespace functional;
        return and_then([&]{ return c.Read((uint8_t*)&dst, sizeof(T)); });
    }
}

#endif
