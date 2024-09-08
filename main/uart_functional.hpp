#ifndef UART_FUNCTIONAL_H_
#define UART_FUNCTIONAL_H_

#include "uart.hpp"
#include "functional_helpers.hpp"

namespace uart
{
    auto match_bytes(Channel &c, duration_ms_t wait, std::span<uint8_t> bytes)
    {
        struct ctx_t
        {
            Channel &c;
            std::span<uint8_t> bytes;
            size_t idx = 0;
        };
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[](ctx_t &ctx)->ExpectedCondition{ return ctx.idx < ctx.bytes.size(); },
                /*iteration*/[wait](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.ReadByte(wait) 
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
    auto match_any_bytes(Channel &c, duration_ms_t wait, Seqs&&... bytes)
    {
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
                /*iteration*/[wait](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.ReadByte(wait) 
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

    auto match_bytes(Channel &c, duration_ms_t wait, const uint8_t *pBytes, uint8_t terminator)
    {
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
                /*iteration*/[wait, pOrig=pBytes](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.ReadByte(wait) 
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

    auto match_bytes(Channel &c, duration_ms_t wait, const char *pStr)
    {
        return match_bytes(c, wait, (const uint8_t*)pStr, 0);
    }

    template<class... BytePtr>
    auto match_any_bytes_term(Channel &c, duration_ms_t wait, uint8_t term, BytePtr&&... bytes)
    {
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
                /*iteration*/[wait](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.ReadByte(wait) 
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

    auto read_until(Channel &c, uint8_t until, duration_ms_t wait)
    {
        struct ctx_t
        {
            Channel &c;
            uint8_t until;
        };
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[wait](ctx_t &ctx)->ExpectedCondition{ return ctx.c.PeekByte(wait) | and_then([&](uint8_t b)->ExpectedCondition{ return b != ctx.until; }); },
                /*iteration*/[wait](ctx_t &ctx)->ExpectedResult{ return ctx.c.ReadByte(wait) | and_then([&]()->ExpectedResult{ return std::ref(ctx.c); }); },
                /*default  */[](ctx_t &ctx)->ExpectedResult{ return std::ref(ctx.c); },
                /*context  */ctx_t{c, until}
                );
    }
}

#endif
