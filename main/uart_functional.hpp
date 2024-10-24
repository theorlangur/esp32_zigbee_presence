#ifndef UART_FUNCTIONAL_H_
#define UART_FUNCTIONAL_H_

#include "uart.hpp"
#include "functional/functional.hpp"

namespace uart
{
    template<class Adaptor = functional::dummy_t>
    inline auto flush_and_wait(Channel &c, duration_ms_t maxWait = kForever, Adaptor adapt = {}, const char *pCtx = "")
    {
        using namespace functional;
        return and_then([&,adapt]{ return c.Flush() | adapt; }, pCtx)
             | and_then([&,maxWait,adapt]{ return c.PeekByte(maxWait) | adapt; }, pCtx);
    }

    inline auto skip_bytes(Channel &c, size_t bytes, const char *pCtx = "")
    {
        using namespace functional;
        struct ctx_t
        {
            Channel &c;
            size_t bytes;
            uint8_t buf[16];
        };
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[](ctx_t &ctx)->ExpectedCondition{ return ctx.bytes > 0; },
                /*iteration*/[](ctx_t &ctx)->ExpectedResult{
                                return ctx.c.Read(ctx.buf, std::min(sizeof(ctx.buf), ctx.bytes)) 
                                | and_then([&](size_t l)->ExpectedResult{ ctx.bytes -= l; return std::ref(ctx.c); });
                            },
                /*default  */[](ctx_t &ctx)->ExpectedResult{ return std::ref(ctx.c); },
                /*context  */ctx_t{c, bytes, {}}
                    ,pCtx
                );
    }

    inline auto match_bytes(Channel &c, std::span<const uint8_t> bytes, const char *pCtx = "")
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
                /*default  */[](ctx_t &ctx, int i)->ExpectedResult{ if (i) return std::ref(ctx.c); else return std::unexpected(::Err{"match_bytes", ESP_OK}); },
                /*context  */ctx_t{c, bytes}
                    ,pCtx
                );
    }

    template<class... Seqs>
    inline auto match_any_bytes(Channel &c, Seqs&&... bytes)
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
                                        int match = -1;
                                        for(auto &s : ctx.sequences)
                                        {
                                            ++match;
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
                                        }

                                        ++ctx.idx;
                                        return MatchAnyResult{std::ref(ctx.c), ctx.match};
                                  });
                            },
                /*default  */[](ctx_t &ctx, int n)->ExpectedResult{ if (n) return MatchAnyResult{std::ref(ctx.c), ctx.match}; else return std::unexpected(::Err{"match_any_bytes", ESP_OK}); },
                /*context  */ctx_t{c, {bytes...}}
                        ,"match_any_bytes"
                );
    }

    inline auto match_bytes(Channel &c, const uint8_t *pBytes, uint8_t terminator, const char *pCtx = "")
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
                /*default  */[](ctx_t &ctx, int n)->ExpectedResult{ if (n) return std::ref(ctx.c); else return std::unexpected(::Err{"match_bytes", ESP_OK}); },
                /*context  */ctx_t{c, pBytes, terminator}
                        , pCtx
                );
    }

    inline auto match_bytes(Channel &c, const char *pStr, const char *pCtx = "")
    {
        return match_bytes(c, (const uint8_t*)pStr, 0, pCtx);
    }

    template<size_t N>
    inline auto match_bytes(Channel &c, const char (&arr)[N], const char *pCtx = "")
    {
        return match_bytes(c, (const uint8_t*)arr, 0, pCtx);
    }

    template<size_t N>
    inline auto match_bytes(Channel &c, const uint8_t (&arr)[N], const char *pCtx = "")
    {
        return match_bytes(c, std::span<const uint8_t>(arr, N), pCtx);
    }

    template<class... BytePtr>
    inline auto match_any_bytes_term(Channel &c, uint8_t term, BytePtr&&... bytes)
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
                                        int match = -1;
                                        for(auto &s : ctx.sequences)
                                        {
                                            ++match;
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
                                        }

                                        ++ctx.idx;
                                        return MatchAnyResult{std::ref(ctx.c), ctx.match};
                                  });
                            },
                /*default  */[](ctx_t &ctx, int n)->ExpectedResult{ if (n) return MatchAnyResult{std::ref(ctx.c), ctx.match}; else return std::unexpected(::Err{"match_any_bytes", ESP_OK}); },
                /*context  */ctx_t{c, term, {(const uint8_t*)bytes...}}
                    ,"match_any_bytes_term"
                );
    }

    template<class C>
    concept convertible_to_const_char_ptr = requires(C &c) { {c}->std::convertible_to<const char*>; };

    template<class... BytePtr> requires (convertible_to_const_char_ptr<BytePtr> &&...)
    inline auto match_any_str(Channel &c, BytePtr&&... bytes)
    {
        return match_any_bytes_term(c, 0, std::forward<BytePtr>(bytes)...);
    }

    inline auto read_until(Channel &c, uint8_t until, duration_ms_t maxWait = kForever, const char *pCtx = "")
    {
        using namespace functional;
        using clock_t = std::chrono::system_clock;
        using time_point_t = std::chrono::time_point<clock_t>;
        struct ctx_t
        {
            Channel &c;
            uint8_t until;
            time_point_t start;
        };
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        using ExpectedCondition = std::expected<bool, ::Err>;
        return repeat_while(
                /*condition*/[maxWait](ctx_t &ctx)->ExpectedCondition{ 
                        if (maxWait != kForever && std::chrono::duration_cast<std::chrono::milliseconds>(clock_t::now() - ctx.start) >= maxWait)
                            return std::unexpected(::Err{"read_until timeout", ESP_OK});
                        return ctx.c.PeekByte() | and_then([&](uint8_t b)->ExpectedCondition{ return b != ctx.until; }); 
                },
                /*iteration*/[maxWait](ctx_t &ctx)->ExpectedResult{ 
                return ctx.c.ReadByte() | and_then([&]()->ExpectedResult{ return std::ref(ctx.c); }); },
                /*default  */[](ctx_t &ctx)->ExpectedResult{ return std::ref(ctx.c); },
                /*context  */ctx_t{c, until, clock_t::now()}
                    , pCtx
                );
    }

    template<class T>
    inline auto read_into(Channel &c, T &dst, const char *pCtx = "")
    {
        using namespace functional;
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        return and_then([&]{ return c.Read((uint8_t*)&dst, sizeof(T)); }, pCtx)
            | and_then([&c]->ExpectedResult{ return std::ref(c); });
    }

    inline auto read_into_bytes(Channel &c, uint8_t *pDst, int l, const char *pCtx = "")
    {
        using namespace functional;
        using ExpectedResult = std::expected<Channel::Ref, ::Err>;
        return and_then([&c,pDst,l]{ return c.Read(pDst, l); }, pCtx)
            | and_then([&c]->ExpectedResult{ return std::ref(c); });
    }

    template<class... Args>
    inline auto write_any(Channel &c, Args&&... args)
    {
        using namespace functional;
        return (and_then([&]{ return c.Send((uint8_t const*)&args, sizeof(args)); }) |...);
    }

    template<class T>
    struct match_t
    {
        using functional_read_helper = void;
        const T v;
        const char *pCtx = "";

        static constexpr size_t size() { return sizeof(std::remove_cvref_t<T>); }
        auto run(Channel &c) { return match_bytes(c, std::span<const uint8_t>((uint8_t const*)&v, sizeof(T)), pCtx); }
    };

    template<size_t N>
    struct skip_t
    {
        using functional_read_helper = void;
        static constexpr size_t size() { return N; }
        auto run(Channel &c) { return skip_bytes(c, N); }
    };

    template<class CB>
    struct callback_t
    {
        using functional_read_helper = void;
        static constexpr size_t size() { return 0; }

        CB v;
        auto run(Channel &c) { 
            using namespace functional;
            return and_then([&]{ return v(); }); 
        }
    };

    template<class T, class D>
    struct read_var_t
    {
        using functional_read_helper = void;
        const T &len;
        D *v;

        static constexpr size_t size() { return 0; }
        size_t rt_size() const { return len; }
        auto run(Channel &c) { return functional::and_then([&]{ return c.Read((uint8_t*)v, len); }); } };

    template<class C>
    concept is_functional_read_helper = requires{ typename C::functional_read_helper; };

    template<class T>
    constexpr size_t uart_sizeof()
    {
        if constexpr (is_functional_read_helper<T>)
            return T::size();
        else
            return sizeof(T);
    }

    template<class T>
    constexpr size_t uart_rtsize(T &a)
    {
        if constexpr (requires{ a.rt_size(); })
            return a.rt_size();
        else
        {
            return uart_sizeof<T>();
        }
    }

    template<class T>
    inline auto recv_for(Channel &c, T &&a)
    {
        using PureT = std::remove_cvref_t<T>;
        if constexpr (is_functional_read_helper<PureT>)
            return a.run(c);
        else
            return read_into(c, a);
    }

    template<class Sz, class T>
    inline auto recv_for_checked(Channel &c, Sz &limit, T &&a)
    {
        using PureT = std::remove_cvref_t<T>;
        using namespace functional;
        auto check_limit = and_then([&]()->Channel::ExpectedResult{ 
                auto sz = uart_rtsize(a);
                if (limit < sz)
                    return std::unexpected(::Err{"Insufficient length", ESP_OK}); 
                limit -= sz;
                return std::ref(c);
            });

        if constexpr (is_functional_read_helper<PureT>)
            return  std::move(check_limit) | a.run(c);
        else
            return std::move(check_limit) | read_into(c, a);
    }

    template<class... Args>
    inline auto read_any(Channel &c, Args&&... args)
    {
        return (uart::recv_for(c, args) | ...);
    }

    template<class Sz, class... Args>
    inline auto read_any_limited(Channel &c, Sz &limit, Args&&... args)
    {
        return (uart::recv_for_checked(c, limit, args) | ...);
    }
}

#endif
