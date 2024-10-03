#ifndef UART_PRIMITIVES_HPP_
#define UART_PRIMITIVES_HPP_
#include "uart.hpp"

namespace uart
{
    namespace primitives
    {
        inline auto flush_and_wait(Channel &c, duration_ms_t maxWait = kForever, const char *pCtx = "")
        {
            using ExpectedResult = Channel::ExpectedResult;
            if (auto r = c.Flush(); !r)
                return ExpectedResult(std::unexpected(r.error()));

            if (auto r = c.PeekByte(maxWait); !r)
                return ExpectedResult(std::unexpected(r.error()));
            return ExpectedResult(std::ref(c));
        }

        inline auto skip_bytes(Channel &c, size_t bytes, const char *pCtx = "")
        {
            using ExpectedResult = std::expected<Channel::Ref, ::Err>;
            uint8_t buf[16];
            while(bytes)
            {
                if (auto r = c.Read(buf, std::min(sizeof(buf), bytes)); !r)
                    return ExpectedResult(std::unexpected(r.error()));
                else
                    bytes -= r.value().v;
            }

            return ExpectedResult(std::ref(c));
        }

        inline auto match_bytes(Channel &c, std::span<const uint8_t> bytes, const char *pCtx = "")
        {
            using ExpectedResult = std::expected<Channel::Ref, ::Err>;
            for(size_t idx = 0, n = bytes.size(); idx < n; ++idx)
            {
                if (auto r = c.ReadByte(); !r)
                    return ExpectedResult(std::unexpected(r.error()));
                else if (bytes[idx] != r.value().v)
                    return ExpectedResult(std::unexpected(::Err{"match_bytes", ESP_OK}));
            }
            return ExpectedResult(std::ref(c));
        }

        template<class... Seqs>
        inline auto match_any_bytes(Channel &c, Seqs&&... bytes)
        {
            using MatchAnyResult = Channel::RetVal<int>;
            using ExpectedResult = std::expected<MatchAnyResult, Err>;

            std::span<uint8_t> sequences[sizeof...(Seqs)] = {bytes...};
            bool anyValidLeft = true;
            size_t idx = 0;
            while(anyValidLeft)
            {
                if (auto r = c.ReadByte(); !r)
                    return ExpectedResult(std::unexpected(r.error()));
                else
                {
                    uint8_t b = r.value().v;
                    anyValidLeft = false;
                    int match = -1;
                    for(auto &s : sequences)
                    {
                        ++match;
                        if (s.empty())
                            continue;

                        if ((s.size() <= idx) && s[idx] != b)
                            s = {};
                        else if (s.size() == (idx + 1)) //match
                            return ExpectedResult(MatchAnyResult{std::ref(c), match});
                        else
                            anyValidLeft = true;
                    }
                    ++idx;
                }
            }
            return ExpectedResult(std::unexpected(::Err{"match_any_bytes", ESP_OK}));
        }

        inline auto match_bytes(Channel &c, const uint8_t *pBytes, uint8_t terminator, const char *pCtx = "")
        {
            using ExpectedResult = std::expected<Channel::Ref, ::Err>;
            while(*pBytes != terminator)
            {
                if (auto r = c.ReadByte(); !r)
                    return ExpectedResult(std::unexpected(r.error()));
                else if (r.value().v != *pBytes)
                    return ExpectedResult(std::unexpected(::Err{"match_bytes", ESP_OK}));
                ++pBytes;
            }
            return ExpectedResult(std::ref(c));
        }

        inline auto match_bytes(Channel &c, const char *pStr, const char *pCtx = "")
        {
            return uart::primitives::match_bytes(c, (const uint8_t*)pStr, 0, pCtx);
        }

        template<size_t N>
        inline auto match_bytes(Channel &c, const char (&arr)[N], const char *pCtx = "")
        {
            return uart::primitives::match_bytes(c, (const uint8_t*)arr, 0, pCtx);
        }

        template<size_t N>
        inline auto match_bytes(Channel &c, const uint8_t (&arr)[N], const char *pCtx = "")
        {
            return uart::primitives::match_bytes(c, std::span<const uint8_t>(arr, N), pCtx);
        }

        template<class... BytePtr>
        inline auto match_any_bytes_term(Channel &c, uint8_t term, BytePtr&&... bytes)
        {
            using MatchAnyResult = Channel::RetVal<int>;
            using ExpectedResult = std::expected<MatchAnyResult, Err>;
            const uint8_t* sequences[sizeof...(BytePtr)]={(const uint8_t*)bytes...};
            bool anyValidLeft = true;
            size_t idx = 0;

            while(anyValidLeft)
            {
                if (auto r = c.ReadByte(); !r)
                    return ExpectedResult(std::unexpected(r.error()));
                else
                {
                    uint8_t b = r.value().v;
                    anyValidLeft = false;
                    int match = -1;
                    for(auto &s : sequences)
                    {
                        ++match;
                        if (!s)
                            continue;

                        if (s[idx] == b)
                        {
                            if (s[idx + 1] == term)
                                return ExpectedResult(MatchAnyResult{std::ref(c), match});
                            anyValidLeft = true;
                        }else
                            s = nullptr;
                    }

                    ++idx;
                }
            }
            return ExpectedResult(std::unexpected(::Err{"match_any_bytes_term", ESP_OK}));
        }

        template<class C>
        concept convertible_to_const_char_ptr = requires(C &c) { {c}->std::convertible_to<const char*>; };

        template<class... BytePtr> requires (convertible_to_const_char_ptr<BytePtr> &&...)
        inline auto match_any_str(Channel &c, BytePtr&&... bytes)
        {
            return uart::primitives::match_any_bytes_term(c, 0, std::forward<BytePtr>(bytes)...);
        }

        inline auto read_until(Channel &c, uint8_t until, duration_ms_t maxWait = kForever, const char *pCtx = "")
        {
            using ExpectedResult = std::expected<Channel::Ref, ::Err>;
            using clock_t = std::chrono::system_clock;
            using time_point_t = std::chrono::time_point<clock_t>;
            time_point_t start = clock_t::now();

            while((maxWait != kForever) 
                    && std::chrono::duration_cast<std::chrono::milliseconds>(clock_t::now() - start) >= maxWait)
            {
                if (auto r = c.PeekByte(); !r)
                    return ExpectedResult(std::unexpected(r.error()));
                else if (r.value().v == until)
                    return ExpectedResult(std::ref(c));

                if (auto r = c.ReadByte(); !r)
                    return ExpectedResult(std::unexpected(r.error()));
            }
            return ExpectedResult(std::unexpected(::Err{"read_until timeout", ESP_OK}));
        }

        template<class T>
        inline auto read_into(Channel &c, T &dst, const char *pCtx = "")
        {
            using ExpectedResult = std::expected<Channel::Ref, ::Err>;
            if (auto r = c.Read((uint8_t*)&dst, sizeof(T)); !r)
                return ExpectedResult(std::unexpected(r.error()));
            else if (r.value().v != sizeof(T))
                return ExpectedResult(std::unexpected(::Err{"read_into wrong len", ESP_OK}));

            return ExpectedResult(std::ref(c));
        }

        inline auto read_into_bytes(Channel &c, uint8_t *pDst, int l, const char *pCtx = "")
        {
            using ExpectedResult = std::expected<Channel::Ref, ::Err>;
            if (auto r = c.Read(pDst, l); !r)
                return ExpectedResult(std::unexpected(r.error()));
            else if (r.value().v != l)
                return ExpectedResult(std::unexpected(::Err{"read_into_bytes wrong len", ESP_OK}));

            return ExpectedResult(std::ref(c));
        }

        template<class... Args>
        inline auto write_any(Channel &c, Args&&... args)
        {
            using ExpectedResult = std::expected<Channel::Ref, ::Err>;
            ExpectedResult r{std::ref(c)};
            (void)((bool)(r = c.Send((uint8_t const*)&args, sizeof(args))) ||...);
            return r;
        }

        template<class T>
        struct match_t
        {
            using functional_read_helper = void;
            const T v;
            const char *pCtx = "";

            static constexpr size_t size() { return sizeof(std::remove_cvref_t<T>); }
            auto run(Channel &c) { return uart::primitives::match_bytes(c, std::span<const uint8_t>((uint8_t const*)&v, sizeof(T)), pCtx); }
        };

        template<size_t N>
        struct skip_t
        {
            using functional_read_helper = void;
            static constexpr size_t size() { return N; }
            auto run(Channel &c) { return uart::primitives::skip_bytes(c, N); }
        };

        template<class CB>
        struct callback_t
        {
            using functional_read_helper = void;
            static constexpr size_t size() { return 0; }

            CB v;
            auto run(Channel &c) { return v(); }
        };

        template<class T, class D>
        struct read_var_t
        {
            using functional_read_helper = void;
            const T &len;
            D *v;

            static constexpr size_t size() { return 0; }
            size_t rt_size() const { return len; }
            auto run(Channel &c) { 
                if (auto r = c.Read((uint8_t*)v, len); !r)
                    return Channel::ExpectedResult(std::unexpected(r.error()));
                return Channel::ExpectedResult(std::ref(c));
            } };

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
                return uart::primitives::uart_sizeof<T>();
            }
        }

        template<class T>
        inline auto recv_for(Channel &c, T &&a)
        {
            using PureT = std::remove_cvref_t<T>;
            if constexpr (is_functional_read_helper<PureT>)
                return a.run(c);
            else
                return uart::primitives::read_into(c, a);
        }

        template<class Sz, class T>
        inline auto recv_for_checked(Channel &c, Sz &limit, T &&a)
        {
            auto sz = uart::primitives::uart_rtsize(a);
            if (limit < sz)
                return Channel::ExpectedResult(std::unexpected(::Err{"recv_for_checked: Insufficient length", ESP_OK})); 
            limit -= sz;

            using PureT = std::remove_cvref_t<T>;
            if constexpr (is_functional_read_helper<PureT>)
                return  a.run(c);
            else
                return uart::primitives::read_into(c, a);
        }

        template<class... Args>
        inline auto read_any(Channel &c, Args&&... args)
        {
            Channel::ExpectedResult r(std::ref(c));
            (void)((bool)(r = uart::primitives::recv_for(c, args)) || ...);
            return r;
        }

        template<class Sz, class... Args>
        inline auto read_any_limited(Channel &c, Sz &limit, Args&&... args)
        {
            Channel::ExpectedResult r(std::ref(c));
            (void)((bool)(r = uart::primitives::recv_for_checked(c, limit, args)) || ...);
            return r;
        }
    }
}
#endif
