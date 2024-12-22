#ifndef GENERIC_HELPERS_HPP_
#define GENERIC_HELPERS_HPP_

#include "esp_err.h"
#include <thread>
#include <chrono>
#include "formatter.h"
#include "spinlock.h"

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

using duration_ms_t = std::chrono::duration<int, std::milli>;
inline static constexpr const duration_ms_t kForever = duration_ms_t(-1);

template<class CB>
struct ScopeExit
{
    ScopeExit(CB &&cb):m_CB(std::move(cb)){}
    ~ScopeExit(){ m_CB(); }
private:
    CB m_CB;
};

class SpinLock
{
public:
    SpinLock()
    {
        spinlock_initialize(&m_Lock);
    }

    void lock() { spinlock_acquire(&m_Lock, SPINLOCK_WAIT_FOREVER); }
    void unlock() { spinlock_release(&m_Lock); }

private:
    spinlock_t m_Lock;
};

class ILockable
{
public:
    virtual ~ILockable() = default; 
    virtual void lock() = 0;
    virtual void unlock() = 0;
};

class NoLock: public ILockable
{
public:
    virtual void lock() override {}
    virtual void unlock() override {}
};

class StdMutexLock: public ILockable
{
public:
    virtual void lock() override { m_Lock.lock(); }
    virtual void unlock() override { m_Lock.unlock(); }
private:
    std::mutex m_Lock;
};

template<class L>
struct LockGuard
{
    LockGuard(L *pL):m_pLock(pL) { if (pL) pL->lock(); }
    ~LockGuard() { if (m_pLock) m_pLock->unlock(); }
private:
    L *m_pLock;
};

template<typename BaseType, class Tag>
struct StrongType
{
    StrongType(BaseType d): m_Data(d) {}
    BaseType data() const { return m_Data; }
private:
    BaseType m_Data;

    friend struct Comparable;
    friend struct Oderable;
};

struct NonCopyable
{
    NonCopyable(NonCopyable const& rhs) = delete;
    NonCopyable& operator=(NonCopyable const& rhs) const = delete;
};

struct NonMovable
{
    NonMovable(NonMovable && rhs) = delete;
    NonMovable& operator=(NonMovable && rhs) const = delete;
};

//struct Comparable
//{
//    template<class T> bool operator==(this T const& lhs, T const& rhs) { return lhs.m_Data == rhs.m_Data; }
//};
//
//struct Oderable: Comparable
//{
//    template<class T> bool operator<(this T const& lhs, T const& rhs) { return lhs.m_Data < rhs.m_Data; }
//    template<class T> bool operator>(this T const& lhs, T const& rhs) { return lhs.m_Data > rhs.m_Data; }
//    template<class T> bool operator<=(this T const& lhs, T const& rhs) { return lhs.m_Data <= rhs.m_Data; }
//    template<class T> bool operator>=(this T const& lhs, T const& rhs) { return lhs.m_Data >= rhs.m_Data; }
//};

template<typename BaseType, BaseType kInv>
struct WithInvalidState
{
    static constexpr BaseType kInvalidState = kInv;
};

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

template<class T, size_t N>
class ArrayCount
{
public:
    using ref_t = std::reference_wrapper<T>;
    using iterator_t = T*;
    using const_iterator_t = const T*;

    ArrayCount():m_Size(0)
    {
    }

    size_t size() const { return m_Size; }

    iterator_t begin() { return m_Data; }
    iterator_t end() { return m_Data + m_Size; }

    const_iterator_t begin() const { return m_Data; }
    const_iterator_t end() const { return m_Data + m_Size; }

    iterator_t find(T const& r)
    {
        for(auto i = begin(), e = end(); i != e; ++i)
            if (*i == r)
                return i;
        return end();
    }

    template<class X, class M>
    iterator_t find(X const& r, M T::*pMem)
    {
        for(auto i = begin(), e = end(); i != e; ++i)
            if ((i->*pMem) == r)
                return i;
        return end();
    }

    std::optional<ref_t> push_back(const T &v) 
    {
        if (m_Size >= N)
            return std::nullopt;
        T *pRef = new (&m_Data[m_Size++]) T(v);
        return *pRef;
    }

    template<class... X>
    std::optional<ref_t> emplace_back(X&&... args) 
    {
        if (m_Size >= N)
            return std::nullopt;
        T *pRef = new (&m_Data[m_Size++]) T{std::forward<X>(args)...};
        return *pRef;
    }

    void erase(iterator_t i)
    {
        if (i < end()) return;
        if constexpr (!std::is_trivially_destructible_v<T>)
            i->~T();

        if ((i + 1) == end())
        {
            --m_Size;
            return;
        }


        if constexpr (std::is_trivially_move_constructible_v<T>)
        {
            std::memmove(i, i + 1, (end() - i) * sizeof(T));
            --m_Size;
        }else
        {
            new (i) T(std::move(*(i + 1)));
            for(auto s = i + 1, e = end() - 1; s != e; ++s)
            {
                *s = std::move(*(s + 1));
            }
            if constexpr (!std::is_trivially_destructible_v<T>)
                (end()-1)->~T();
            --m_Size;
        }
    }

private:
    union
    {
        T m_Data[N];
    };
    size_t m_Size = 0;
};

#define CALL_ESP_EXPECTED(location, f) \
    if (auto err = f; err != ESP_OK) \
        return std::unexpected(Err{location, err})

#ifdef NDEBUG
#define FMT_PRINT(fmt,...) {}
#else
#define FMT_PRINT(fmt,...) { char buf[256]; tools::format_to(tools::BufferFormatter(buf), fmt __VA_OPT__(,) __VA_ARGS__); printf("%s", buf); }
#endif

#endif
