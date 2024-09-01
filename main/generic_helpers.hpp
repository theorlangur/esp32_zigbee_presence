#ifndef GENERIC_HELPERS_HPP_
#define GENERIC_HELPERS_HPP_

#include "esp_err.h"
#include <thread>

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
