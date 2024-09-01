#ifndef GENERIC_HELPERS_HPP_
#define GENERIC_HELPERS_HPP_

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

#endif
