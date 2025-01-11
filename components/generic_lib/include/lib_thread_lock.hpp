#ifndef LIB_THREAD_LOCK_HPP_
#define LIB_THREAD_LOCK_HPP_

#include <mutex>
#include "spinlock.h"

namespace thread{
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

}
#endif
