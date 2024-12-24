#ifndef OBJECT_POOL_HPP_
#define OBJECT_POOL_HPP_

#include <bitset>
#include "type_traits.hpp"

template<class T, size_t N>
class ObjectPool
{
public:
    template<ObjectPool<T,N> &staticPool>
    class Ptr
    {
    public:
        using can_relocate = void;

        template<class... Args>
        Ptr(Args&&... args):m_pPtr(staticPool.Acquire(std::forward<Args>(args)...)) { }
        ~Ptr() { staticPool.Release(m_pPtr); }

        Ptr(const Ptr&) = delete;//no copy
        Ptr& operator=(const Ptr &rhs) = delete;

        //move ok
        Ptr(Ptr &&rhs): m_pPtr(rhs.m_pPtr) { rhs.m_pPtr = nullptr; }
        Ptr& operator=(Ptr &&rhs)
        { 
            m_pPtr = rhs.m_pPtr;
            rhs.m_pPtr = nullptr; 
            return *this;
        }

        auto operator->() const { return m_pPtr; }
        operator T*() const { return m_pPtr; }
        T& operator *() const { return *m_pPtr; }
    private:
        T *m_pPtr = nullptr;
    };

    ObjectPool()
    {
        for(size_t i = 0; i < N; ++i) m_Data[i].m_NextFree = i + 1;
    }

    bool IsValid(T *pPtr) const
    {
        if (pPtr)
        {
            assert(((Elem*)pPtr >= m_Data) && ((Elem*)pPtr < (m_Data + N)));
            size_t i = (Elem*)pPtr - m_Data;
            return m_Allocated.test(i);
        }
        return false;
    }

    template<class... Args>
    T* Acquire(Args&&... args)
    {
        if (m_FirstFree >= N) return nullptr;
        auto i = m_FirstFree;
        m_FirstFree = m_Data[m_FirstFree].m_NextFree;
        m_Allocated.set(i);
        return new (&m_Data[i].m_Object) T{std::forward<Args>(args)...};
    }

    void Release(T* pPtr)
    {
        if (pPtr)
        {
            assert(((Elem*)pPtr >= m_Data) && ((Elem*)pPtr < (m_Data + N)));
            if constexpr (!simple_destructible_t<T>)
                pPtr->~T();
            size_t i = (Elem*)pPtr - m_Data;
            m_Data[i].m_NextFree = m_FirstFree;
            m_FirstFree = i;
            m_Allocated.reset(m_FirstFree);
        }
    }
private:
    std::bitset<N> m_Allocated;
    size_t m_FirstFree = 0;
    union Elem
    {
        Elem():m_NextFree(0){}
        ~Elem(){}
        size_t m_NextFree;
        T m_Object;
    };
    Elem m_Data[N];
};

#endif
