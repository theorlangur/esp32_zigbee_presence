#ifndef LIB_OBJECT_POOL_HPP_
#define LIB_OBJECT_POOL_HPP_

#include <bitset>
#include "lib_type_traits.hpp"

template<class T, size_t N>
class ObjectPool
{
public:
    template<ObjectPool<T,N> &staticPool>
    class Ptr
    {
    public:
        using can_relocate = void;

        Ptr() = default;
        Ptr(T *pPtr): m_pPtr(pPtr) {}
        template<class... Args>// requires (!(std::is_same_v<std::remove_cvref_t<Args>, T*>||...))
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

        void reset(T *pNew = nullptr) { staticPool.Release(m_pPtr); m_pPtr = pNew; }
        T* release() { auto *pRes = m_pPtr; m_pPtr = nullptr; return pRes; }

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
