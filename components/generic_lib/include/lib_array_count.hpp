#ifndef LIB_ARRAY_COUNT_HPP_
#define LIB_ARRAY_COUNT_HPP_

#include <cstddef>
#include <cstring>
#include <optional>
#include "lib_type_traits.hpp"

template<class T, size_t N>
class ArrayCount
{
public:
    using naked_t = std::remove_pointer_t<std::remove_cvref_t<T>>;
    using ref_t = std::reference_wrapper<T>;
    using iterator_t = T*;
    using const_iterator_t = const T*;

    ArrayCount():m_Size(0) { }
    ArrayCount(const ArrayCount&) = delete;
    ArrayCount(ArrayCount&&) = delete;
    ~ArrayCount() { clear(); }

    ArrayCount& operator=(const ArrayCount&) = delete;
    ArrayCount& operator=(ArrayCount&&) = delete;

    size_t size() const { return m_Size; }

    iterator_t begin() { return m_Data; }
    iterator_t end() { return m_Data + m_Size; }

    const_iterator_t begin() const { return m_Data; }
    const_iterator_t end() const { return m_Data + m_Size; }

    T& operator[](size_t i) { return m_Data[i]; }
    const T& operator[](size_t i) const { return m_Data[i]; }
    
    void clear()
    {
        if constexpr (!simple_destructible_t<T>)
        {
            for(auto i = begin(), e = end(); i != e; ++i)
                i->~T();
        }
        m_Size = 0;
    }

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
        {
            if ((i->*pMem) == r)
                return i;
        }
        return end();
    }

    template<class X, class M> requires requires(T& t) { *t; }//supports deref
    iterator_t find(X const& r, M deref_t<T>::*pMem)
    {
        for(auto i = begin(), e = end(); i != e; ++i)
        {
            if (((**i).*pMem) == r)
                return i;
        }
        return end();
    }

    std::optional<ref_t> push_back(const T &v) 
    {
        if (m_Size >= N)
            return std::nullopt;
        T *pRef = new (&m_Data[m_Size++]) T(v);
        return *pRef;
    }

    std::optional<ref_t> push_back(T &&v) 
    {
        if (m_Size >= N)
            return std::nullopt;
        T *pRef = new (&m_Data[m_Size++]) T(std::move(v));
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
        if (i >= end()) return;

        if constexpr (!simple_destructible_t<T>)
            i->~T();

        if ((i + 1) == end())
        {
            if constexpr (!simple_destructible_t<T>)
                (end()-1)->~T();
            --m_Size;
            return;
        }


        if constexpr (std::is_trivially_move_constructible_v<T> || requires{typename T::can_relocate;})
            std::memmove((void*)i, (void*)(i + 1), (end() - i) * sizeof(T));
        else
        {
            new (i) T(std::move(*(i + 1)));
            for(auto s = i + 1, e = end() - 1; s != e; ++s)
                *s = std::move(*(s + 1));

            if constexpr (!simple_destructible_t<T>)
                (end()-1)->~T();
        }
        --m_Size;
    }

private:
    union
    {
        T m_Data[N];
    };
    size_t m_Size = 0;
};

#endif
