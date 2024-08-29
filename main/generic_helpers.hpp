#ifndef GENERIC_HELPERS_HPP_
#define GENERIC_HELPERS_HPP_

template<typename BaseType, class Tag>
struct StrongType
{
    StrongType(BaseType d): m_Data(d) {}
    BaseType data() const { return m_Data; }
private:
    BaseType m_Data;

    friend class Comparable;
    friend class Oderable;
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

struct Comparable
{
    template<class T> bool operator==(this T const& lhs, T const& rhs) const { return lhs.m_Data == rhs.m_Data; }
};

struct Oderable: Comparable
{
    template<class T> bool operator<(this T const& lhs, T const& rhs) const { return lhs.m_Data < rhs.m_Data; }
    template<class T> bool operator>(this T const& lhs, T const& rhs) const { return lhs.m_Data > rhs.m_Data; }
    template<class T> bool operator<=(this T const& lhs, T const& rhs) const { return lhs.m_Data <= rhs.m_Data; }
    template<class T> bool operator>=(this T const& lhs, T const& rhs) const { return lhs.m_Data >= rhs.m_Data; }
};

template<typename BaseType, BaseType kInv>
struct WithInvalidState
{
    static constexpr BaseType kInvalidState = kInv;
};

#endif
