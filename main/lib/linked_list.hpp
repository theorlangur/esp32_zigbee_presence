#ifndef LINKED_LIST_HPP_
#define LINKED_LIST_HPP_

#include <type_traits>

struct LinkedList;
struct Node
{
    Node() = default;
    ~Node();

    void RemoveFromList();
    void AddToList(LinkedList &l);

    LinkedList *m_pList = nullptr;
    Node *m_pNext = nullptr;
    Node *m_pPrev = nullptr;
};

struct LinkedList
{
    template<class N = Node>
    struct Iterator
    {
        N *m_pThis = nullptr;
        N *m_pNext = nullptr;
        void operator++()
        {
            if (m_pThis)
            {
                m_pThis = m_pNext;
                if (m_pThis)
                    m_pNext = static_cast<N*>(m_pThis->m_pNext);
                else
                    m_pNext = nullptr;
            }
        }
        bool operator!=(Iterator const& rhs) const { return m_pThis != rhs.m_pThis; }
        auto operator*() const { return m_pThis; }
    };

    Iterator<> begin() { if (m_pFirst) return {m_pFirst, m_pFirst->m_pNext}; else return end(); }
    Iterator<> end() { return {}; }

    LinkedList& operator+=(Node &n);

    Node *m_pFirst = nullptr;
};

template<class NodeType>
struct LinkedListT: LinkedList
{
    Iterator<NodeType> begin() { if (m_pFirst) return {(NodeType*)m_pFirst, (NodeType*)m_pFirst->m_pNext}; else return end(); }
    Iterator<NodeType> end() { static_assert(std::is_base_of_v<Node, NodeType>, "NodeType must be derived from Node"); return {}; }

    LinkedListT& operator+=(NodeType &n) { LinkedList::operator+=(n); return *this; }
};

#endif
