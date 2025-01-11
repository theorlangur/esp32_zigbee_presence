#include "lib_linked_list.hpp"

#include <cstddef>
#include <cstring>
#include <optional>
#include "lib_type_traits.hpp"

Node::~Node()
{
    RemoveFromList();
}

void Node::RemoveFromList()
{
    if (m_pList)
    {
        if (!m_pPrev)
            m_pList->m_pFirst = m_pNext;
        else
            m_pPrev->m_pNext = m_pNext;

        if (m_pNext)
            m_pNext->m_pPrev = m_pPrev;

        m_pList = nullptr;
        m_pPrev = nullptr;
        m_pNext = nullptr;
    }
}

void Node::AddToList(LinkedList &l)
{
    RemoveFromList();
    m_pList = &l;
    m_pNext = l.m_pFirst;
    if (l.m_pFirst)
        l.m_pFirst->m_pPrev = this;
    l.m_pFirst = this;
}

LinkedList& LinkedList::operator+=(Node &n)
{
    n.AddToList(*this);
    return *this;
}
