#include "zb_binds.hpp"

namespace zb
{
    BindInfoPool g_BindInfoPool;
    void BindInfo::Do()
    {
        switch(m_State)
        {
            case State::New:
                {
                }
                break;
            case State::VerifyBinds:
                {
                }
                break;
            case State::SendBindToMeReq:
                {
                }
                break;
            case State::SendConfigureReport:
                {
                }
                break;
            case State::FailedToConfigure:
                {
                }
                break;
        }
    }

    void BindInfo::GetBindTable()
    {
    }
}
