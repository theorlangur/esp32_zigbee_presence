#include <cstring>
#include "ld2420.hpp"
#include "generic_helpers.hpp"
#include "functional/functional.hpp"
#include "uart_functional.hpp"

const char* LD2420::err_to_str(ErrorCode e)
{
    switch(e)
    {
        case ErrorCode::Ok: return "Ok";
        case ErrorCode::Init: return "Init";
        case ErrorCode::SendFrame: return "SendFrame";
        case ErrorCode::SendFrame_Incomplete: return "SendFrame_Incomplete";
        case ErrorCode::SendCommand_InvalidResponse: return "SendCommand_InvalidResponse";
        case ErrorCode::SendCommand_FailedWrite: return "SendCommand_FailedWrite";
        case ErrorCode::SendCommand_FailedRead: return "SendCommand_FailedRead";
        case ErrorCode::SendCommand_WrongFormat: return "SendCommand_WrongFormat";
        case ErrorCode::SendCommand_Failed: return "SendCommand_Failed";
        case ErrorCode::SendCommand_InsufficientSpace: return "SendCommand_InsufficientSpace";
        case ErrorCode::RecvFrame_Malformed: return "RecvFrame_Malformed";
        case ErrorCode::RecvFrame_Incomplete: return "RecvFrame_Incomplete";
        case ErrorCode::SimpleData_Malformed: return "SimpleData_Malformed";
        case ErrorCode::EnergyData_Malformed: return "EnergyData_Malformed";
        case ErrorCode::SimpleData_Failure: return "SimpleData_Failure";
        case ErrorCode::EnergyData_Failure: return "EnergyData_Failure";
        case ErrorCode::FillBuffer_NoSpace: return "FillBuffer_NoSpace";
        case ErrorCode::FillBuffer_ReadFailure: return "FillBuffer_ReadFailure";
        case ErrorCode::MatchError: return "MatchError";
        case ErrorCode::RestartFailed: return "RestartFailed";
    }
    return "unknown";
}

LD2420::LD2420(uart::Port p, int baud_rate):
    uart::Channel(p, baud_rate)
{
    SetParity(uart::Parity::Disable);
    SetHWFlowControl(uart::HWFlowCtrl::Disable);
    SetQueueSize(10);
    SetRxBufferSize(1024);
    SetTxBufferSize(1024);
}

LD2420::ExpectedResult LD2420::Init(int txPin, int rxPin)
{
    using namespace functional;
    return Configure() 
        | and_then([&]{ return SetPins(txPin, rxPin); })
        | and_then([&]{ return Open(); })
        | AdaptToResult("LD2420::Init", ErrorCode::Init);
}

LD2420::ExpectedResult LD2420::ReloadConfig()
{
    using namespace functional;
    return OpenCommandMode()
        | and_then([&]{ return UpdateVersion(); })
        | and_then([&]{ return UpdateMinMaxTimeout(); })
        | repeat_n(16, [&](uint8_t i){ return UpdateGate(i); })
        | and_then([&]{ return CloseCommandMode(); })
        | transform_error([&](CmdErr e){ return e.e; });
}

LD2420::ExpectedResult LD2420::Restart()
{
    using namespace functional;
    return OpenCommandMode()
        | transform_error([&](CmdErr e){ return e.e; })
        | and_then([&]{ return SendFrameV2(Cmd::Restart); })
        | uart::flush_and_wait(*this, kRestartTimeout, AdaptToResult("LD2420::Restart", ErrorCode::RestartFailed))
        | if_then(//after restart the default mode 'Simple'. We might want to switch
          /*if*/    [&]{ return m_Mode != SystemMode::Simple; },
          /*then*/  [&]{ return ChangeConfiguration().SetSystemMode(m_Mode).EndChange(); }
                );
}

LD2420::ExpectedResult LD2420::FactoryReset()
{
    auto &ch = ChangeConfiguration()
                .SetTimeout(120)
                .SetMinDistanceRaw(1)
                .SetMaxDistanceRaw(12);
    constexpr uint16_t kMoveThreshold[] = {
//Gates:  0      1     2    3    4    5    6    7    8    9    10   11   12   13   14   15
        60000, 30000, 400, 300, 250, 250, 250, 250, 300, 250, 250, 250, 250, 200, 200, 200
    };
    constexpr uint16_t kStillThreshold[] = {
//Gates:  0      1     2    3    4    5    6    7    8    9    10   11   12   13   14   15
        40000, 20000, 200, 250, 150, 150, 150, 150, 150, 150, 150, 150, 100, 100, 100, 100
    };

    for(uint8_t g = 0; g < 16; ++g)
        ch.SetMoveThreshold(g, kMoveThreshold[g])
          .SetStillThreshold(g, kStillThreshold[g]);

    return ch.EndChange();
}

LD2420::ExpectedOpenCmdModeResult LD2420::OpenCommandMode()
{
    using namespace functional;
    uint16_t protocol_version = 2;
    OpenCmdModeResponse r;
    return SendFrameV2(Cmd::OpenCmd, protocol_version)
        | and_then([&]{ std::this_thread::sleep_for(duration_ms_t(100)); })
        | transform_error([](Err e){ return CmdErr{e, 0}; })
        | and_then([&]{ return SendCommandV2(Cmd::OpenCmd, to_send(protocol_version), to_recv(r.protocol_version, r.buffer_size)); })
        | and_then([&]()->ExpectedOpenCmdModeResult{ return OpenCmdModeRetVal{std::ref(*this), r}; });
}

LD2420::ExpectedGenericCmdResult LD2420::CloseCommandMode()
{
    using namespace functional;
    return SendCommandV2(Cmd::CloseCmd, to_send(), to_recv())
            | and_then([&]()->ExpectedGenericCmdResult{ return std::ref(*this); });
}

std::string_view LD2420::GetVersion() const
{
    return m_Version;
}

LD2420::ExpectedGenericCmdResult LD2420::SetSystemModeInternal(SystemMode mode)
{
    return SendCommandV2(Cmd::WriteSys, to_send(SysRegs::Mode, mode), to_recv());
}


LD2420::ExpectedGenericCmdResult LD2420::UpdateVersion()
{
    using namespace functional;
    uint16_t verLen;
    return SendCommandV2(Cmd::ReadVer, to_send(), to_recv(verLen, uart::read_var_t{verLen, m_Version})) 
            | and_then([&]{ m_Version[verLen] = 0; });
}

LD2420::ExpectedGenericCmdResult LD2420::UpdateMinMaxTimeout()
{
    using namespace functional;
    return SendCommandV2(Cmd::ReadADB, to_send(ADBRegs::MinDistance, ADBRegs::MaxDistance, ADBRegs::Timeout), to_recv(m_MinDistance, m_MaxDistance, m_Timeout)); 
}

LD2420::ExpectedGenericCmdResult LD2420::UpdateGate(uint8_t gate)
{
    using namespace functional;
    return SendCommandV2(Cmd::ReadADB
            , to_send(ADBRegs::MoveThresholdGateBase + gate, ADBRegs::StillThresholdGateBase + gate)
            , to_recv(m_Gates[gate].m_MoveThreshold, m_Gates[gate].m_StillThreshold)); 
}

LD2420::ExpectedResult LD2420::ReadSimpleFrame()
{
    using namespace functional;
    SetDefaultWait(duration_ms_t(150));
    uint16_t val = 0;
    auto ParseNum = repeat_while(
            [&]()->WhileResult<::Err>{ return PeekByte() | and_then([&](uint8_t b)->WhileResult<::Err>{ return b >= '0' && b <= '9'; }); },
            [&]{ return ReadByte() | and_then([&](uint8_t b)->Channel::ExpectedResult{ val = val * 10 + b - '0'; return std::ref((Channel&)*this);}); },
            [&]()->Channel::ExpectedResult{ return std::ref((Channel&)*this); }
            );

    return start_sequence(*this)
                    | uart::read_until(*this, 'O')
                    | uart::match_any_str(*this, "ON", "OFF")
                    | and_then([&](int match){ m_Presence.m_Detected = match == 0; })
                    | uart::match_bytes(*this, "\r\n")
                    | if_then(
                            /*if  */ m_Presence.m_Detected
                            /*then*/,   uart::match_bytes(*this, "Range ")
                                        | ParseNum
                                        | uart::match_bytes(*this, "\r\n")
                                        | and_then([&]{ m_Presence.m_Distance = float(val) / 100; })
                            )
                    | AdaptToResult("LD2420::ReadSimpleFrameV2", ErrorCode::SimpleData_Failure)
                    ;

}

LD2420::ExpectedResult LD2420::TryReadSimpleFrame(int attempts)
{
    using namespace functional;
    return ExpectedResult{std::ref(*this)} | retry_on_fail(attempts, [&]{ return ReadSimpleFrame(); });
}

LD2420::ExpectedResult LD2420::ReadEnergyFrame()
{
    using namespace functional;
    SetDefaultWait(duration_ms_t(150));
    constexpr uint8_t header[] = {0xf4, 0xf3, 0xf2, 0xf1};
    constexpr uint8_t footer[] = {0xf8, 0xf7, 0xf6, 0xf5};
    uint16_t reportLen = 0;
    uint16_t distance = 0;
    return start_sequence()
        | uart::read_until(*this, header[0])
        | uart::match_bytes(*this, header)
        | uart::read_into(*this, reportLen)
        | uart::read_into(*this, m_Presence.m_Detected)
        | uart::read_into(*this, distance)
        | and_then([&]{ m_Presence.m_Distance = float(distance) / 100; })
        | repeat_n(16, [&](int i){ return start_sequence() | uart::read_into(*this, m_Gates[i].m_Energy); })
        | uart::match_bytes(*this, footer)
        | AdaptToResult("LD2420::ReadEnergyFrame", ErrorCode::EnergyData_Failure)
        ;
}

LD2420::ExpectedResult LD2420::TryReadEnergyFrame(int attempts)
{
    using namespace functional;
    return ExpectedResult{std::ref(*this)} | retry_on_fail(attempts, [&]{ return ReadEnergyFrame(); });
}


/**********************************************************************/
/* ConfigBlock                                                        */
/**********************************************************************/
LD2420::ConfigBlock& LD2420::ConfigBlock::SetSystemMode(SystemMode mode)
{
    m_Changed.Mode = true;
    m_NewMode = mode;
    return *this;
}
LD2420::ConfigBlock& LD2420::ConfigBlock::SetMinDistance(int dist)
{
    m_Changed.MinDistance = true;
    m_NewMinDistance = std::clamp(dist * 10 / 7, 1, 12);
    return *this;
}
LD2420::ConfigBlock& LD2420::ConfigBlock::SetMinDistanceRaw(uint32_t dist)
{
    m_Changed.MinDistance = true;
    m_NewMinDistance = std::clamp(dist, uint32_t(1), uint32_t(12));
    return *this;
}
LD2420::ConfigBlock& LD2420::ConfigBlock::SetMaxDistance(int dist)
{
    m_Changed.MaxDistance = true;
    m_NewMaxDistance = std::clamp(dist * 10 / 7, 1, 12);
    return *this;
}

LD2420::ConfigBlock& LD2420::ConfigBlock::SetMaxDistanceRaw(uint32_t dist)
{
    m_Changed.MaxDistance = true;
    m_NewMaxDistance = std::clamp(dist, uint32_t(1), uint32_t(12));
    return *this;
}

LD2420::ConfigBlock& LD2420::ConfigBlock::SetTimeout(uint32_t t)
{
    m_Changed.Timeout = true;
    m_NewTimeout = t;
    return *this;
}

LD2420::ConfigBlock& LD2420::ConfigBlock::SetMoveThreshold(uint8_t gate, uint16_t energy)
{
    if (gate > 15)
        return *this;

    m_GateChanges |= uint32_t(1) << (gate * 2);
    m_NewGates[gate].move = energy;
    return *this;
}

LD2420::ConfigBlock& LD2420::ConfigBlock::SetStillThreshold(uint8_t gate, uint16_t energy)
{
    if (gate > 15)
        return *this;

    m_GateChanges |= uint32_t(1) << (gate * 2 + 1);
    m_NewGates[gate].still = energy;
    return *this;
}

LD2420::ExpectedResult LD2420::ConfigBlock::EndChange()
{
    using namespace functional;
    if (!m_GateChanges && !m_MiscChanges)
        return std::ref(d);
    ScopeExit clearChanges = [&]{ m_GateChanges = m_MiscChanges = 0; };
    return d.OpenCommandMode()
        | if_then([&]()->bool{ return m_Changed.Mode; }
                    , [&]{ d.m_Mode = m_NewMode; return d.SetSystemModeInternal(d.m_Mode); })
        | if_then([&]()->bool{ return m_Changed.MinDistance || m_Changed.MaxDistance || m_Changed.Timeout; }, 
                    [&]{ 
                            if (m_Changed.MinDistance) d.m_MinDistance = m_NewMinDistance;
                            if (m_Changed.MaxDistance) d.m_MaxDistance = m_NewMaxDistance;
                            if (m_Changed.Timeout) d.m_Timeout = m_NewTimeout;
                            return d.SendCommandV2(Cmd::WriteADB
                                    ,to_send(
                                        ADBParam{ADBRegs::MinDistance, d.m_MinDistance}
                                        , ADBParam{ADBRegs::MaxDistance, d.m_MaxDistance}
                                        , ADBParam{ADBRegs::Timeout, d.m_Timeout} 
                                        )
                                    ,to_recv());
                       })
        | if_then(m_GateChanges, 
              repeat_n(16, [&](uint8_t g)->ExpectedGenericCmdResult{ 
                    if (m_GateChanges & (uint32_t(3) << (g * 2))) //check if both
                    {
                        d.m_Gates[g].m_MoveThreshold = m_NewGates[g].move;
                        d.m_Gates[g].m_StillThreshold = m_NewGates[g].still;
                        return d.SendCommandV2(Cmd::WriteADB
                                ,to_send(
                                      ADBParam{ADBRegs::MoveThresholdGateBase + g, d.m_Gates[g].m_MoveThreshold}
                                    , ADBParam{ADBRegs::StillThresholdGateBase + g, d.m_Gates[g].m_StillThreshold}
                                    )
                                ,to_recv());
                    }
                    else if (m_GateChanges & (1 << (g * 2)))
                    {
                        d.m_Gates[g].m_MoveThreshold = m_NewGates[g].move;
                        return d.SendCommandV2(Cmd::WriteADB
                                ,to_send( ADBParam{ADBRegs::MoveThresholdGateBase + g, d.m_Gates[g].m_MoveThreshold})
                                ,to_recv());
                    }
                    else if (m_GateChanges & (1 << (g * 2 + 1)))
                    {
                        d.m_Gates[g].m_StillThreshold = m_NewGates[g].still;
                        return d.SendCommandV2(Cmd::WriteADB
                                ,to_send( ADBParam{ADBRegs::StillThresholdGateBase + g, d.m_Gates[g].m_StillThreshold})
                                ,to_recv());
                    }
                    else
                        return std::ref(d);
              })
          )
        | and_then([&]{ return d.CloseCommandMode(); })
        | transform_error([&](CmdErr e){ return e.e; });
}
