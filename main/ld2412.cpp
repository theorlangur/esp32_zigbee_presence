#include <cstring>
#include "ld2412.hpp"
#include "generic_helpers.hpp"
#include "functional/functional.hpp"
#include "uart_functional.hpp"

const char* LD2412::err_to_str(ErrorCode e)
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

LD2412::LD2412(uart::Port p, int baud_rate):
    uart::Channel(p, baud_rate)
{
    SetParity(uart::Parity::Disable);
    SetHWFlowControl(uart::HWFlowCtrl::Disable);
    SetQueueSize(10);
    SetRxBufferSize(256);
    SetTxBufferSize(256);
}

void LD2412::SetPort(uart::Port p)
{
    Channel::SetPort(p);
}

uart::Port LD2412::GetPort() const
{
    return Channel::GetPort();
}

LD2412::ExpectedResult LD2412::Init(int txPin, int rxPin)
{
    using namespace functional;
    return Configure() 
        | and_then([&]{ return SetPins(txPin, rxPin); })
        | and_then([&]{ return Open(); })
        | AdaptToResult("LD2412::Init", ErrorCode::Init);
}

LD2412::ExpectedResult LD2412::ReloadConfig()
{
    return std::ref(*this);
    //using namespace functional;
    //return OpenCommandMode()
    //    | and_then([&]{ return UpdateVersion(); })
    //    | and_then([&]{ return UpdateMinMaxTimeout(); })
    //    | repeat_n(16, [&](uint8_t i){ return UpdateGate(i); })
    //    | and_then([&]{ return CloseCommandMode(); })
    //    | transform_error([&](CmdErr e){ return e.e; });
}

LD2412::ExpectedResult LD2412::Restart()
{
    return std::ref(*this);
    //using namespace functional;
    //return OpenCommandMode()
    //    | transform_error([&](CmdErr e){ return e.e; })
    //    | and_then([&]{ return SendFrameV2(Cmd::Restart); })
    //    | uart::flush_and_wait(*this, kRestartTimeout, AdaptToResult("LD2412::Restart", ErrorCode::RestartFailed))
    //    | if_then(//after restart the default mode 'Simple'. We might want to switch
    //      /*if*/    [&]{ return m_Mode != SystemMode::Simple; },
    //      /*then*/  [&]{ return ChangeConfiguration().SetSystemMode(m_Mode).EndChange(); }
    //            );
}

LD2412::ExpectedResult LD2412::FactoryReset()
{
//    auto &ch = ChangeConfiguration()
//                .SetTimeout(120)
//                .SetMinDistanceRaw(1)
//                .SetMaxDistanceRaw(12);
//    constexpr uint16_t kMoveThreshold[] = {
////Gates:  0      1     2    3    4    5    6    7    8    9    10   11   12   13   14   15
//        60000, 30000, 400, 300, 250, 250, 250, 250, 300, 250, 250, 250, 250, 200, 200, 200
//    };
//    constexpr uint16_t kStillThreshold[] = {
////Gates:  0      1     2    3    4    5    6    7    8    9    10   11   12   13   14   15
//        40000, 20000, 200, 250, 150, 150, 150, 150, 150, 150, 150, 150, 100, 100, 100, 100
//    };
//
//    for(uint8_t g = 0; g < 16; ++g)
//        ch.SetMoveThreshold(g, kMoveThreshold[g])
//          .SetStillThreshold(g, kStillThreshold[g]);
//
//    return ch.EndChange();
    return std::ref(*this);
}

LD2412::ExpectedOpenCmdModeResult LD2412::OpenCommandMode()
{
    using namespace functional;
    uint16_t protocol_version = 1;
    OpenCmdModeResponse r;
    return SendFrameV2(Cmd::OpenCmd, protocol_version)
        | and_then([&]{ std::this_thread::sleep_for(duration_ms_t(100)); })
        | transform_error([](Err e){ return CmdErr{e, 0}; })
        | and_then([&]{ return SendCommandV2(Cmd::OpenCmd, to_send(protocol_version), to_recv(r.protocol_version, r.buffer_size)); })
        | and_then([&]()->ExpectedOpenCmdModeResult{ return OpenCmdModeRetVal{std::ref(*this), r}; });
}

LD2412::ExpectedGenericCmdResult LD2412::CloseCommandMode()
{
    using namespace functional;
    return SendCommandV2(Cmd::CloseCmd, to_send(), to_recv())
            | and_then([&]()->ExpectedGenericCmdResult{ return std::ref(*this); });
}

std::string_view LD2412::GetVersion() const
{
    return m_Version;
}

LD2412::ExpectedGenericCmdResult LD2412::SetSystemModeInternal(SystemMode mode)
{
    Cmd c = mode == SystemMode::Energy ? Cmd::EnterEngMode : Cmd::LeaveEngMode;
    return SendCommandV2(c, to_send(), to_recv());
}


LD2412::ExpectedGenericCmdResult LD2412::UpdateVersion()
{
    using namespace functional;
    uint16_t verLen;
    return SendCommandV2(Cmd::ReadVer, to_send(), to_recv(verLen, uart::read_var_t{verLen, m_Version})) 
            | and_then([&]{ m_Version[verLen] = 0; });
}

//LD2412::ExpectedGenericCmdResult LD2412::UpdateMinMaxTimeout()
//{
//    using namespace functional;
//    return SendCommandV2(Cmd::ReadADB, to_send(ADBRegs::MinDistance, ADBRegs::MaxDistance, ADBRegs::Timeout), to_recv(m_MinDistance, m_MaxDistance, m_Timeout)); 
//}
//
//LD2412::ExpectedGenericCmdResult LD2412::UpdateGate(uint8_t gate)
//{
//    using namespace functional;
//    return SendCommandV2(Cmd::ReadADB
//            , to_send(ADBRegs::MoveThresholdGateBase + gate, ADBRegs::StillThresholdGateBase + gate)
//            , to_recv(m_Gates[gate].m_MoveThreshold, m_Gates[gate].m_StillThreshold)); 
//}

LD2412::ExpectedResult LD2412::ReadFrame()
{
//ReadFrame: Read bytes: f4 f3 f2 f1 0b 00 02 aa 02 00 00 00 a0 00 64 55 00 f8 f7 f6 f5 
    using namespace functional;
    constexpr uint8_t header[] = {0xf4, 0xf3, 0xf2, 0xf1};
    constexpr uint8_t footer[] = {0xf8, 0xf7, 0xf6, 0xf5};
    constexpr uint8_t report_begin[] = {0xaa};
    constexpr uint8_t report_end[] = {0x55};
    SystemMode mode;
    uint8_t check;
    uint16_t reportLen = 0;
    return start_sequence()
        | uart::read_until(*this, header[0], duration_ms_t(1000), "Searching for header")
        | uart::match_bytes(*this, header, "Matching header")
        | uart::read_into(*this, reportLen, "Reading report len")
        | uart::read_into(*this, mode, "Reading mode")
        | uart::match_bytes(*this, report_begin, "Matching rep begin")
        | if_then_else(
                [&]{ return mode == SystemMode::Simple; }
                ,[&]{ return  start_sequence() | uart::read_into(*this, m_Presence);}
                ,[&]{ return  start_sequence() | uart::skip_bytes(*this, reportLen - 4); }//skip all except for [<type> 0xAA and 0x55 <Check>]
          )
        | uart::match_bytes(*this, report_end)
        | uart::read_into(*this, check)
        | uart::match_bytes(*this, footer)
        | AdaptToResult("LD2412::ReadEnergyFrame", ErrorCode::EnergyData_Failure)
        ;
}

LD2412::ExpectedResult LD2412::TryReadFrame(int attempts, bool flush, Drain drain)
{
    using namespace functional;
    if (drain != Drain::No)
    {
        using ExpectedCondition = std::expected<bool, ::Err>;
        //printf("TryReadFRame: draining first\n");
        SetDefaultWait(duration_ms_t(0));
        int i = 0, maxIterations = 100;
        auto r = start_sequence(std::ref(*this)) 
            | repeat_while(
                     [&]()->ExpectedCondition{ return i < maxIterations; }
                    ,[&]{ ++i; 
                        //vTaskDelay(1);
                        //auto r = (m_Mode == SystemMode::Energy) ? ReadEnergyFrame() : ReadSimpleFrame(); 
                        //if (!r)
                        //{
                        //    printf("TryReadFRame: drain iteration %d failed\n", i);
                        //}else
                        //{
                        //    //printf("TryReadFRame: drain iteration %d ok\n", i);
                        //}
                        //return r;
                        return ReadFrame(); 
                    }
                    ,[&]()->ExpectedResult{ return std::ref(*this); }
                    );
        if (i > 1)//if i is at least 2 that means that at least 1 iteration was successful 
        {
            //FMT_PRINT("TryReadFrame: {} iterations\n", i);
            return std::ref(*this);
        }
        else if (drain == Drain::Try)
        {
            //FMT_PRINT("TryReadFrame: no iterations; Trying to wait for read\n");
            return TryReadFrame(attempts, flush, Drain::No);
        }
        else
        {
            //FMT_PRINT("TryReadFrame: result\n");
            return r;
        }
    }else
    {
        //FMT_PRINT("TryReadFrame: no drain\n");
        SetDefaultWait(duration_ms_t(150));
        return start_sequence(std::ref(*this)) 
            | if_then(
                    [&]{ return flush; }
                    ,[&]{ return Flush() | AdaptToResult("LD2412::TryReadFrame", (m_Mode == SystemMode::Energy) ? ErrorCode::EnergyData_Failure : ErrorCode::SimpleData_Failure); }
              )
            | retry_on_fail(attempts, [&]{ return ReadFrame(); });
    }
}

/**********************************************************************/
/* ConfigBlock                                                        */
/**********************************************************************/
LD2412::ConfigBlock& LD2412::ConfigBlock::SetSystemMode(SystemMode mode)
{
    m_Changed.Mode = true;
    m_NewMode = mode;
    return *this;
}
LD2412::ConfigBlock& LD2412::ConfigBlock::SetMinDistance(int dist)
{
    m_Changed.MinDistance = true;
    m_NewMinDistance = std::clamp(dist * 10 / 7, 1, 12);
    return *this;
}
LD2412::ConfigBlock& LD2412::ConfigBlock::SetMinDistanceRaw(uint32_t dist)
{
    m_Changed.MinDistance = true;
    m_NewMinDistance = std::clamp(dist, uint32_t(1), uint32_t(12));
    return *this;
}
LD2412::ConfigBlock& LD2412::ConfigBlock::SetMaxDistance(int dist)
{
    m_Changed.MaxDistance = true;
    m_NewMaxDistance = std::clamp(dist * 10 / 7, 1, 12);
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetMaxDistanceRaw(uint32_t dist)
{
    m_Changed.MaxDistance = true;
    m_NewMaxDistance = std::clamp(dist, uint32_t(1), uint32_t(12));
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetTimeout(uint32_t t)
{
    m_Changed.Timeout = true;
    m_NewTimeout = t;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetMoveThreshold(uint8_t gate, uint16_t energy)
{
    if (gate > 15)
        return *this;

    m_GateChanges |= uint32_t(1) << (gate * 2);
    m_NewGates[gate].move = energy;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetStillThreshold(uint8_t gate, uint16_t energy)
{
    if (gate > 15)
        return *this;

    m_GateChanges |= uint32_t(1) << (gate * 2 + 1);
    m_NewGates[gate].still = energy;
    return *this;
}

LD2412::ExpectedResult LD2412::ConfigBlock::EndChange()
{
        return std::ref(d);
    //using namespace functional;
    //if (!m_GateChanges && !m_MiscChanges)
    //    return std::ref(d);
    //ScopeExit clearChanges = [&]{ m_GateChanges = m_MiscChanges = 0; };
    //return d.OpenCommandMode()
    //    | if_then([&]()->bool{ return m_Changed.Mode; }
    //                , [&]{ d.m_Mode = m_NewMode; return d.SetSystemModeInternal(d.m_Mode); })
    //    | if_then([&]()->bool{ return m_Changed.MinDistance || m_Changed.MaxDistance || m_Changed.Timeout; }, 
    //                [&]{ 
    //                        if (m_Changed.MinDistance) d.m_MinDistance = m_NewMinDistance;
    //                        if (m_Changed.MaxDistance) d.m_MaxDistance = m_NewMaxDistance;
    //                        if (m_Changed.Timeout) d.m_Timeout = m_NewTimeout;
    //                        return d.SendCommandV2(Cmd::WriteADB
    //                                ,to_send(
    //                                    ADBParam{ADBRegs::MinDistance, d.m_MinDistance}
    //                                    , ADBParam{ADBRegs::MaxDistance, d.m_MaxDistance}
    //                                    , ADBParam{ADBRegs::Timeout, d.m_Timeout} 
    //                                    )
    //                                ,to_recv());
    //                   })
    //    | if_then(m_GateChanges, 
    //          repeat_n(16, [&](uint8_t g)->ExpectedGenericCmdResult{ 
    //                if (m_GateChanges & (uint32_t(3) << (g * 2))) //check if both
    //                {
    //                    d.m_Gates[g].m_MoveThreshold = m_NewGates[g].move;
    //                    d.m_Gates[g].m_StillThreshold = m_NewGates[g].still;
    //                    return d.SendCommandV2(Cmd::WriteADB
    //                            ,to_send(
    //                                  ADBParam{ADBRegs::MoveThresholdGateBase + g, d.m_Gates[g].m_MoveThreshold}
    //                                , ADBParam{ADBRegs::StillThresholdGateBase + g, d.m_Gates[g].m_StillThreshold}
    //                                )
    //                            ,to_recv());
    //                }
    //                else if (m_GateChanges & (1 << (g * 2)))
    //                {
    //                    d.m_Gates[g].m_MoveThreshold = m_NewGates[g].move;
    //                    return d.SendCommandV2(Cmd::WriteADB
    //                            ,to_send( ADBParam{ADBRegs::MoveThresholdGateBase + g, d.m_Gates[g].m_MoveThreshold})
    //                            ,to_recv());
    //                }
    //                else if (m_GateChanges & (1 << (g * 2 + 1)))
    //                {
    //                    d.m_Gates[g].m_StillThreshold = m_NewGates[g].still;
    //                    return d.SendCommandV2(Cmd::WriteADB
    //                            ,to_send( ADBParam{ADBRegs::StillThresholdGateBase + g, d.m_Gates[g].m_StillThreshold})
    //                            ,to_recv());
    //                }
    //                else
    //                    return std::ref(d);
    //          })
    //      )
    //    | and_then([&]{ return d.CloseCommandMode(); })
    //    | transform_error([&](CmdErr e){ return e.e; });
}
