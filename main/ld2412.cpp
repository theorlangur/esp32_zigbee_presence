#include <cstring>
#include "ld2412.hpp"
#include "generic_helpers.hpp"

#define DBG_UART Channel::DbgNow _dbg_uart{this}; 
#define DBG_ME DbgNow _dbg_me{this}; 

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
        case ErrorCode::FactoryResetFailed: return "FactoryResetFailed";
        case ErrorCode::BTFailed: return "BTFailed";
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
    SetDefaultWait(kDefaultWait);
    TRY_UART_COMM(Configure(), "Init", ErrorCode::Init);
    SetPins(txPin, rxPin);
    TRY_UART_COMM(Open(), "Init", ErrorCode::Init);
    return ReloadConfig();
}

LD2412::ExpectedResult LD2412::ReloadConfig()
{
    TRY_UART_COMM(OpenCommandMode(), "ReloadConfig", ErrorCode::SendCommand_Failed);
    TRY_UART_COMM(UpdateVersion(), "ReloadConfig", ErrorCode::SendCommand_Failed);
    TRY_UART_COMM(SendCommandV2(Cmd::ReadBaseParams, to_send(), to_recv(m_Configuration.m_Base)), "ReloadConfig", ErrorCode::SendCommand_Failed);
    TRY_UART_COMM(SendCommandV2(Cmd::GetMoveSensitivity, to_send(), to_recv(m_Configuration.m_MoveThreshold)), "ReloadConfig", ErrorCode::SendCommand_Failed);
    TRY_UART_COMM(SendCommandV2(Cmd::GetStillSensitivity, to_send(), to_recv(m_Configuration.m_StillThreshold)), "ReloadConfig", ErrorCode::SendCommand_Failed);
    TRY_UART_COMM(SendCommandV2(Cmd::GetMAC, to_send(uint16_t(0x0001)), to_recv(m_BluetoothMAC)), "ReloadConfig", ErrorCode::SendCommand_Failed);
    TRY_UART_COMM(CloseCommandMode(), "ReloadConfig", ErrorCode::SendCommand_Failed);
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::UpdateDistanceRes()
{
    TRY_UART_COMM(OpenCommandMode(), "UpdateDistanceRes", ErrorCode::SendCommand_Failed);
    TRY_UART_COMM(SendCommandV2(Cmd::GetDistanceRes, to_send(), to_recv(m_DistanceRes)), "UpdateDistanceRes", ErrorCode::SendCommand_Failed);
    TRY_UART_COMM(CloseCommandMode(), "UpdateDistanceRes", ErrorCode::SendCommand_Failed);
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::SwitchBluetooth(bool on)
{
    SetDefaultWait(kDefaultWait);
    TRY_UART_COMM(OpenCommandMode(), "SwitchBluetooth", ErrorCode::BTFailed);
    TRY_UART_COMM(SendCommandV2(Cmd::SwitchBluetooth, to_send(uint16_t(on)), to_recv()), "SwitchBluetooth", ErrorCode::BTFailed);
    TRY_UART_COMM(SendFrameV2(Cmd::Restart), "SwitchBluetooth", ErrorCode::BTFailed);
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    TRY_UART_COMM(uart::primitives::flush_and_wait(*this, kRestartTimeout), "SwitchBluetooth", ErrorCode::BTFailed);
    if (m_Mode != SystemMode::Simple)
    {
        auto rs = ChangeConfiguration().SetSystemMode(m_Mode).EndChange();
        TRY_UART_COMM(rs, "SwitchBluetooth", ErrorCode::BTFailed);
    }
    return ReloadConfig();
}

LD2412::ExpectedResult LD2412::Restart()
{
    SetDefaultWait(kDefaultWait);
    TRY_UART_COMM(OpenCommandMode(), "Restart", ErrorCode::RestartFailed);
    TRY_UART_COMM(SendFrameV2(Cmd::Restart), "Restart", ErrorCode::RestartFailed);
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    TRY_UART_COMM(uart::primitives::flush_and_wait(*this, kRestartTimeout), "Restart", ErrorCode::RestartFailed);
    if (m_Mode != SystemMode::Simple)
    {
        auto rs = ChangeConfiguration().SetSystemMode(m_Mode).EndChange();
        TRY_UART_COMM(rs, "SwitchBluetooth", ErrorCode::BTFailed);
    }
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::FactoryReset()
{
    SetDefaultWait(duration_ms_t(1000));
    TRY_UART_COMM(OpenCommandMode(), "FactoryReset", ErrorCode::FactoryResetFailed);
    TRY_UART_COMM(SendCommandV2(Cmd::FactoryReset, to_send(), to_recv()), "FactoryReset", ErrorCode::FactoryResetFailed);
    TRY_UART_COMM(SendFrameV2(Cmd::Restart), "FactoryReset", ErrorCode::FactoryResetFailed);
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    TRY_UART_COMM(uart::primitives::flush_and_wait(*this, kRestartTimeout), "FactoryReset", ErrorCode::FactoryResetFailed);
    if (m_Mode != SystemMode::Simple)
    {
        auto rs = ChangeConfiguration().SetSystemMode(m_Mode).EndChange();
        TRY_UART_COMM(rs, "FactoryReset", ErrorCode::FactoryResetFailed);
    }
    return ReloadConfig();
}

LD2412::ExpectedOpenCmdModeResult LD2412::OpenCommandMode()
{
    uint16_t protocol_version = 1;
    OpenCmdModeResponse r;
    if (auto r = SendFrameV2(Cmd::OpenCmd, protocol_version); !r)
        return std::unexpected(CmdErr{r.error(), 0});
    std::this_thread::sleep_for(duration_ms_t(100)); 

    if (auto rs = SendCommandV2(Cmd::OpenCmd, to_send(protocol_version), to_recv(r.protocol_version, r.buffer_size)); !rs)
        return std::unexpected(rs.error());

    return OpenCmdModeRetVal{std::ref(*this), r};
}

LD2412::ExpectedGenericCmdResult LD2412::CloseCommandMode()
{
    return SendCommandV2(Cmd::CloseCmd, to_send(), to_recv());
}

LD2412::ExpectedGenericCmdResult LD2412::SetSystemModeInternal(SystemMode mode)
{
    Cmd c = mode == SystemMode::Energy ? Cmd::EnterEngMode : Cmd::LeaveEngMode;
    return SendCommandV2(c, to_send(), to_recv());
}


LD2412::ExpectedGenericCmdResult LD2412::UpdateVersion()
{
    constexpr uint16_t kVersionBegin = 0x2412;
    return SendCommandV2(Cmd::ReadVer, to_send(), to_recv(uart::primitives::match_t{kVersionBegin}, m_Version));
}

LD2412::ExpectedResult LD2412::ReadFrame()
{
//ReadFrame: Read bytes: f4 f3 f2 f1 0b 00 02 aa 02 00 00 00 a0 00 64 55 00 f8 f7 f6 f5 
    constexpr uint8_t report_begin[] = {0xaa};
    constexpr uint8_t report_end[] = {0x55};
    SystemMode mode;
    uint8_t check;
    uint16_t reportLen = 0;
    TRY_UART_COMM(uart::primitives::read_until(*this, kDataFrameHeader[0], duration_ms_t(1000), "Searching for header"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    TRY_UART_COMM(uart::primitives::match_bytes(*this, kDataFrameHeader, "Matching header"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    TRY_UART_COMM(uart::primitives::read_any(*this, reportLen, mode), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    TRY_UART_COMM(uart::primitives::match_bytes(*this, report_begin, "Matching rep begin"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    TRY_UART_COMM(uart::primitives::read_into(*this, m_Presence), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);//simple Part of the detection is always there
    if (mode == SystemMode::Energy)
    {
        if ((reportLen - 4 - sizeof(m_Presence)) != sizeof(m_Engeneering))
            return std::unexpected(Err{{"Wrong engeneering size"}, "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed});
        TRY_UART_COMM(uart::primitives::read_into(*this, m_Engeneering), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    }
    TRY_UART_COMM(uart::primitives::match_bytes(*this, report_end, "Matching rep end"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    TRY_UART_COMM(uart::primitives::read_into(*this, check), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);//simple Part of the detection is always there
    TRY_UART_COMM(uart::primitives::match_bytes(*this, kDataFrameFooter, "Matching footer"), "ReadFrameReadFrame", ErrorCode::SimpleData_Malformed);
    return std::ref(*this);
}

LD2412::ExpectedResult LD2412::TryReadFrame(int attempts, bool flush, Drain drain)
{
    if (drain != Drain::No)
    {
        //printf("TryReadFRame: draining first\n");
        SetDefaultWait(duration_ms_t(0));
        int i = 0;
        for(; i < 100; ++i)
        {
            if (auto r = ReadFrame(); !r)
            {
                if (i > 0)//if i is at least 2 that means that at least 1 iteration was successful 
                {
                    break;
                }
                else if (drain == Drain::Try)
                {
                    //FMT_PRINT("TryReadFrame: no iterations; Trying to wait for read\n");
                    return TryReadFrame(attempts, flush, Drain::No);
                }else
                    return r;
            }
        }
    }else
    {
        //FMT_PRINT("TryReadFrame: no drain\n");
        SetDefaultWait(duration_ms_t(150));
        auto ec = (m_Mode == SystemMode::Energy) ? ErrorCode::EnergyData_Failure : ErrorCode::SimpleData_Failure;
        if (flush)
            TRY_UART_COMM(Flush(), "LD2412::TryReadFrame", ec);

        for(int i = 0; i < attempts; ++i)
        {
            if (auto r = ReadFrame(); !r)
            {
                if ((i + 1) == attempts)
                    return to_result(std::move(r), "LD2412::TryReadFrame", ec);
            }else
                return std::ref(*this);
        }
    }
    return std::ref(*this);
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
    m_Configuration.m_Base.m_MinDistanceGate = std::clamp(dist * 10 / 7, 1, 12);
    return *this;
}
LD2412::ConfigBlock& LD2412::ConfigBlock::SetMinDistanceRaw(uint8_t dist)
{
    m_Changed.MinDistance = true;
    m_Configuration.m_Base.m_MinDistanceGate = std::clamp(dist, uint8_t(1), uint8_t(12));
    return *this;
}
LD2412::ConfigBlock& LD2412::ConfigBlock::SetMaxDistance(int dist)
{
    m_Changed.MaxDistance = true;
    m_Configuration.m_Base.m_MaxDistanceGate = std::clamp(dist * 10 / 7, 1, 12);
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetMaxDistanceRaw(uint8_t dist)
{
    m_Changed.MaxDistance = true;
    m_Configuration.m_Base.m_MaxDistanceGate = std::clamp(dist, uint8_t(1), uint8_t(12));
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetTimeout(uint16_t t)
{
    m_Changed.Timeout = true;
    m_Configuration.m_Base.m_Duration = t;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetOutPinPolarity(bool lowOnPresence)
{
    m_Changed.OutPin = true;
    m_Configuration.m_Base.m_OutputPinPolarity = lowOnPresence;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetMoveThreshold(uint8_t gate, uint8_t energy)
{
    if (gate > 13)
        return *this;

    m_Changed.MoveThreshold = true;
    m_Configuration.m_MoveThreshold[gate] = energy;
    return *this;
}

LD2412::ConfigBlock& LD2412::ConfigBlock::SetStillThreshold(uint8_t gate, uint8_t energy)
{
    if (gate > 13)
        return *this;

    m_Changed.StillThreshold = true;
    m_Configuration.m_StillThreshold[gate] = energy;
    return *this;
}

LD2412::ExpectedResult LD2412::ConfigBlock::EndChange()
{
    if (!m_Changes)
        return std::ref(d);
    ScopeExit clearChanges = [&]{ m_Changes = 0; };
    TRY_UART_COMM(d.OpenCommandMode(), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    if (m_Changed.Mode)
    {
        d.m_Mode = m_NewMode; 
        TRY_UART_COMM(d.SetSystemModeInternal(d.m_Mode), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    if (m_Changed.MinDistance || m_Changed.MaxDistance || m_Changed.Timeout || m_Changed.OutPin)
    {
        d.m_Configuration.m_Base = m_Configuration.m_Base;
        TRY_UART_COMM(d.SendCommandV2(Cmd::WriteBaseParams ,to_send(d.m_Configuration.m_Base) ,to_recv()), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    if (m_Changed.MoveThreshold)
    {
        std::ranges::copy(m_Configuration.m_MoveThreshold, d.m_Configuration.m_MoveThreshold);
        TRY_UART_COMM(d.SendCommandV2(Cmd::SetMoveSensitivity ,to_send(d.m_Configuration.m_MoveThreshold) ,to_recv()), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    if (m_Changed.StillThreshold)
    {
        std::ranges::copy(m_Configuration.m_StillThreshold, d.m_Configuration.m_StillThreshold);
        TRY_UART_COMM(d.SendCommandV2(Cmd::SetStillSensitivity ,to_send(d.m_Configuration.m_StillThreshold) ,to_recv()), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    }
    TRY_UART_COMM(d.CloseCommandMode(), "LD2412::ConfigBlock::EndChange", ErrorCode::SendCommand_Failed);
    return std::ref(d);
}
