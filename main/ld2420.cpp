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
        | transform_error([](::Err &c){ return Err{c, "LD2420::Init", ErrorCode::Init}; })
        | and_then([&]()->ExpectedResult { return std::ref(*this); });
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

void LD2420::frame_t::WriteCustom(std::span<uint8_t> const d)
{
    len = (uint16_t)d.size();
    data[4] = uint8_t(len & 0xff);
    data[5] = uint8_t(len >> 8);

    //copy actual data in place
    memcpy(data + kDataOffset, d.data(), len);
    constexpr uint8_t kFooter[] = {0x04, 0x03, 0x02, 0x01};
    //footer
    memcpy(data + kDataOffset + len, kFooter, sizeof(kFooter));
}

uint16_t LD2420::frame_t::TotalSize() const
{
    return len + kDataOffset + kFooterSize;
}

void LD2420::frame_t::WriteCmd(uint16_t cmd, std::span<uint8_t> const d)
{
    len = (uint16_t)d.size() + 2;
    data[4] = uint8_t(len & 0xff);
    data[5] = uint8_t(len >> 8);

    //cmd
    data[6] = uint8_t(cmd & 0xff);
    data[7] = uint8_t(cmd >> 8);
    //copy actual data in place
    memcpy(data + kDataOffset + 2, d.data(), d.size());
    constexpr uint8_t kFooter[] = {0x04, 0x03, 0x02, 0x01};
    //footer
    memcpy(data + kDataOffset + len, kFooter, sizeof(kFooter));
}

bool LD2420::frame_t::VerifyHeader() const
{
    return (data[0] == 0xFD) && (data[1] ==  0xFC) && (data[2]== 0xFB) && (data[3] == 0xFA);
}

bool LD2420::frame_t::VerifyFooter() const
{
    return (data[6 + len + 0] == 0x04) && (data[6 + len + 1] ==  0x03) && (data[6 + len + 2]== 0x02) && (data[6 + len + 3] == 0x01);
}

LD2420::ExpectedResult LD2420::SendFrame(const frame_t &frame)
{
    using namespace functional;
    if constexpr (kDebugFrame)
    {
        printf("Sending frame of %d bytes\n", frame.TotalSize());
        for(int i = 0; i < frame.TotalSize(); ++i)
            printf(" %X", frame.data[i]);
        printf("\n");
    }
    return Send(frame.data, frame.TotalSize()) 
            | transform_error([](::Err uartErr){ return Err{uartErr, "LD2420::SendFrame", ErrorCode::SendFrame}; })
            | and_then([&]()->ExpectedResult{ return std::ref(*this); });
}

LD2420::ExpectedDataResult LD2420::RecvFrame(frame_t &frame)
{
    using namespace functional;
    return Read(frame.data, 4, duration_ms_t(100))
        | and_then([&](uart::Channel &c, size_t l)->uart::Channel::ExpectedValue<size_t>{
                if (l != 4 || !frame.VerifyHeader())
                {
                    if constexpr (kDebugFrame)
                        printf("RecvFrame Failure: %d bytes\n", l);
                    return std::unexpected(::Err{"LD2420::RecvFrame < 4"});
                }
                return c.Read(frame.data + 4, 2);
          })
        | and_then([&](uart::Channel &c, size_t l)->uart::Channel::ExpectedValue<size_t>{
            //printf("Read %d bytes\n", l);
            //printf("2 len bytes: %X %X\n", frame.data[4], frame.data[5]);
            if (l != 2)
                return std::unexpected(::Err{"LD2420::RecvFrame < 2"});
            frame.len = frame.data[4] + (frame.data[5] << 8);
            return c.Read(frame.data + frame_t::kDataOffset, frame.len + frame_t::kFooterSize);
          })
        | transform_error([](::Err uartErr){ return CmdErr{{uartErr, "LD2420::RecvFrame", ErrorCode::RecvFrame_Malformed}, 0}; })
        | and_then([&](uart::Channel &c, size_t l)->ExpectedDataResult{
            if (l != (frame.len + frame_t::kFooterSize))
                return std::unexpected(CmdErr{{::Err{}, "LD2420::RecvFrame < len", ErrorCode::RecvFrame_Incomplete}, 0});
            if (!frame.VerifyFooter())
                return std::unexpected(CmdErr{{::Err{}, "LD2420::RecvFrame < len", ErrorCode::RecvFrame_Malformed}, 0});

            return DataRetVal{*this, std::span<uint8_t>(frame.data + frame_t::kDataOffset, frame.len)};
          });
}

LD2420::ExpectedDataResult LD2420::SendCommand(uint16_t cmd, const std::span<uint8_t> outData, std::span<uint8_t> inData)
{
    using namespace functional;
    if (!inData.empty()  && (inData.size() < sizeof(frame_t)))
        return std::unexpected(CmdErr{Err{{}, "SendCommand", ErrorCode::SendCommand_InsufficientSpace}, 0});

    frame_t f;
    f.WriteCmd(cmd, outData);
    if constexpr (kDebugCommands)
        printf("Sending command %X; Span: %d\n", cmd, outData.size());
    return Flush() //flushing whatever unread was in the buffer
        | transform_error([](::Err e){ return Err{e, "SendCommand", ErrorCode::SendCommand_Failed}; })
        | and_then([&]{ return SendFrame(f); })
        | transform_error([](Err e){ return CmdErr{e, 0}; })
        | and_then([&]{ return WaitAllSent() | transform_error([](::Err e){ return CmdErr{Err{e, "SendCommandOnly", ErrorCode::SendCommand_FailedWrite}, 0};}); })
        | and_then([&]{ frame_t *pF = reinterpret_cast<frame_t *>(inData.data()); return RecvFrame(*pF); })
        | and_then([&](LD2420 &c, std::span<uint8_t> d) -> ExpectedDataResult{ 
                uint16_t resp_cmd = d[0] + (d[1] << 8);
                if (resp_cmd != (cmd | 0x0100))
                    return std::unexpected(CmdErr{Err{{}, "SendCommandOnly", ErrorCode::SendCommand_InvalidResponse}, 0});

                uint16_t status = d[2] + (d[3] << 8);
                if (status != 0)//non-0 is an error
                    return std::unexpected(CmdErr{Err{{}, "SendCommandOnly", ErrorCode::SendCommand_Failed}, status});
                return DataRetVal{*this, d.subspan(4, d.size() - 4)};
          });
}

LD2420::ExpectedOpenCmdModeResult LD2420::OpenCommandMode()
{
    using namespace functional;
    frame_t f;
    uint16_t protocol_version = 2;
    f.WriteCmd(0xff, std::span<uint8_t>((uint8_t*)&protocol_version, 2));
    return SendFrame(f) 
        | transform_error([](Err e){ return CmdErr{e, 0}; })
        | and_then([&]{ 
                //printf("Sleeping before sending actual command");
                std::this_thread::sleep_for(duration_ms_t(100)); 
                return SendCommand(0xff, protocol_version, f); 
          })
        | and_then([&](LD2420 &d, std::span<uint8_t> res)->ExpectedOpenCmdModeResult{
                if (res.size() != 4)
                    return std::unexpected(CmdErr{Err{{}, "OpenCommandMode", ErrorCode::SendCommand_WrongFormat}, 0});
                OpenCmdModeResponse r;
                r.protocol_version = *(uint16_t*)res.data();
                r.buffer_size = *(uint16_t*)(res.data() + 2);
                return OpenCmdModeRetVal{std::ref(d), r};
          });
}

LD2420::ExpectedCloseCmdModeResult LD2420::CloseCommandMode()
{
    using namespace functional;
    frame_t f;
    return SendCommand(0xfe, std::span<uint8_t>{}, f)
        | and_then([&]()->ExpectedCloseCmdModeResult{ return std::ref(*this); });
}

std::string_view LD2420::GetVersion() const
{
    return m_Version;
}

LD2420::ExpectedSingleRawADBResult LD2420::ReadRawADBSingle(uint16_t param)
{
    using namespace functional;
    frame_t f;
    std::span<uint8_t> data((uint8_t*)&param, sizeof(param));
    //printf("Param to obtain: %X\n", param);
    return SendCommand(0x0008, data, f) 
        | and_then([](LD2420 &d, std::span<uint8_t> val)->ExpectedSingleRawADBResult{
                if (val.size() < 4)
                    return std::unexpected(CmdErr{Err{{}, "LD2420::ReadRawADBSingle", ErrorCode::SendCommand_WrongFormat}, (uint16_t)val.size()});
                if (val.size() > 4)
                {
                    if constexpr (kDebugCommands)
                    {
                        printf("Unexpected bytes: ");
                        for(uint8_t b : val) printf(" %X", b);
                        printf("\n");
                        fflush(stdout);
                    }
                }
                return SingleRawADBRetVal{std::ref(d), *(uint32_t*)val.data()};
          });
}

LD2420::ExpectedGenericCmdResult LD2420::SetSystemModeInternal(SystemMode mode)
{
    return WriteRawSysMulti(LD2420::SetParam{uint16_t(0x0), (uint32_t)mode});
}


LD2420::ExpectedGenericCmdResult LD2420::UpdateVersion()
{
    using namespace functional;
    frame_t f;
    return SendCommand(0x0, std::span<uint8_t>{}, f)
        | and_then([&](LD2420 &d, std::span<uint8_t> val)->ExpectedGenericCmdResult
          { 
              uint16_t *pLen = (uint16_t*)val.data();
              if (*pLen >= sizeof(m_Version))
                    return std::unexpected(CmdErr{Err{{}, "LD2420::UpdateVersion", ErrorCode::SendCommand_InsufficientSpace}, *pLen});
              memcpy(m_Version, val.data() + sizeof(uint16_t), *pLen);
              m_Version[*pLen] = 0;
              return std::ref(*this);
          });
}

LD2420::ExpectedGenericCmdResult LD2420::UpdateSystemMode()
{
    using namespace functional;
    return ReadRawSysMulti(SysRegs::Mode)
        | and_then([&](LD2420 &d, Params<1> val)->ExpectedGenericCmdResult
          { 
              memcpy(&m_Mode, &val.value[0], sizeof(val.value[0]));
              return std::ref(*this);
          });
}

LD2420::ExpectedGenericCmdResult LD2420::UpdateMinMaxTimeout()
{
    using namespace functional;
    return ReadRawADBMulti(ADBRegs::MinDistance, ADBRegs::MaxDistance, ADBRegs::Timeout)
        | and_then([&](LD2420 &d, Params<3> val)->ExpectedGenericCmdResult
          { 
              memcpy(&m_MinDistance, &val.value[0], sizeof(val.value[0]));
              memcpy(&m_MaxDistance, &val.value[1], sizeof(val.value[1]));
              memcpy(&m_Timeout, &val.value[2], sizeof(val.value[2]));
              return std::ref(*this);
          });
}

LD2420::ExpectedGenericCmdResult LD2420::UpdateGate(uint8_t gate)
{
    using namespace functional;
    return ReadRawADBMulti((uint16_t)ADBRegs::MoveThresholdGateBase + gate, (uint16_t)ADBRegs::StillThresholdGateBase + gate)
        | and_then([&](LD2420 &d, Params<2> val)->ExpectedGenericCmdResult
          { 
              memcpy(&m_Gates[gate].m_MoveThreshold, &val.value[0], sizeof(Gate::m_MoveThreshold));
              memcpy(&m_Gates[gate].m_StillThreshold, &val.value[1], sizeof(Gate::m_StillThreshold));
              return std::ref(*this);
          });
}

LD2420::ExpectedResult LD2420::TryFillBuffer(size_t s)
{
    using namespace functional;
    if (!m_BufferEmpty)
    {
        size_t avail = m_BufferWriteTo < m_BufferReadFrom ? (m_BufferReadFrom - m_BufferWriteTo) : sizeof(m_Buffer) - m_BufferWriteTo + m_BufferReadFrom;
        if (avail < s)
            return std::unexpected(Err{{}, "LD2420::TryFillBuffer", ErrorCode::FillBuffer_NoSpace});
    }

    return ExpectedResult{std::ref(*this)} |
        repeat_while(
                /*condition*/[&]()->std::expected<bool, Err>{ return s > 0; },
                /*iteration*/[&]()->ExpectedResult{
                    size_t availCont = m_BufferWriteTo < m_BufferReadFrom ? (m_BufferReadFrom - m_BufferWriteTo) : sizeof(m_Buffer) - m_BufferWriteTo;
                    if (availCont > s) availCont = s;
                    return Read((uint8_t*)m_Buffer + m_BufferWriteTo, availCont, duration_ms_t{100})
                        | transform_error([&](::Err e){ return Err{e, "LD2420::TryFillBuffer", ErrorCode::FillBuffer_ReadFailure};})
                        | and_then([&](size_t l)->ExpectedResult{
                                s -= l;
                                m_BufferWriteTo = (m_BufferWriteTo + l) % sizeof(m_Buffer);
                                return std::ref(*this);
                        });
                },
                /*on no iterations*/[&]()->ExpectedResult{return std::ref(*this);}
        );
}

LD2420::ExpectedResult LD2420::ReadSimpleFrame()
{
    using namespace functional;
    SetDefaultWait(duration_ms_t(150));
    uint16_t val = 0;
    auto ParseNum = repeat_while(
            [&]()->std::expected<bool,::Err>{ return PeekByte() | and_then([&](uint8_t b)->std::expected<bool,::Err>{ return b >= '0' && b <= '9'; }); },
            [&]{ return ReadByte() | and_then([&](uint8_t b)->Channel::ExpectedResult{ val = val * 10 + b - '0'; return std::ref((Channel&)*this);}); },
            [&]()->Channel::ExpectedResult{ return std::ref((Channel&)*this); }
            );

    return start_sequence(*this)
                    | uart::read_until(*this, 'O')
                    | uart::match_any_str(*this, "ON", "OFF")
                    | and_then([&](uart::Channel &d, int match)->Channel::ExpectedResult{
                            m_Presence.m_Detected = match == 0;
                            return std::ref((uart::Channel&)*this);
                      })
                    | uart::match_bytes(*this, "\r\n")
                    | and_then([&]()->Channel::ExpectedResult{
                            if (m_Presence.m_Detected)
                                return 
                                    start_sequence(*this)
                                    | uart::match_bytes(*this, "Range ")
                                    | std::move(ParseNum)
                                    | uart::match_bytes(*this, "\r\n")
                                    | and_then([&]()->Channel::ExpectedResult{
                                            m_Presence.m_Distance = float(val) / 100;
                                            return std::ref((uart::Channel&)*this);
                                    });
                            else
                                return std::ref((uart::Channel&)*this);
                            })
                    | transform_error([&](::Err e){ return Err{e, "LD2420::ReadSimpleFrameV2", ErrorCode::FillBuffer_ReadFailure};})
                    | and_then([&]()->ExpectedResult{ return std::ref(*this); });
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
    uint16_t reportLen = 0;
    uint16_t distance = 0;
    return start_sequence()
        | uart::read_until(*this, 0xf4)
        | uart::match_bytes(*this, header)
        | uart::read_into(*this, reportLen)
        | uart::read_into(*this, m_Presence.m_Detected)
        | uart::read_into(*this, distance)
        | transform_error([&](::Err e){ return Err{e, "LD2420::ReadEnergyFrame", ErrorCode::EnergyData_Failure};})
        | and_then([&]()->ExpectedResult{ return std::ref(*this); });
}

LD2420::ExpectedResult LD2420::UpdateMinMaxTimeoutConfig()
{
    using namespace functional;
    return OpenCommandMode()
        | and_then([&]{ return WriteRawADBMulti(
                    ADBParam{uint16_t(ADBRegs::MinDistance), m_MinDistance}
                    , ADBParam{uint16_t(ADBRegs::MaxDistance), m_MaxDistance}
                    , ADBParam{uint16_t(ADBRegs::Timeout), m_Timeout}); })
        | and_then([&]{ return CloseCommandMode(); })
        | transform_error([&](CmdErr e){ return e.e; });
}

