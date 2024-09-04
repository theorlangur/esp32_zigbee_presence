#include <cstring>
#include "ld2420.hpp"
#include "generic_helpers.hpp"
#include "functional_helpers.hpp"

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
    return Configure() 
        | and_then([&]{ return SetPins(txPin, rxPin); })
        | and_then([&]{ return Open(); })
        | transform_error([](::Err &c){ return Err{c, "LD2420::Init", ErrorCode::Init}; })
        | and_then([](uart::Channel &c)->ExpectedResult { return std::ref(static_cast<LD2420&>(c)); });
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
    return Send(frame.data, frame.TotalSize()) 
            | transform_error([](::Err uartErr){ return Err{uartErr, "LD2420::SendFrame", ErrorCode::SendFrame}; })
            | and_then([&](uart::Channel &c)->ExpectedResult{ return std::ref(static_cast<LD2420&>(c)); });
}

LD2420::ExpectedDataResult LD2420::RecvFrame(frame_t &frame)
{
    return Read(frame.data, 4, duration_ms_t(100))
        | and_then([&](uart::Channel &c, size_t l)->uart::Channel::ExpectedValue<size_t>{
                //printf("Read %d bytes\n", l);
                //printf("4 header bytes: %X %X %X %X\n", frame.data[0], frame.data[1], frame.data[2], frame.data[3]);
                //fflush(stdout);
                if (l != 4 || !frame.VerifyHeader())
                    return std::unexpected(::Err{"LD2420::RecvFrame < 4"});
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
    if (!inData.empty()  && (inData.size() < sizeof(frame_t)))
        return std::unexpected(CmdErr{Err{{}, "SendCommand", ErrorCode::SendCommand_InsufficientSpace}, 0});

    frame_t f;
    f.WriteCmd(cmd, outData);
    printf("Sending command %X\n", cmd);
    return Flush() //flushing whatever unread was in the buffer
        | transform_error([](::Err e){ return Err{e, "SendCommand", ErrorCode::SendCommand_Failed}; })
        | and_then([&]{ return SendFrame(f); })
    //return SendFrame(f)
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
    frame_t f;
    uint16_t protocol_version = 2;
    f.WriteCmd(0xff, std::span<uint8_t>((uint8_t*)&protocol_version, 2));
    return SendFrame(f) 
        | transform_error([](Err e){ return CmdErr{e, 0}; })
        | and_then([&]{ 
                printf("Sleeping before sending actual command");
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
    frame_t f;
    return SendCommand(0xfe, std::span<uint8_t>{}, f)
        | and_then([&]()->ExpectedCloseCmdModeResult{ return std::ref(*this); });
}

LD2420::ExpectedStrResult LD2420::GetVersion(version_buf_t &buf)
{
    frame_t &f = reinterpret_cast<frame_t &>(buf);
    return SendCommand(0x0, std::span<uint8_t>{}, f)
        | and_then([&](LD2420 &d, std::span<uint8_t> val)->ExpectedStrResult
          { 
              uint16_t *pLen = (uint16_t*)val.data();
              return StrRetVal{std::ref(*this), std::string_view((char *)val.data() + 2, *pLen)}; 
          });
}

LD2420::ExpectedSingleRawADBResult LD2420::ReadRawADBSingle(uint16_t param)
{
    frame_t f;
    std::span<uint8_t> data((uint8_t*)&param, sizeof(param));
    printf("Param to obtain: %X\n", param);
    return SendCommand(0x0008, data, f) 
        | and_then([](LD2420 &d, std::span<uint8_t> val)->ExpectedSingleRawADBResult{
                if (val.size() < 4)
                    return std::unexpected(CmdErr{Err{{}, "LD2420::ReadRawADBSingle", ErrorCode::SendCommand_WrongFormat}, (uint16_t)val.size()});
                if (val.size() > 4)
                {
                    printf("Unexpected bytes: ");
                    for(uint8_t b : val)
                        printf(" %X", b);
                    printf("\n");
                    fflush(stdout);
                }
                return SingleRawADBRetVal{std::ref(d), *(uint32_t*)val.data()};
          });
}

LD2420::ExpectedGenericCmdResult LD2420::SetSystemMode(uint16_t mode)
{
    return WriteRawSysMulti(LD2420::SetParam{uint16_t(0x0), mode});
}
