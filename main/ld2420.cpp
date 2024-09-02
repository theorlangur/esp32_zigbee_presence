#include <cstring>
#include "ld2420.hpp"
#include "generic_helpers.hpp"
#include "functional_helpers.hpp"

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
        | and_then([&](uart::Channel &c){ return c.SetPins(txPin, rxPin); })
        | transform_error([](::Err &c){ return Err{c, "LD2420::Init", ErrorCode::Init}; })
        | and_then([](uart::Channel &c)->ExpectedResult { return std::ref(static_cast<LD2420&>(c)); });
}

void LD2420::frame_t::WriteCustom(std::span<uint8_t> const d)
{
    len = (uint16_t)d.size();
    data[4] = uint8_t(len & 0xff);
    data[5] = uint8_t(len >> 8);

    constexpr size_t kDataOffset = 4 + 2;//header + len
    //copy actual data in place
    memcpy(data + kDataOffset, d.data(), len);
    constexpr uint8_t kFooter[] = {0x04, 0x03, 0x02, 0x01};
    //footer
    memcpy(data + kDataOffset + len, kFooter, sizeof(kFooter));
}

void LD2420::frame_t::WriteCmd(uint16_t cmd, std::span<uint8_t> const d)
{
    len = (uint16_t)d.size() + 2;
    data[4] = uint8_t(len & 0xff);
    data[5] = uint8_t(len >> 8);

    constexpr size_t kDataOffset = 4 + 2;//header + len
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
    return Send(frame.data, frame.len) 
            | transform_error([](::Err uartErr){ return Err{uartErr, "LD2420::SendFrame", ErrorCode::SendFrame}; })
            | and_then([&](uart::Channel &c)->ExpectedResult{ return std::ref(static_cast<LD2420&>(c)); });
}

LD2420::ExpectedDataResult LD2420::RecvFrame(frame_t &frame)
{
    return Read(frame.data, 4)
        | and_then([&](uart::Channel &c, size_t l)->uart::Channel::ExpectedValue<size_t>{
                if (l != 4 || !frame.VerifyHeader())
                    return std::unexpected(::Err{"LD2420::RecvFrame < 4"});
                return c.Read(frame.data + 4, 2);
          })
        | and_then([&](uart::Channel &c, size_t l)->uart::Channel::ExpectedValue<size_t>{
            if (l != 4)
                return std::unexpected(::Err{"LD2420::RecvFrame < 2"});
            frame.len = frame.data[4] + (frame.data[5] << 8);
            return c.Read(frame.data + 6, frame.len);
          })
        | transform_error([](::Err uartErr){ return CmdErr{{uartErr, "LD2420::RecvFrame", ErrorCode::RecvFrame_Malformed}, 0}; })
        | and_then([&](uart::Channel &c, size_t l)->ExpectedDataResult{
            if (l != frame.len)
                return std::unexpected(CmdErr{{::Err{}, "LD2420::RecvFrame < len", ErrorCode::RecvFrame_Incomplete}, 0});
            if (!frame.VerifyFooter())
                return std::unexpected(CmdErr{{::Err{}, "LD2420::RecvFrame < len", ErrorCode::RecvFrame_Malformed}, 0});

            return DataRetVal{*this, std::span<uint8_t>(frame.data + 6, frame.len)};
          });
}

LD2420::ExpectedDataResult LD2420::SendCommand(uint16_t cmd, const std::span<uint8_t> outData, std::span<uint8_t> inData)
{
    if (!inData.empty()  && (inData.size() < sizeof(frame_t)))
        return std::unexpected(CmdErr{Err{{}, "SendCommand", ErrorCode::SendCommand_InsufficientSpace}, 0});

    frame_t f;
    f.WriteCmd(cmd, outData);
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
