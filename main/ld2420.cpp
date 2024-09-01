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

LD2420::ExpectedResult LD2420::SendFrame(const uint8_t *pData, uint16_t len)
{
    uint8_t frame[64] = { 0xFD, 0xFC, 0xFB, 0xFA //header
        , uint8_t(len & 0xff), uint8_t(len >> 8) //length
    };
    constexpr size_t kDataOffset = 4 + 2;//header + len
    //copy actual data in place
    memcpy(frame + kDataOffset, pData, len);
    constexpr uint8_t kFooter[] = {0x04, 0x03, 0x02, 0x01};
    //footer
    memcpy(frame + kDataOffset + len, kFooter, sizeof(kFooter));

    size_t sz = kDataOffset + len + sizeof(kFooter);

    return Send(frame, sz) 
            | transform_error([](::Err uartErr){ return Err{uartErr, "LD2420::SendFrame", ErrorCode::SendFrame}; })
            | and_then([&](uart::Channel &c)->ExpectedResult{ return std::ref(static_cast<LD2420&>(c)); });
}
