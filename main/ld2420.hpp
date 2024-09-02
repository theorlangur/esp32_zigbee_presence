#ifndef LD2420_H_
#define LD2420_H_

#include "uart.hpp"
#include <span>

class LD2420: protected uart::Channel
{
public:
    enum class ErrorCode: uint8_t
    {
        Ok,
        Init,
        SendFrame,
        SendFrame_Incomplete,

        SendCommand_InvalidResponse,
        SendCommand_FailedWrite,
        SendCommand_FailedRead,
        SendCommand_WrongFormat,
        SendCommand_Failed,
        SendCommand_InsufficientSpace,

        RecvFrame_Malformed,
        RecvFrame_Incomplete,
    };
    static const char* err_to_str(ErrorCode e);

    using Ref = std::reference_wrapper<LD2420>;
    struct Err
    {
        ::Err uartErr;
        const char *pLocation;
        ErrorCode code;
    };
    using ExpectedResult = std::expected<Ref, Err>;

    template<typename V>
    using RetVal = RetValT<Ref, V>;

    template<class V>
    using ExpectedValue = std::expected<RetVal<V>, Err>;

    LD2420(uart::Port p, int baud_rate = 115200);

    ExpectedResult Init(int txPin, int rxPin);
private:
    struct frame_t
    {
        uint8_t data[64] = {0xFD, 0xFC, 0xFB, 0xFA};
        uint16_t len;

        void WriteCustom(std::span<uint8_t> const d);
        void WriteCmd(uint16_t cmd, std::span<uint8_t> const d);
        bool VerifyHeader() const;
        bool VerifyFooter() const;
    };
    struct CmdErr
    {
        Err e;
        uint16_t returnCode;
    };
    using DataRetVal = RetValT<Ref, std::span<uint8_t>>;
    using ExpectedDataResult = std::expected<DataRetVal, CmdErr>;
    ExpectedDataResult SendCommand(uint16_t cmd, const std::span<uint8_t> outData, std::span<uint8_t> inData);
    ExpectedResult SendFrame(const frame_t &frame);
    ExpectedDataResult RecvFrame(frame_t &frame);
};

#endif
