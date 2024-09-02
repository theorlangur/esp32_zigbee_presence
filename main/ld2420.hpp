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
public:
    struct CmdErr
    {
        Err e;
        uint16_t returnCode;
    };
    using DataRetVal = RetValT<Ref, std::span<uint8_t>>;
    using ExpectedDataResult = std::expected<DataRetVal, CmdErr>;
    ExpectedDataResult SendCommand(uint16_t cmd, const std::span<uint8_t> cmdData, std::span<uint8_t> respData);
    template<class T>
    ExpectedDataResult SendCommand(uint16_t cmd, T &&cmdData, frame_t &respData)
    {
        if constexpr (!std::is_same_v<T, std::span<uint8_t>>)
            return SendCommand(cmd, std::span<uint8_t>((uint8_t*)&cmdData, sizeof(T)), std::span<uint8_t>((uint8_t*)&respData, sizeof(frame_t)));
        else
            return SendCommand(cmd, cmdData, std::span<uint8_t>((uint8_t*)&respData, sizeof(frame_t)));
    }
    ExpectedResult SendFrame(const frame_t &frame);
    ExpectedDataResult RecvFrame(frame_t &frame);


    struct OpenCmdModeResponse
    {
        uint16_t protocol_version;
        uint16_t buffer_size;
    };
    using OpenCmdModeRetVal = RetValT<Ref, OpenCmdModeResponse>;
    using ExpectedOpenCmdModeResult = std::expected<OpenCmdModeRetVal, CmdErr>;
    using ExpectedCloseCmdModeResult = std::expected<Ref, CmdErr>;

    ExpectedOpenCmdModeResult OpenCommandMode();
    ExpectedCloseCmdModeResult CloseCommandMode();
};

#endif
